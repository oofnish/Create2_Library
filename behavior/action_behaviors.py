import math
import os

from behavior.action_sequence import EVENT_TYPE, UPDATE_RESULT
from behavior.pidcontrol import PID_Control
from createlib.create_oi import *

POLLING_PERIOD = 50

class Threshold:
    """
    A set of sensor thresholds used by actions for making behavior decisions
    """
    Clear = 15      # Clear threshold -- the maximum reading that we consider to be a zero value when looking for zero
    Found = 50      # Found threshold -- the minimum reading that we consider to be a nonzero value when looking for it
    Follow = 300    # Threshold for wall following distance


class Action:
    active = False

    def __init__(self):
        self.active = False

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

    def begin(self):
        pass

    def update(self, evt):
        if not self.active:
            return UPDATE_RESULT.PASS

    def cleanup(self):
        pass


class CollisionBackupAction(Action):
    def __init__(self, worldref, vel, t):
        super().__init__()
        self.world = worldref
        self.vel = vel
        self.total_t = t
        self.elapsed_t = 0
        self.delaying = False

    def begin(self):
        pass

    def activate(self):
        print("Activating Collision Backup")
        super().activate()
        self.elapsed_t = 0
        self.delaying = True
        self._stop_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(-self.vel, -self.vel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)

    def _time_elapsed(self, time):
        if not self.active:
            return UPDATE_RESULT.PASS
        self.elapsed_t += time
        if self.delaying and self.elapsed_t > 2000:
            self.delaying = False
            self._start_motion()
        elif self.delaying:
            return UPDATE_RESULT.BREAK

        if 0 < self.total_t <= self.elapsed_t:
            self.deactivate()
            return UPDATE_RESULT.OK

        # if we're active, and we haven't completed motion, prevent later actions from running
        return UPDATE_RESULT.BREAK

    def update(self, evt):
        sensors = self.world.sense()
        if sensors.charger_state == CHARGE_SOURCE.HOME_BASE:
            print("CHARGING (Collision)")
            self.deactivate()
            return UPDATE_RESULT.OK
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.COLLIDE:
                if not self.active:
                    self.active = True
                    self.activate()
                result = UPDATE_RESULT.BREAK
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result = UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                result = self._time_elapsed(evt.data)
            case _:
                result = UPDATE_RESULT.PASS
        return result


class TurnLeftToClearAction(Action):
    """
    A behavior that triggers when a wall is detected in front of the robot, causing the robot to rotate left until all
    Frontward (Center_* and Front_*) sensors report that they do not detect an obstacle.  As a result, the robot will
    be somewhat aligned to a wall on it's right side.

    This tactic works in many cases, but exhibits a problem when near a corner; in this case, the front_right sensor
    will clear early, leaving the bot facing around 45 degrees toward the corner.  In many cases, this is not enough to
    clear the corner and the bot will collide with it.
    """
    def __init__(self, worldref, vel):
        super().__init__()
        self.world = worldref
        self.vel = vel

    def begin(self):
        pass

    def activate(self):
        print("Activating Turn Left To Clear")
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        print("Deactivating turn left")
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(-self.vel, self.vel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)

    def _time_elapsed(self):
        if not self.active:
            return UPDATE_RESULT.PASS

        sensors = self.world.sense()

        if sensors.light_bumper_center_left < Threshold.Clear \
                and sensors.light_bumper_front_left < Threshold.Clear \
                and sensors.light_bumper_front_right < Threshold.Clear:
                #and sensors.light_bumper_center_right < Threshold.Clear:
            self.deactivate()
            return UPDATE_RESULT.OK

        # we're in the middle of turning since sensors haven't cleared yet, prevent later actions from running
        return UPDATE_RESULT.BREAK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FRONT:
                if not self.active:
                    self.activate()
                result = UPDATE_RESULT.BREAK
            case EVENT_TYPE.FINISH:
                self.deactivate()
                return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                result = self._time_elapsed()
            case _:
                if self.active:
                    result = UPDATE_RESULT.BREAK
                else:
                    result = UPDATE_RESULT.PASS
        return result


class TurnRightToClearAction(Action):
    """
    A behavior that triggers when a wall is detected directly in front, causing the robot to rotate right until the
    right wall disappears and reappears, at which time the action is complete.  While the turning action is in progress,
    later actions are skipped.

    As opposed to TurnLeftToClearAction, this approach to aligning to a wall is much more inefficient, but is more
    robust to cases where the robot is near a corner.
    """
    def __init__(self, worldref, vel):
        super().__init__()
        self.world = worldref
        self.vel = vel
        self.lostright = False

    def begin(self):
        pass

    def activate(self):
        print("Activating Turn Right To Clear")
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.lostright = False
        self._stop_motion()
        self.world.update_wheel_velocities(self.vel, -self.vel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)

    def _time_elapsed(self):
        if not self.active:
            return UPDATE_RESULT.PASS

        sensors = self.world.sense()

        # It is possible that at the start, the right sensor can see a wall, we need to watch for the wall to disappear
        # before we can start watching for it to come back
        if self.lostright is True:
            # watch for the right wall to return (likely after making around a 270 degree turn)
            if sensors.light_bumper_right > Threshold.Found:
                self._stop_motion()
                self.active = False
                return UPDATE_RESULT.OK

        # watch for the right wall to disappear at least once.
        if sensors.light_bumper_right < Threshold.Clear:
            self.lostright = True

        return UPDATE_RESULT.BREAK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FRONT:
                self.activate()
                result = UPDATE_RESULT.BREAK
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result = UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                result = self._time_elapsed()
            case _:
                result = UPDATE_RESULT.PASS
        return result


class WallFollowAction(Action):
    def __init__(self, worldref, baseVel):
        super().__init__()
        self.basevel = baseVel
        self.rvel = baseVel
        self.lvel = baseVel
        self.world = worldref

        self.controller = PID_Control(.1, 0.00004, .5, POLLING_PERIOD, 15)
        self.last_ff = False
        self.moving = False

    def begin(self):
        pass

    def activate(self):
        print("Activating Wall Follow")
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.rvel, self.lvel)
        self.moving = True

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        self.moving = False

    def _time_elapsed(self, time):
        sensors = self.world.sense()

        if sensors.ir_opcode > 160 or sensors.ir_opcode_right > 160:
            if self.last_ff:
                self.deactivate()
                return UPDATE_RESULT.DONE
            else:
                self.last_ff = True
        else:
            self.last_ff = False

        u = self.controller.PID(Threshold.Follow - sensors.light_bumper_right)

        dev = int(u)

        dev = max(min(dev, 100), -100)
        self.rvel = self.basevel + dev
        self.lvel = self.basevel - dev
        self._start_motion()

        return UPDATE_RESULT.OK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result= UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.moving:
                    self.activate()
                result = self._time_elapsed(evt.data)
            case _:
                result = UPDATE_RESULT.PASS
        return result


class DockingAction(Action):
    """
    Class to assist with docking the robot after detecting the
    docking station during a wall follow.

    Infared Character Values for Roomba Discover Drive-on Charger
    242 Force Field
    248 Red Buoy
    250 Red Buoy and Force Field
    252 Red Buoy and Green Buoy
    254 Red Buoy, Green Buoy, and Force Field
    246 Green Buoy and Force Field
    244 Green Buoy

    Roomba 600 Drive-on Charger
    161 Force Field
    168 Red Buoy
    169 Red Buoy and Force Field
    172 Red Buoy and Green Buoy
    173 Red Buoy, Greeen Buoy, and Force Field
    165 Green Buoy and Force Field
    164 Green Buoy
    """

    def __init__(self, worldref, baseVel):
        super().__init__()
        self.basevel = baseVel
        self.rvel = baseVel
        self.lvel = baseVel
        self.world = worldref
        self.dock_controller = PID_Control(.4, .01, .5, POLLING_PERIOD)
        self.moving = False


    def begin(self):
        pass

    def activate(self):
        print("Activating Docking")
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.rvel, self.lvel)
        self.moving = True

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        self.moving = False

    def _time_elapsed(self, time):
        sensors = self.world.sense()

        if sensors.charger_state == CHARGING_STATE.CHARGING_FAULT:
            print("CHARGING FAULT")

        if sensors.charger_state > CHARGING_STATE.NOT_CHARGING:
            print("CHARGING {}".format(sensors.charger_state))
            self.deactivate()
            return UPDATE_RESULT.DONE

        BIT_FF = 0x1
        BIT_GREEN = 0x4
        BIT_RED = 0x8

        error = 0
        s = 0
        if sensors.ir_opcode_left & BIT_GREEN:
            s = s | 0x1
            error = error - 1
        if sensors.ir_opcode_left & BIT_RED:
            s = s | 0x2
            error = error + 2
        if sensors.ir_opcode_right & BIT_RED:
            s = s | 0x4
            error = error + 1
        if sensors.ir_opcode_right & BIT_GREEN:
            s = s | 0x8
            error = error - 2
        if sensors.ir_opcode_right == sensors.ir_opcode_left == 0:
            if sensors.ir_opcode & BIT_GREEN:
                error = error - 3
            if sensors.ir_opcode & BIT_RED:
                error = error + 3

        # use left ir sensor to detect docking station when wall following with
        # red buoy and force field?
        u = self.dock_controller.PID(0 - error)

        dev = 2*int(u)

        dev = max(min(dev, 50), -50)
        self.rvel = self.basevel + dev
        self.lvel = self.basevel - dev
        self._start_motion()

        return UPDATE_RESULT.OK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result= UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.moving:
                    self.activate()
                result = self._time_elapsed(evt.data)
            case _:
                result = UPDATE_RESULT.PASS
        return result


class MoveToDocking(Action):
    """
    Class to assist with docking the robot after detecting the
    docking station during a wall follow.
    """

    def __init__(self, worldref, baseVel):
        super().__init__()
        self.basevel = baseVel
        self.rvel = baseVel
        self.lvel = baseVel
        self.world = worldref
        self.dock_controller = PID_Control(2, 0, 0, POLLING_PERIOD)
        self.moving = False

        self.turn_toward_dock = False

    def begin(self):
        pass

    def activate(self):
        print("Moving Toward Dock")
        self.turn_toward_dock = False
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.rvel, self.lvel)
        self.moving = True

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        self.moving = False

    def _time_elapsed(self, time):
        sensors = self.world.sense()

        BIT_FF = 0x1
        BIT_GREEN = 0x4
        BIT_RED = 0x8

        if not self.turn_toward_dock:
            if sensors.ir_opcode & BIT_GREEN and sensors.ir_opcode & BIT_RED == 0:
                self.turn_toward_dock = True
        elif sensors.ir_opcode & BIT_RED != 0:
                self.turn_toward_dock = False
                self.deactivate()
                return UPDATE_RESULT.DONE

        error = 0
        if sensors.ir_opcode_left & BIT_GREEN:
            error = error - 1
        if sensors.ir_opcode_left & BIT_RED:
            error = error + 2
        if sensors.ir_opcode_right & BIT_RED:
            error = error + 1
        if sensors.ir_opcode_right & BIT_GREEN:
            error = error - 2

        # use left ir sensor to detect docking station when wall following with
        # red buoy and force field?
        u = self.dock_controller.PID(0 - error)

        dev = int(u)
 #
        dev = max(min(dev, 50), -50)
        if not self.turn_toward_dock:
            self.rvel = self.basevel + dev
            self.lvel = self.basevel - dev
            self._start_motion()
        else:
            self.rvel = self.basevel + dev
            self.lvel = -self.basevel - dev
            self._start_motion()

        return UPDATE_RESULT.OK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result= UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.moving:
                    self.activate()
                result = self._time_elapsed(evt.data)
            case _:
                result = UPDATE_RESULT.PASS
        return result


class WanderAction(Action):
    def __init__(self, worldref, rvel, lvel):
        super().__init__()
        self.rvel = rvel
        self.lvel = lvel
        self.world = worldref
        self.complete = False

    def begin(self):
        pass

    def activate(self):
        print("Activating Wander")
        super().activate()
        self.complete = False
        self._start_motion()

    def deactivate(self):
        if not self.active:
            return
        super().deactivate()
        self.complete = True
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.rvel, self.lvel)
        self.moving = True

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        self.moving = False

    def _timer_elapsed(self):
        if self.complete:
            return UPDATE_RESULT.PASS

        if not self.active:
            self.activate()

        return UPDATE_RESULT.BREAK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FRONT:
                self.deactivate()
            case EVENT_TYPE.COLLIDE:
                self.deactivate()
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result = UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                result = self._timer_elapsed()
            case _:
                result = UPDATE_RESULT.PASS
        return result

class MoveTimeAction(Action):
    def __init__(self, worldref, rvel, lvel, t):
        super().__init__()
        self.rvel = rvel
        self.lvel = lvel
        self.total_t = t
        self.elapsed_t = 0
        self.paused = False
        self.world = worldref

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.rvel, self.lvel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        pass

    def activate(self):
        print("Activating Move Time")
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def begin(self):
        # send command to drive robot with the configured wheel velocities
        self._start_motion()

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FRONT:
                # halt wheel motion
                self.deactivate()
                self.paused = True
            case EVENT_TYPE.RESUME:
                # start wheel motion again
                if self.paused:
                    self.activate()
                self.paused = False
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result = UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.paused:
                    self.elapsed_t += evt.data
                    if 0 < self.total_t <= self.elapsed_t:
                        self._stop_motion()
                        result = UPDATE_RESULT.DONE
            case _:
                result = UPDATE_RESULT.PASS
        return result

    def cancel(self):
        if 0 < self.total_t <= self.elapsed_t:
            return True

    def duration(self):
        return self.total_t


class MoveDistanceAction(Action):
    def __init__(self, worldref, dist, vel=150):
        super().__init__()
        self.elapsed_t = 0
        self.vel = vel
        self.world = worldref
        #self.total_t = (dist/vel) * 1000
        self.total_t = self.world.get_move_time_bias(dist, vel)
        self.paused = False

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.vel, self.vel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        pass

    def activate(self):
        print("Activating Move Distance")
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def begin(self):
        # send command to drive robot with the configured wheel velocities
        self._start_motion()

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FRONT:
                # halt wheel motion
                self.deactivate()
                self.paused = True
            case EVENT_TYPE.RESUME:
                # start wheel motion again
                if self.paused:
                    self.activate()
                self.paused = False
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result = UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.paused:
                    self.elapsed_t += evt.data
                    if 0 < self.total_t <= self.elapsed_t:
                        self._stop_motion()
                        self.deactivate()
                        result = UPDATE_RESULT.OK
                    else:
                        result = UPDATE_RESULT.BREAK
            case _:
                result = UPDATE_RESULT.PASS
        return result


class Rotate90Action(Action):
    def __init__(self, worldref, direction, vel=150):
        super().__init__()
        self.elapsed_t = 0
        self.vel = vel
        self.world = worldref
        self.total_t = self.world.get_rotate_time_bias(math.pi/2, vel)
        self.dir = direction
        self.done = False

    def _start_motion(self):
        # send command to start robot with configured velocities
        if self.dir == "right":
            self.world.update_wheel_velocities(self.vel, -self.vel)
        else:
            self.world.update_wheel_velocities(-self.vel, self.vel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        pass

    def activate(self):
        print("Activating 90 Degree rotate {}".format(self.dir))
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self.done = True
        self._stop_motion()

    def begin(self):
        pass
        # send command to drive robot with the configured wheel velocities
        #self._start_motion()

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FINISH:
                self.deactivate()
                result = UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if self.done:
                    return UPDATE_RESULT.PASS
                if not self.active:
                    self.activate()
                if self.active:
                    self.elapsed_t += evt.data
                    if 0 < self.total_t <= self.elapsed_t:
                        self._stop_motion()
                        self.deactivate()
                        result = UPDATE_RESULT.OK
                    else:
                        result = UPDATE_RESULT.BREAK
            case _:
                result = UPDATE_RESULT.PASS
        return result