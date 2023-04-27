from behavior.action_sequence import EVENT_TYPE, UPDATE_RESULT
from behavior.pidcontrol import PID_Control


class Threshold:
    """
    A set of sensor thresholds used by actions for making behavior decisions
    """
    Clear = 15      # Clear threshold -- the maximum reading that we consider to be a zero value when looking for zero
    Found = 50      # Found threshold -- the minimum reading that we consider to be a nonzero value when looking for it
    Follow = 125    # Threshold for wall following distance


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

    def begin(self):
        pass

    def activate(self):
        super().activate()
        self.elapsed_t = 0
        self._start_motion()

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
        if 0 < self.total_t <= self.elapsed_t:
            self.deactivate()
            return UPDATE_RESULT.OK

        # if we're active, and we haven't completed motion, prevent later actions from running
        return UPDATE_RESULT.BREAK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.COLLIDE:
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
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
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
                and sensors.light_bumper_front_right < Threshold.Clear \
                and sensors.light_bumper_center_right < Threshold.Clear:
            self.deactivate()
            return UPDATE_RESULT.OK

        # we're in the middle of turning since sensors haven't cleared yet, prevent later actions from running
        return UPDATE_RESULT.BREAK

    def update(self, evt):
        result = UPDATE_RESULT.OK
        match evt.type:
            case EVENT_TYPE.FRONT:
                self.activate()
                result = UPDATE_RESULT.BREAK
            case EVENT_TYPE.FINISH:
                self.deactivate()
                return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                result = self._time_elapsed()
            case _:
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
        super().activate()
        self._start_motion()

    def deactivate(self):
        super().deactivate()
        self._stop_motion()

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.lostright = False
        print("RIGHT START")
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
        self.controller = PID_Control(6, .1, 30)
        self.moving = False

    def begin(self):
        pass

    def activate(self):
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
        u = self.controller.PID(Threshold.Follow - sensors.light_bumper_right)

        # HACK: bumper right == 0 causes too large a turn, so a crash happens... but we want it higher than normal
        # omega cap to make turns tight enough.  Robustify this
        if sensors.light_bumper_right == 0:
            self.rvel = self.basevel + 70  # dev
            self.lvel = self.basevel - 70  # dev
            self._start_motion()

        dev = int(u)

        dev = max(min(dev, 50), -50)
        self.rvel = self.basevel + dev
        self.lvel = self.basevel - dev
        self._start_motion()

        # # ugly logic
        # elif u > 0:
        #     dev = int(u)
        #     if dev > 50:
        #         dev = 50
        #     #dev=20
        #     self.rvel = self.basevel + dev
        #     self.lvel = self.basevel - dev
        #     self._start_motion()
        # elif u < 0:
        #     dev = int(u)
        #     if dev < -50:
        #         dev = -50
        #     #dev=-20
        #     self.rvel = self.basevel + dev
        #     self.lvel = self.basevel - dev
        #     self._start_motion()
        #
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
        super().activate()
        self.complete = False
        self._start_motion()

    def deactivate(self):
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

    def begin(self):
        # send command to drive robot with the configured wheel velocities
        self._start_motion()
        print("B", end="")

    def update(self, evt):
        if super().update(self, evt) == UPDATE_RESULT.PASS:
            return UPDATE_RESULT.PASS
        match evt.type:
            case EVENT_TYPE.FRONT:
                # halt wheel motion
                print("P", end="")
                self._stop_motion()
                self.paused = True
            case EVENT_TYPE.RESUME:
                # start wheel motion again
                if self.paused:
                    print("R", end="")
                    self._start_motion()
                self.paused = False
            case EVENT_TYPE.COLLIDE:
                print("E {}s {}m".format(self.elapsed_t / 1000, (self.elapsed_t * self.rvel) / (1000 * 1000)), end="")
                self._stop_motion()
                return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.paused:
                    self.elapsed_t += evt.data
                    if 0 < self.total_t <= self.elapsed_t:
                        print("E {}s {}m".format(self.elapsed_t / 1000, (self.elapsed_t * self.rvel) / (1000 * 1000)),
                              end="")
                        self._stop_motion()
                        return UPDATE_RESULT.DONE
                    print(".", end="")
            case _:
                print("")
        return UPDATE_RESULT.OK

    def cancel(self):
        if 0 < self.total_t <= self.elapsed_t:
            return True

    def duration(self):
        return self.total_t
