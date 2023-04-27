from behavior.action_sequence import EVENT_TYPE, UPDATE_RESULT
from behavior.pidcontrol import PID_Control


class CollisionBackupAction:
    def __init__(self, worldref, vel, t):
        self.active = False
        self.world = worldref
        self.vel = vel
        self.total_t = t
        self.elapsed_t = 0

    def begin(self):
        pass

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(-self.vel, -self.vel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)

    def update(self, evt):
        match evt.type:
            case EVENT_TYPE.COLLIDE:
                print("LEFT MOVE COLLIDE")
                self.active = True
                self._start_motion()
            case EVENT_TYPE.FINISH:
                if self.active:
                    self._stop_motion()
                    return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if self.active:
                    self.elapsed_t += evt.data
                    if 0 < self.total_t <= self.elapsed_t:
                        self._stop_motion()
                        self.active = False
                    else:
                        print("eating - backup")
                        return UPDATE_RESULT.BREAK
            case _:
                print("")
        return UPDATE_RESULT.OK


class TurnLeftToClearAction:
    def __init__(self, worldref, vel):
        self.active = False
        self.world = worldref
        self.vel = vel

    def begin(self):
        pass

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(-self.vel, self.vel)

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)

    def update(self, evt):
        match evt.type:
            case EVENT_TYPE.FRONT:
                self.active = True
                self._start_motion()
            case EVENT_TYPE.FINISH:
                if self.active:
                    self._stop_motion()
                    return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if self.active:
                    sensors = self.world.sense()
                    #print("TURN LEFT TURNING", [sensors.light_bumper_center_left, sensors.light_bumper_front_left, sensors.light_bumper_center_right])
                    if sensors.light_bumper_center_left < 20 \
                            and sensors.light_bumper_front_left < 20 \
                            and sensors.light_bumper_front_right < 20 \
                            and sensors.light_bumper_center_right < 20:
                        self._stop_motion()
                        self.active = False
                    else:
                        print("eating - left")
                        return UPDATE_RESULT.BREAK
            case _:
                print("miscevent", evt.type)
        return UPDATE_RESULT.OK


class TurnRightToClearAction:
    def __init__(self, worldref, vel):
        self.active = False
        self.world = worldref
        self.vel = vel
        self.lostright = False
        self.ismoving = False

    def begin(self):
        pass

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.lostright = False
        self.ismoving = True
        print("RIGHT START")
        self._stop_motion()
        self.world.update_wheel_velocities(self.vel, -self.vel)

    def _stop_motion(self):
        self.ismoving = False
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)

    def update(self, evt):
        match evt.type:
            case EVENT_TYPE.FRONT:
                self.active = True
                self._start_motion()
            case EVENT_TYPE.FINISH:
                if self.active:
                    self._stop_motion()
                    return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if self.active:

                    sensors = self.world.sense()
                    if self.lostright == True:
                        print("NoRight")
                        if sensors.light_bumper_right > 50:
                            print("SeeRight")
                            self._stop_motion()
                            self.active = False
                            return UPDATE_RESULT.OK

                    if sensors.light_bumper_right < 10:
                        print("Lostright")
                        self.lostright = True

                    return UPDATE_RESULT.BREAK

                    #print("TURN LEFT TURNING", [sensors.light_bumper_center_left, sensors.light_bumper_front_left, sensors.light_bumper_center_right])
                    # if sensors.light_bumper_center_left < 20 \
                    #         and sensors.light_bumper_front_left < 20 \
                    #         and sensors.light_bumper_front_right < 20 \
                    #         and sensors.light_bumper_center_right < 20:
                    #     self._stop_motion()
                    #     self.active = False
                    #else:
                    #    print("eating - right")
                    #    return UPDATE_RESULT.BREAK
            case _:
                print("miscevent", evt.type)
        return UPDATE_RESULT.OK


class WallFollowAction:
    def __init__(self, worldref, baseVel):
        self.basevel = baseVel
        self.rvel = baseVel
        self.lvel = baseVel
        self.elapsed_t = 0
        self.world = worldref
        self.controller = PID_Control(6, .1, 30)
        self.moving = False
        print("New PID Created")

    def begin(self):
        pass

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.rvel, self.lvel)
        self.moving = True

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        self.moving = False

    def update(self, evt):
        match evt.type:
            case EVENT_TYPE.FINISH:
                print("Motion stopping")
                self._stop_motion()
                return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.moving:
                    self._start_motion()
                print("follow timer ")

                sensors = self.world.sense()
                u = self.controller.PID(125 - sensors.light_bumper_right)

                # print("R:{}, PID:{}".format(sensors.light_bumper_right, u))
                if sensors.light_bumper_right == 0:
                    print("Lost Wall!")
                    self.rvel = self.basevel + 70  # dev
                    self.lvel = self.basevel - 70  # dev
                    self._start_motion()
                    # self._stop_motion()
                    # return UPDATE_RESULT.DONE

                elif u > 0:
                    dev = int(u)
                    print("r {}".format(dev))
                    if dev > 50:
                        dev = 50
                    dev=20
                    self.rvel = self.basevel + dev
                    self.lvel = self.basevel - dev
                    self._start_motion()
                elif u < 0:
                    dev = int(u)
                    print("l {}".format(dev))
                    if dev < -50:
                        dev = -50
                        dev=-20
                    self.rvel = self.basevel + dev
                    self.lvel = self.basevel - dev
                    self._start_motion()

            case _:
                print("")
        return UPDATE_RESULT.OK


class WanderAction:
    def __init__(self, worldref, rvel, lvel):
        self.rvel = rvel
        self.lvel = lvel
        self.world = worldref
        self.complete = False
        self.moving = False

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.world.update_wheel_velocities(self.rvel, self.lvel)
        self.moving = True

    def _stop_motion(self):
        # send command to stop robot
        self.world.update_wheel_velocities(0, 0)
        self.moving = False

    def begin(self):
        pass

    def update(self, evt):
        match evt.type:
            case EVENT_TYPE.FRONT:
                if not self.complete:
                    self._stop_motion()
                    self.complete = True
            case EVENT_TYPE.COLLIDE:
                self._stop_motion()
                self.complete = True
                return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.complete:
                    if not self.moving:
                        self._start_motion()
                    print("eating - wander")
                    return UPDATE_RESULT.BREAK
            #     if not self.paused:
            #         self.elapsed_t += evt.data
            #         if 0 < self.total_t <= self.elapsed_t:
            #             self._stop_motion()
            #             return UPDATE_RESULT.DONE
            #         print(".", end="")
            case _:
                print("")
        return UPDATE_RESULT.OK


class MoveTimeAction:
    def __init__(self, worldref, rvel, lvel, t):
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
