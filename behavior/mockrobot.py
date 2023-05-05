import math

from createlib.create_robot import *


class Map:
    """
    Map is a grid cell representation of a virtual world, grid_size cells on a side.  If viewed from above, the bottom
    left grid corner would correspond to a coordinate 0, 0
    cell contents are 0 for free, 1 for occupied.  An occupied cell is considered to be completely full
    A map can also contain beacon objects that "emit" a force field in a gaussian falloff, and a pair of directional
    beams in a tight angle, with respect to the angle of the beacon.
    """
    class Beacon:
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta

    def __init__(self, grid_size, cell_size=406.4):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.cells = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

        for i, row in enumerate(self.cells):
            for j, cell in enumerate(row):
                if i == 4 or i == grid_size-5 or j == 4 or j == grid_size-5:
                    self.cells[i][j] = 2

        self.initial_position = [cell_size * grid_size / 2, cell_size * grid_size / 2, 0]
        self.beacons = []

    def add_beacon(self, x, y, theta):
        self.beacons.append(Map.Beacon(x, y, theta))

    def c_size(self):
        return self.cell_size

    def cell(self, x, y):
        return self.cells[x][y]

    def get_start(self):
        return self.initial_position[0], self.initial_position[1], self.initial_position[2]

    def ray_query(self, start, theta, max_distance):
        def lineIntersection(ray_start, ray_end, test_start, test_end):
            rx = ray_end[0] - ray_start[0]
            ry = ray_end[1] - ray_start[1]
            sx = test_end[0] - test_start[0]
            sy = test_end[1] - test_start[1]

            det = (-sx * ry + rx * sy)
            if abs(det) < 1e-10:
                return None
            s = (-ry * (ray_start[0] - test_start[0]) + rx * (ray_start[1] - test_start[1])) / det
            t = ( sx * (ray_start[1] - test_start[1]) - sy * (ray_start[0] - test_start[0])) / det
            if s >= 0 and s <= 1 and t >= 0 and t <= 1:
                return (ray_start[0] + (t * rx), ray_start[1] + (t * ry))

            return None

        # grid coordinate for starting pos
        sg_x = int(start[0]/self.cell_size)
        sg_y = int(start[1]/self.cell_size)

        # center coordinate of starting grid position
        ch = self.cell_size/2
        ccx = (sg_x+1)*self.cell_size - ch
        ccy = (sg_x+1)*self.cell_size - ch

        # small offset to pick correct grid coordinate after boundary intersection
        side_bump = [(1, 0), (0, 1), (-1, 0), (0, -1)]

        # horizontal and vertical grid lines for intersection
        grid_lines = [((ccx+ch, -1e6), (ccx+ch, 1e6)), ((-1e6, ccy+ch), (1e6, ccy+ch)),
                      ((ccx-ch, -1e6), (ccx-ch, 1e6)), ((-1e6, ccy-ch), (1e6, ccy-ch))]

        # endpoint for ray intersection test
        end = (start[0] + max_distance*math.cos(theta), start[1]+max_distance * math.sin(theta))

        closest = 1e6
        ret = [None, 0, 0, 0]
        for n, seg in enumerate(grid_lines):
            isect = lineIntersection(start, end, seg[0], seg[1])
            if isect:
                dist = math.sqrt((isect[0]-start[0])**2 + (isect[1]-start[1])**2)
                cs = (isect[0]+side_bump[n][0], isect[1]+side_bump[n][1])
                gx = int(cs[0]/self.cell_size)
                gy = int(cs[1]/self.cell_size)
                if self.cells[gx][gy] == 2:
                    if dist < closest:
                        closest = dist
                        ret = [isect, dist, gx, gy]

        return ret


class MockSensorSuite:
    def __init__(self):
        # variance bump and wheel drop sensors is a probability that they will not report an event that occurs
        self.v_bump = 0
        self.v_wheeldrop = [0]*2
        # variance in binary light bump sensors is a probability that a sensor will not report an even that's there.
        # note, the binary light bump sensors use the 'analog' light bump sensors, so those variances combine
        self.v_light_bin = [0]*6
        # variance in light bump sensors is a deviation from the ideal value
        self.v_lightbump = [0]*6
        # variance in ir sensors indicates the probability that they will not report a signal that's there.
        self.v_ir_omni = 0
        self.v_ir_left = 0
        self.v_ir_right = 0

        # light sensor range
        self.ls_range = 220
        self.ls_offset = 0
        self.ls_scale = 1024 / self.ls_range

    def sensor_center(self, x, y, theta):
        return x + self.ls_offset * math.cos(theta), y + self.ls_offset * math.sin(theta)

    def poll(self, real_map, sensors, x, y, theta):
        cx, cy = self.sensor_center(x, y, theta)

        def light_sensor(angle):
            dist = real_map.ray_query((cx, cy), theta+angle, self.ls_range)[1]
            dinv = self.ls_range - dist
            if dinv < 0 or dinv >= self.ls_range:
                dinv = 0
            return int(dinv*self.ls_scale)

            # sx = sx + dist * math.cos(st+angle)
            # sy = sy + dist * math.sin(st+angle)
            #
            # grid_x = int(sx / real_map.c_size())
            # grid_y = int(sy / real_map.c_size())
            # if real_map.cell(grid_x, grid_y) == 2:
            #     return dist
            # else:
            #     return 0


        #real_map.ray_query((x, y), theta+math.pi/12, 600)
        #sensors[61:63] = struct.pack(">H", ray_query(x, y, theta, 305, 0))
        #sensors[63:65] = struct.pack(">H", ray_query(x, y, theta, 305, 0))
        sensors[57:59] = struct.pack(">H", light_sensor(math.pi/2))
        sensors[59:61] = struct.pack(">H", light_sensor(math.pi/4))
        sensors[61:63] = struct.pack(">H", light_sensor(0))
        sensors[63:65] = struct.pack(">H", light_sensor(0))
        sensors[65:67] = struct.pack(">H", light_sensor(-math.pi/4))
        sensors[67:69] = struct.pack(">H", light_sensor(-math.pi/2))
        #sensors[57:59] = struct.pack(">H", int(real_map.ray_query((cx, cy), theta+math.pi/2, self.ls_range)[1]))
        #sensors[59:61] = struct.pack(">H", int(real_map.ray_query((cx, cy), theta+math.pi/4, self.ls_range)[1]))
        #sensors[61:63] = struct.pack(">H", int(real_map.ray_query((cx, cy), theta+math.pi/8, self.ls_range)[1]))
        #sensors[63:65] = struct.pack(">H", int(real_map.ray_query((cx, cy), theta-math.pi/8, self.ls_range)[1]))
        #sensors[65:67] = struct.pack(">H", int(real_map.ray_query((cx, cy), theta-math.pi/4, self.ls_range)[1]))
        #sensors[67:69] = struct.pack(">H", int(real_map.ray_query((cx, cy), theta-math.pi/2, self.ls_range)[1]))


class MockCreate2(Create2):
    def __init__(self, port, baud=115200):
        self.sampling_rate = 0.015
        self.sleep_timer = 0.5
        self.vel_variance = 1.0
        self.song_list = {}
        self.D = 235

        self.map = Map(21)

        # commanded velocity values
        self.desired_rvel = 0
        self.desired_lvel = 0

        # 'real' velocities that include variance.  Currently, assumes instantaneous acceleration
        self.real_rvel = 0
        self.real_lvel = 0

        # x, y, theta values calculated from real velocity
        self.x, self.y, self.theta = self.map.get_start()

        self.sensors = MockSensorSuite()

        self.last_update_time = time.time()

    def set_map(self, world_map):
        self.map = world_map

    def set_sensor_suite(self, sensor_suite):
        self.sensors = sensor_suite

    def close(self):
        pass

    def start(self):
        time.sleep(self.sleep_timer)

    def getMode(self):
        time.sleep(self.sampling_rate)

    def wake(self):
        pass

    def reset(self):
        return bytes('Firmware Version:', 'bl-start\r\nSTR730\r\nbootloader id: #x47186549 82ECCFFF\r\nbootloader info \
        rev: #xF000\r\nbootloader rev: #x0001\r\n2007-05-14-1715-L   \r')

    def stop(self):
        time.sleep(self.sleep_timer)

    def safe(self):
        time.sleep(self.sleep_timer)

    def full(self):
        time.sleep(self.sleep_timer)

    def power(self):
        time.sleep(self.sleep_timer)

    def clean(self):
        time.sleep(self.sleep_timer)

    def dock(self):
        time.sleep(self.sleep_timer)

    def drive_stop(self):
        time.sleep(self.sleep_timer)

    def limit(self, val, low, hi):
        val = val if val < hi else hi
        val = val if val > low else low
        return val

    def _get_predicted_position(self, dt=None):
        if not dt:
            current_time = time.time()
            dt = current_time - self.last_update_time

        omega = (self.real_rvel - self.real_lvel) / self.D

        if self.real_lvel != self.real_rvel:
            # non-equal velocities create an arc of movement
            r = self.D * (self.real_lvel + self.real_rvel) / (2 * (self.real_rvel - self.real_lvel))
            d_theta = omega * dt

            x = self.x + r * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            y = self.y + r * (math.cos(self.theta) - math.cos(self.theta + d_theta))
            theta = self.theta + d_theta
        else:
            # equal velocities produce a linear-ish position calculation
            # velocity component
            v = (self.real_rvel + self.real_lvel) / 2
            x = self.x + v * dt * math.cos(self.theta)
            y = self.y + v * dt * math.sin(self.theta)

            theta = self.theta

        return x, y, theta

    def _update_position(self, dt):

        self.x, self.y, self.theta = self._get_predicted_position(dt)
        # Angular velocity of robot based on independent wheel velocities.
        # omega = (self.real_rvel - self.real_lvel) / self.D
        #
        # if self.real_lvel != self.real_rvel:
        #     # non-equal velocities create an arc of movement
        #     r = self.D * (self.real_lvel + self.real_rvel) / (2 * (self.real_rvel - self.real_lvel))
        #     d_theta = omega * dt
        #
        #     self.x = self.x + r * (math.sin(self.theta + d_theta) - math.sin(self.theta))
        #     self.y = self.y + r * (math.cos(self.theta) - math.cos(self.theta + d_theta))
        #     self.theta = self.theta + d_theta
        # else:
        #     # equal velocities produce a linear-ish position calculation
        #     # velocity component
        #     v = (self.real_rvel + self.real_lvel) / 2
        #     self.x = self.x + v * dt * math.cos(self.theta)
        #     self.y = self.y + v * dt * math.sin(self.theta)

    def _set_velocities(self, l_vel, r_vel):
        self.desired_lvel = l_vel
        self.desired_rvel = r_vel
        self.real_lvel = self.desired_lvel * self.vel_variance
        self.real_rvel = self.desired_rvel * self.vel_variance

    def drive_direct(self, l_vel, r_vel):
        current_time = time.time()
        dt = current_time - self.last_update_time

        # update robot position
        self._update_position(dt)
        self._set_velocities(l_vel, r_vel)

        self.last_update_time = current_time

    def drive_pwm(self, r_pwm, l_pwm):
        pass

    def led(self, led_bits=0, power_color=0, power_intensity=0):
        pass

    def digit_led_ascii(self, display_string):
        pass

    def clearSongMemory(self):
        pass

    def createSong(self, song_num, notes):
        dt = 0
        for i in range(len(notes)//2):
            dt += notes[2*i+1]
        dt = dt/64
        self.song_list[song_num] = dt
        return dt

    def playSong(self, song_num):
        time_len = self.song_list[song_num]
        return time_len

    def get_sensors(self):
        x, y, theta = self._get_predicted_position()
        # reallocation is faster than zeroing... though GC might be a concern
        mock_sensors = bytearray(80)
        self.sensors.poll(self.map, mock_sensors, x, y, theta)

        time.sleep(self.sampling_rate)
        sensors = SensorPacketDecoder(mock_sensors)

        return sensors
