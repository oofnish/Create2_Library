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
                if i == 2 or i == grid_size-2 or j==2 or j==grid_size-2:
                    self.cells[i][j] = 2


        self.beacons = []

    def add_beacon(self, x, y, theta):
        self.beacons.append(Map.Beacon(x, y, theta))

    def c_size(self):
        return self.cell_size

    def cell(self, x, y):
        return self.cells[x][y]


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

    def poll(self, map, sensors, x, y, theta):
        def ray_query(sx, sy, st, dist, angle):
            sx = sx + dist * math.cos(st+angle)
            sy = sy + dist * math.sin(st+angle)

            grid_x = int(sx / map.c_size())
            grid_y = int(sy / map.c_size())
            if map.cell(grid_x, grid_y) == 2:
                return dist
            else:
                return 0

        sensors[61:63] = struct.pack(">H", ray_query(x, y, theta, 300, 0))
        sensors[63:65] = struct.pack(">H", ray_query(x, y, theta, 300, 0))



class MockCreate2(Create2):
    def __init__(self, port, baud=115200):
        self.sampling_rate = 0.015
        self.sleep_timer = 0.5
        self.vel_variance = 1.0
        self.song_list = {}
        self.D = 235

        # commanded velocity values
        self.desired_rvel = 0
        self.desired_lvel = 0

        # 'real' velocities that include variance.  Currently, assumes instantaneous acceleration
        self.real_rvel = 0
        self.real_lvel = 0

        # x, y, theta values calculated from real velocity
        self.x = 0
        self.y = 0
        self.theta = 0

        self.map = Map(21)
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

    def _update_position(self, dt):
        # Angular velocity of robot based on independent wheel velocities.
        omega = (self.real_rvel - self.real_lvel) / self.D

        if self.real_lvel != self.real_rvel:
            # non-equal velocities create an arc of movement
            r = self.D * (self.real_lvel + self.real_rvel) / (2 * (self.real_rvel - self.real_lvel))
            d_theta = omega * dt

            self.x = self.x + r * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y = self.y + r * (math.cos(self.theta) - math.cos(self.theta + d_theta))
            self.theta = self.theta + d_theta
        else:
            # equal velocities produce a linear-ish position calculation
            # velocity component
            v = (self.real_rvel + self.real_lvel) / 2
            self.x = self.x + v * dt * math.cos(self.theta)
            self.y = self.y + v * dt * math.sin(self.theta)

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
        # reallocation is faster than zeroing... though GC might be a concern
        mock_sensors = bytearray(80)

        self.sensors.poll(self.map, mock_sensors, self.x, self.y, self.theta)

        time.sleep(self.sampling_rate)
        sensors = SensorPacketDecoder(mock_sensors)

        return sensors
