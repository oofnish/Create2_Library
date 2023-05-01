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

        self.beacons = []

    def add_beacon(self, x, y, theta):
        self.beacons.append(Map.Beacon(x, y, theta))


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

    def poll(self, x, y, theta):
        pass


class MockCreate2(Create2):
    def __init__(self, port, baud=115200):
        self.sampling_rate = 0.015
        self.sleep_timer = 0.5
        self.song_list = {}

        # commanded velocity values
        self.desired_rvel = 0
        self.desired_lvel = 0

        # 'real' velocities that include variance.  Currently, assumes instantaneous acceleration
        self.real_rvel = 0
        self.real_xvel = 0

        # x, y, theta values calculated from real velocity
        self.x = 0
        self.y = 0
        self.theta = 0

        self.map = None
        self.sensors = None

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

    def drive_direct(self, l_vel, r_vel):
        pass

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

        time.sleep(self.sampling_rate)
        sensors = SensorPacketDecoder(mock_sensors)

        return sensors
