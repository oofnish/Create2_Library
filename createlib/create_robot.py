##############################################
# The MIT License (MIT)
# Copyright (c) 2017 Kevin Walchko
# Copyright (c) 2021 Nick Stiffler
# see LICENSE for full details
##############################################
# This is the main code for interacting with the Create 2
##############################################
# Changelog:
#   Fixed the getMode() function

import struct
import time
import math
from createlib.packets import SensorPacketDecoder, decode
from createlib.create_serial import SerialCommandInterface
from createlib.create_oi import OPCODES, SENSOR_PACKETS, DRIVE
from createlib.periodic_event import PeriodicEvent


class Create2(object):
    """
    The top level class for controlling a Create2.
    This is the only class that outside scripts should be interacting with.
    """

    def __init__(self, port, baud=115200):
        """
        Constructor, sets up class
        - creates serial port
        - creates decoder
        - sets the sampling_rate (15 ms)
        """
        self.SCI = SerialCommandInterface()
        self.SCI.open(port, baud)
        self.decoder = None
        self.sampling_rate = 0.015
        self.sleep_timer = 0.5
        self.song_list = {}

        # setup beep as song 4
        beep_song = [64, 16]
        self.createSong(4, beep_song)
        self.drive_direct_thread = PeriodicEvent(0, self.drive_direct, delegate_args=(500, 500), stop_fn=self.drive_stop)

    def __del__(self):
        """
        Destructor, cleans up when class goes out of scope
        """
        # stop motors
        self.drive_stop()
        time.sleep(self.sleep_timer)

        # turn off LEDs
        self.led()
        self.digit_led_ascii('    ')
        time.sleep(0.1)

        # close it down
        time.sleep(0.1)
        self.stop()  # power down, makes a low beep sound
        time.sleep(0.1)
        self.close()  # close serial port
        time.sleep(0.1)

    def close(self):
        """
        Closes up serial ports and terminates connection to the Create2
        """
        self.SCI.close()

    # ------------------- Mode Control ------------------------

    def start(self):
        """
        Puts the Create 2 into Passive mode.

        You must always send the Start commandbefore sending any other commands to the OI.
        """
        self.SCI.write(OPCODES.START)
        time.sleep(self.sleep_timer)

    def getMode(self):
        """
        Return the Mode
        """
        self.SCI.write(OPCODES.SENSORS, (SENSOR_PACKETS.OI_MODE,))
        time.sleep(self.sampling_rate)
        ans = self.SCI.read(1)
        if ans is not None and len(ans) == 1:
            byte = decode('unsigned_byte', ans)
        else:
            byte = 'Error, not mode returned'
        print(f"Mode: {byte}")

    def wake(self):
        """
        Wake up robot. See OI spec, pg 7 under passive mode.

        This should reset the 5 min timer in passive mode.
        Unfortunately, if you are using the "offical" create cable ...
        it doesn't work! They wired it wrong:
        https://robotics.stackexchange.com/questions/7895/irobot-create-2-powering-up-after-sleep
        """
        self.SCI.ser.rts = True
        self.SCI.ser.dtr = True
        time.sleep(1)
        self.SCI.ser.rts = False
        self.SCI.ser.dtr = False
        time.sleep(1)
        self.SCI.ser.rts = True
        self.SCI.ser.dtr = True
        time.sleep(1)  # Technically it should wake after 500ms.

    def reset(self):
        """
        This command resets the robot, as if you had removed and reinserted the
        battery. This command is buggy.
        ('Firmware Version:', 'bl-start\r\nSTR730\r\nbootloader id: #x47186549 82ECCFFF\r\nbootloader info rev: #xF000\r\nbootloader rev: #x0001\r\n2007-05-14-1715-L   \r')
        """
        self.clearSongMemory()
        self.SCI.write(OPCODES.RESET)
        time.sleep(1)
        ret = self.SCI.read(128)
        return ret

    def stop(self):
        """
        Puts the Create 2 into OFF mode. All streams will stop and the robot will no
        longer respond to commands. Use this command when you are finished
        working with the robot.
        """
        self.clearSongMemory()
        self.SCI.write(OPCODES.STOP)
        time.sleep(self.sleep_timer)

    def safe(self):
        """
        Puts the Create 2 into safe mode. Blocks for a short (<.5 sec) amount
        of time so the bot has time to change modes.
        """
        self.SCI.write(OPCODES.SAFE)
        time.sleep(self.sleep_timer)
        self.clearSongMemory()

    def full(self):
        """
        Puts the Create 2 into full mode. Blocks for a short (<.5 sec) amount
        of time so the bot has time to change modes.
        """
        self.SCI.write(OPCODES.FULL)
        time.sleep(self.sleep_timer)
        self.clearSongMemory()

    def power(self):
        """
        Puts the Create 2 into Passive mode. The OI can be in Safe, or
        Full mode to accept this command.
        """
        self.SCI.write(OPCODES.POWER)
        time.sleep(self.sleep_timer)

    def clean(self):
        """
        Activates the Create2 Clean mode
        """
        self.SCI.write(OPCODES.CLEAN)
        time.sleep(self.sleep_timer)

    def dock(self):
        """
        Create2 attempts to seek the dock
        """
        self.SCI.write(OPCODES.SEEK_DOCK)
        time.sleep(self.sleep_timer)
    # ------------------ Drive Commands ------------------

    def drive_stop(self):
        self.drive_direct(0,0)
        time.sleep(self.sleep_timer)  # wait just a little for the robot to stop

    def limit(self, val, low, hi):
        val = val if val < hi else hi
        val = val if val > low else low
        return val

    def drive_direct(self, l_vel, r_vel):
        """
        Drive motors directly: [-500, 500] mm/sec
        """
        l_vel = self.limit(l_vel, -500, 500)
        r_vel = self.limit(r_vel, -500, 500)
        data = struct.unpack('4B', struct.pack('>2h', r_vel, l_vel))  # write do this?
        self.SCI.write(OPCODES.DRIVE_DIRECT, data)

    def drive_pwm(self, r_pwm, l_pwm):
        """
        Drive motors PWM directly: [-255, 255] PWM
        """
        r_pwm = self.limit(r_pwm, -255, 255)
        l_pwm = self.limit(l_pwm, -255, 255)
        data = struct.unpack('4B', struct.pack('>2h', r_pwm, l_pwm))  # write do this?
        self.SCI.write(OPCODES.DRIVE_PWM, data)

    def drive_detect_light_sensors(self):
        # initial sensor check to make sure we're not at a wall
        sensors = self.get_sensors()
        if sensors.light_bumper_center_left < 1000 or \
            sensors.light_bumper_center_right < 1000 or \
            sensors.light_bumper_front_left < 1000 or \
            sensors.light_bumper_front_right < 1000:
            # start thread
            self.drive_direct_thread.start()
            self.drive_direct_thread.go()
                # loop to check for sensor 
        while self.drive_direct_thread.is_active:
            sensors = self.get_sensors()
            if sensors.light_bumper_center_left > 1000 or \
                sensors.light_bumper_center_right > 1000 or \
                sensors.light_bumper_front_left > 1000 or \
                sensors.light_bumper_front_right > 1000:
                self.drive_direct_thread.stop()
            time.sleep(0.05)

    def drive_direct_bump_wheel_drops(self):
        # initial sensor check to make sure we're not at a wall
        sensors = self.get_sensors()
        if not (sensors.bumps_wheeldrops.bump_left or sensors.bumps_wheeldrops.bump_right or
                sensors.bumps_wheeldrops.wheeldrop_right or sensors.bumps_wheeldrops.wheeldrop_left):
            self.drive_direct_thread.start()
            self.drive_direct_thread.go()
        while self.drive_direct_thread.is_active:
            if (sensors.bumps_wheeldrops.bump_left or sensors.bumps_wheeldrops.bump_right or
                    sensors.bumps_wheeldrops.wheeldrop_right or sensors.bumps_wheeldrops.wheeldrop_left):
                self.drive_direct_thread.stop()
            time.sleep(0.05)

    def distance_driving_light_sensors(self, distance_to_travel): 
        # initial sensor check to make sure we're not at a wall
        time_counter = 0
        sensors = self.get_sensors()
        if sensors.light_bumper_center_left < 1000 or \
            sensors.light_bumper_center_right < 1000 or \
            sensors.light_bumper_front_left < 1000 or \
            sensors.light_bumper_front_right < 1000:
            # start thread
            self.drive_direct_thread.start()
            self.drive_direct_thread.go()
            # start timer
            start_time = time.perf_counter_ns()
            current_distance = 0
            # loop to check for sensor 
            while self.drive_direct_thread.is_active and current_distance < distance_to_travel:
                sensors = self.get_sensors()
                if sensors.light_bumper_center_left > 1000 or \
                    sensors.light_bumper_center_right > 1000 or \
                    sensors.light_bumper_front_left > 1000 or \
                    sensors.light_bumper_front_right > 1000:
                    self.drive_direct_thread.stop()
                time.sleep(0.05)
                current_distance = ((time.perf_counter_ns() - start_time)*(math.pow(10, 9)))*500
            end_time = time.perf_counter()
            total_time = end_time - start_time
            return f"Total time of trip: {total_time}\n\n Approximate distance traveled with velocity of 500 mm/s: {current_distance} mm"
        else:
            return "Obstacle detected!\n\n Total time of trip: 0 \n\nDistance traveled: 0"


    # ------------------------ LED ----------------------------

    def led(self, led_bits=0, power_color=0, power_intensity=0):
        """
        led_bits: [check robot, dock, spot, debris]
        power_color: green [0] - red [255]
        power_instensity: off [0] - [255] full on

        All leds other than power are on/off.
        """
        data = (led_bits, power_color, power_intensity)
        self.SCI.write(OPCODES.LED, data)

    def light_toggle(self):
        """
        toggles light state between two values, tracked with light_state_a boolean
        """
        if self.light_state_a:
            self.led(led_bits=6)
            self.light_state_a = False
        else:
            self.led(led_bits=9, power_color=255, power_intensity=255)
            self.light_state_a = True

    def digit_led_ascii(self, display_string):
        """
        This command controls the four 7 segment displays using ASCII character codes.
        Arguments:
            display_string: A four character string to be displayed. This must be four
                characters. Any blank characters should be represented with a space: ' '
                Due to the limited display, there is no control over upper or lowercase
                letters. create2api will automatically convert all chars to uppercase, but
                some letters (Such as 'B' and 'D') will still display as lowercase on the
                Create 2's display. C'est la vie. Any Create non-printable character
                will be replaced with a space ' '.
        """
        display_list = [32]*4
        for i, c in enumerate(display_string[:4]):
            val = ord(c.upper())
            if 32 <= val <= 126:
                display_list[i] = val
            else:
                # Char was not available. Just print a blank space
                display_list[i] = 32

        self.SCI.write(OPCODES.DIGIT_LED_ASCII, tuple(display_list))

    # ------------------------ Songs ----------------------------

    def clearSongMemory(self):
        for sn in range(4):
            song = [70,0]
            self.createSong(sn,song)
            self.playSong(sn)
        time.sleep(0.1)

    def createSong(self, song_num, notes):
        """
        Creates a song
        Arguments
            song_num: 1-4
            notes: 16 notes and 16 durations each note should be held for (1 duration = 1/64 second)
        """
        size = len(notes)
        if (2 > size > 32) or (size % 2 != 0):
            raise Exception('Songs must be between 1-16 notes and have a duration for each note')
        if 0 > song_num > 3:
            raise Exception('Song number must be 0 - 3')

        if not isinstance(notes, tuple):
            notes = tuple(notes)

        dt = 0
        for i in range(len(notes)//2):
            dt += notes[2*i+1]
        dt = dt/64

        msg = (song_num, size//2,) + notes
        # print('>> msg:', (OPCODES.SONG,) + msg)
        self.SCI.write(OPCODES.SONG, msg)

        self.song_list[song_num] = dt

        return dt

    def playSong(self, song_num):
        """
        Play a song
            Arguments
                song_num: 0-4
            returns the song duration in seconds to sleep for
        """
        # if 0 > song_num > 3:
        #     raise Exception('Song number must be 0 - 3')

        # print('let us play', song_num)
        try:
            time_len = self.song_list[song_num]
        except:
            print("*** Invalid Song: {} ***".format(song_num))
            return 0

        # print('>> msg:', (OPCODES.PLAY, song_num,))
        self.SCI.write(OPCODES.PLAY, (song_num,))

        return time_len


    # ------------------------ Sensors ----------------------------

    def get_sensors(self):
        """
        return: a namedtuple
        WARNING: returns pkt 100, everything. And it is the default packet request now.
        """
        with self.SCI.lock:
            opcode = OPCODES.SENSORS
            cmd = (100,)
            sensor_pkt_len = 80

            self.SCI.flush()
            self.SCI.write(opcode, cmd)
            time.sleep(self.sampling_rate)  # wait 15 msec
            packet_byte_data = self.SCI.read(sensor_pkt_len)
            sensors = SensorPacketDecoder(packet_byte_data)

            return sensors
        

    def get_group_packet_3(self):
        sensors = self.get_sensors()
        # need packets 21-24
        # Charging State, Voltage, Current, Temperature, Battery Charge, Battery Capacity
        message = "Charging State: {}\n\nVoltage: {} mV\n\nCurrent: {} mA\n\nTemperature: {} C\n\nBattery Charge: {} mAh\n\nBattery Capacity: {} mAh".format(
            ["Not charging", "Reconditioning Charging", "Full Charging",
            "Trickle Charging", "Waiting", "Charging Fault Condition", "Communication Error!"][sensors.charger_state],
            sensors.voltage,
            sensors.current,
            sensors.temperature,
            sensors.battery_charge,
            sensors.battery_capacity
        )
        return message





