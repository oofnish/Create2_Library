##############################################
# The MIT License (MIT)
# Copyright (c) 2021 Nick Stiffler
#############################################
# The following draws from:
#   Copyright (c) 2020 iRobot Corporation
#   Copyright (c) 2017 Kevin Walchko
##############################################
# The following is an interface between
#   - Create 2 and 
#   - pyserial 
# to be used in by our tkinter driver file 
#############################################
# Changelog:
#   + threading - repeatable lock for "locking" communication channel

import serial 
import struct
import threading

class SerialCommandInterface(object):
    """
    This class handles sending commands to the Create2. Writes will take in tuples
    and format the data to transfer to the Create.
    """

    def __init__(self):
        """
        Constructor.

        Creates the serial port, but doesn't open it yet. Call open(port) to open
        it.
        """
        self.ser = serial.Serial()
        self.lock = threading.RLock()

    def __del__(self):
        """
        Destructor.

        Closes the serial port
        """
        self.close()

    def open(self, port, baud=115200, timeout=1):
        """
        Opens a serial port to the create.

        port: the serial port to open, ie, '/dev/ttyUSB0'
        buad: default is 115200, but can be changed to a lower rate via the create api
        """
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = timeout

        # close the serial connection if it has already been opened
        if self.ser.is_open:
            self.ser.close()

        self.ser.open()
        if self.ser.is_open:
            # print("Create opened serial: {}".format(self.ser))
            print('-'*40)
            print(' Create opened serial connection')
            print('   port: {}'.format(self.ser.port))
            print('   datarate: {} bps'.format(self.ser.baudrate))
            print('-'*40)
        else:
            raise Exception(f"Failed to open {port} at {baud}")

    def write(self, opcode, data=None):
        """
        Writes a command to the create. There needs to be an opcode and optionally
        data. Not all commands have data associated with it.

        opcode: see create api
        data: a tuple with data associated with a given opcode (see api)
        """
        with self.lock:
            msg = (opcode,)

            # Sometimes opcodes don't need data. Since we can't add
            # a None type to a tuple, we have to make this check.
            if data:
                msg += data

            # print(">> write:", msg)
            self.ser.write(struct.pack('B' * len(msg), *msg))
            self.ser.flush()

    def read(self, num_bytes):
        """
        Read a string of 'num_bytes' bytes from the robot.

        Arguments:
            num_bytes: The number of bytes we expect to read.
        """
        if not self.ser.is_open:
            raise Exception("You must open the serial port first")

        with self.lock:
            data = self.ser.read(num_bytes)

        return data

    def flush(self):
        """
        Flush the input buffer, discarding all contents
        """
        if not self.ser.is_open:
            raise Exception("You must open the serial port first")

        self.ser.flushInput()

    def close(self):
        """
        Closes the serial connection.
        """
        if self.ser.is_open:
            print(f"Closing port {self.ser.port} @ {self.ser.baudrate}")
            self.ser.close()