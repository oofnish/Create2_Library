#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Version History
# v1.0: Python2.7 -- 2015//05/27
# v2.0: Update to Python3 -- 2020/04/01
# v2.1: Stiffler (bare) modifications -- 2022/02/02
# v3.0: Stiffler Quality of Life changes

###########################################################################
# Copyright (c) 2015-2020 iRobot Corporation#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

from tkinter import *
import tkinter.messagebox
import tkinter.simpledialog

import os, sys, glob # for listing serial ports

# Create Library
import createlib as cl

try:
    import serial
except ImportError:
    tkinter.messagebox.showerror('Import error', 'Please install pyserial.')
    raise


TEXTWIDTH = 100 # window width, in characters
TEXTHEIGHT = 24 # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 300


class TetheredDriveApp(Tk):
    def config_commands_before_connect(self):
        """
        Configure commands available before a robot is connected.  This is mostly connect and quit operations
        """
        self.key_actions = {
            "C": KeyAction("Connect", self.on_connect, None),
            "Escape": KeyAction("Quick Shutdown", None, None),
            "ESCAPE": KeyAction("", self.shutdown, None),
        }

    def config_commands(self):
        """
        Configure commands for user interaction with the robot.
        """
        self.key_actions = {
            # Key action definition format:
            # KEY : Help text | KeyPress Callback | KeyRelease Callback | callback arguments (kwargs)
            "P":      KeyAction("Passive",  self.direct_command, None, press_arg=self.robot.start),
            "S":      KeyAction("Safe",     self.direct_command, None, press_arg=self.robot.safe),
            "F":      KeyAction("Full",     self.direct_command, None, press_arg=self.robot.full),
            "C":      KeyAction("Clean",    self.direct_command, None, press_arg=self.robot.clean),
            "D":      KeyAction("Dock",     self.direct_command, None, press_arg=self.robot.dock),
            "R":      KeyAction("Reset",    self.direct_command, None, press_arg=self.robot.reset),
            "B":      KeyAction("Print Sensors",    self.print_sensors, None),

            "Z":      KeyAction("Query Wall Signal/Cliff Signals", self.query_wall_cliff_signals, None),
            "Y":      KeyAction("Query Group Packet ID #3", self.query_group_3, None),
            "X":      KeyAction("LED Toggle", self.light_toggler_toggle, None),

            # The following actions are virtual, 'pretty output' items that do not correspond directly to actions, but
            # stand in for action groups or provide prettier name aliases
            "Space":  KeyAction("Beep", None, None),
            "Escape": KeyAction("Quick Shutdown", None, None),
            "Arrows": KeyAction("Motion", None, None),

            # The following actions have emtpy help text, therefore do not show in help. They perform the actions for
            # the 'pretty output' items above
            # "SPACE":  KeyAction("", self.send_command_ascii, None, press_arg='140 3 1 64 16 141 3'),
            "SPACE":  KeyAction("", self.play_song, None, press_arg=(3, [64, 16])),
            "ESCAPE": KeyAction("", self.shutdown, None),
            "UP":     KeyAction("", self.add_motion, self.add_motion,
                                press_arg=(VELOCITYCHANGE, 0), release_arg=(-VELOCITYCHANGE, 0)),
            "DOWN":   KeyAction("", self.add_motion, self.add_motion,
                                press_arg=(-VELOCITYCHANGE, 0), release_arg=(VELOCITYCHANGE, 0)),
            "LEFT":   KeyAction("", self.add_motion, self.add_motion,
                                press_arg=(0, ROTATIONCHANGE), release_arg=(0, -ROTATIONCHANGE)),
            "RIGHT":  KeyAction("", self.add_motion, self.add_motion,
                                press_arg=(0, -ROTATIONCHANGE), release_arg=(0, ROTATIONCHANGE)),
        }

    # Initialize a "Robot" object
    robot = None

    def __init__(self):
        Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        # key actions dictionary is filled by config_commands or config_commands_before_connect and contains the action
        # mappings
        self.key_actions = {}
        self.config_commands_before_connect()

        create_menu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=create_menu)

        create_menu.add_command(label="Connect", command=self.on_connect)
        create_menu.add_command(label="Help", command=self.on_help)
        create_menu.add_command(label="Quit", command=self.on_quit)

        self.text = Text(self, height = TEXTHEIGHT, width = TEXTWIDTH, wrap = WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=LEFT, fill=BOTH, expand=True)
        self.scroll.pack(side=RIGHT, fill=Y)

        self.text.insert(END, self.help_text({k: a.help for (k, a) in self.key_actions.items() if a.help != ""}))

        # if only one port is in the list of serial ports, try to auto connect to it.  This is disabled on failure
        self.auto_connect = True

        os.system('xset r off')
        self.bind("<Key>", self.cb_keypress)
        self.bind("<KeyRelease>", self.cb_keyrelease)

        # periodic light state switcher
        self.light_timer = None
        self.light_state_a = False

    def __del__(self):
        # re-enable the xwindows key repeat.  If this doesn't run, key repeat will be stuck off, and the resulting
        # suffering will lead to the dark side
        os.system('xset r on')

    # --- display helper functions ---

    @staticmethod
    def pretty_print(sensors):
        str = f"{'-'*70}\n"
        str += f"{'Sensor':>40} | {'Value':<5}\n"
        str += f"{'-'*70}\n"
        for k, v in sensors._asdict().items():
            str += f"{k}: {v}\n"
        return str

    @staticmethod
    def help_text(key_dict):
        """
        Function that generates "help" based on the supplied Dictionary
        """
        ret_str = "Supported Keys:"
        for key, value in key_dict.items():
            ret_str += f"\n{key}\t{value}"
        ret_str += "\n\nIf nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.\n"
        return ret_str

    # --- tkinter event handling functions ---

    def cb_keypress(self, event):
        """
        Key press handler callback function, actions are defined in the key actions structure above
        """
        k = event.keysym.upper()
        if k in self.key_actions:
            self.key_actions[k].press()

    def cb_keyrelease(self, event):
        """
        Key release handler callback function, actions are defined in the key actions structure above
        """
        k = event.keysym.upper()
        if k in self.key_actions:
            self.key_actions[k].release()

    def on_connect(self):
        if self.robot is not None:
            tkinter.messagebox.showinfo('Oops', "You're already connected to the robot!")
            return

        try:
            ports = self.get_serial_ports()
            if len(ports) == 1 and self.auto_connect:
                port = ports[0]
            else:
                port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        except EnvironmentError:
            port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.')

        if port is not None:
            print("Trying " + str(port) + "... ")
            try:
                self.robot = cl.Create2(port=port, baud=115200)
                print("Connected!")
                tkinter.messagebox.showinfo('Connected', "Connection succeeded!")
                self.text.delete("1.0", END)
                self.config_commands()
                self.text.insert(END, self.help_text({k: a.help for (k, a) in self.key_actions.items() if a.help != ""}))
            except Exception as e:
                print(f"Failed. Exception - {e}")
                self.auto_connect = False
                tkinter.messagebox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))

    def on_help(self):
        """
        Display help text
        """
        tkinter.messagebox.showinfo('Help', self.help_text({k: a.help for (k, a) in self.key_actions.items() if a.help != ""}))

    def on_quit(self):
        """
        Confirm whether the user wants to quit.
        """
        if tkinter.messagebox.askyesno('Really?', 'Are you sure you want to quit?'):
            if self.robot is not None:
                print("Robot object deleted")
                del self.robot
            self.destroy()

    @staticmethod
    def get_serial_ports():
        """
        Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    # --- robot command operations ---

    def rr(foo):
        """
        rr is a decorator that imposes a requirement for a connection to a robot before launching a command
        """
        def require_robot(self, *args, **kwargs):
            if self.robot is None:
                tkinter.messagebox.showinfo('Error', "Robot Not Connected!")
                return
            foo(self, *args, **kwargs)
        return require_robot

    def shutdown(self):
        """
        if the robot is configured (connected), this cleanly destroys the controller object before shutting down the ui
        """
        if self.robot:
            del self.robot
            self.robot = None
        self.destroy()

    @rr
    def print_sensors(self):
        """
        gets sensors from the robot prints their values to stdout
        """
        sensors = self.robot.get_sensors()
        sensor_str = self.pretty_print(sensors)
        print(sensor_str)

    @rr
    def direct_command(self, command_func):
        """
        send a simple direct command to the robot via the provided function.
        :param command_func: a command function such as self.robot.passive that does not take any arguments
        """
        command_func()

    @rr
    def play_song(self, song_id, song_notes):
        """
        Play a song on the robot.   If song_notes is None, then it just tries to play the song number.  Otherwise, it
        sets up a song using the given number and assigns it the note values in song_notes
        :param song_id: a song number for the robot
        :param song_notes: a list of song notes.  If None, only an existing song will attempt to play, otherwise it will
        configure the song before playing
        """
        if song_notes:
            self.robot.createSong(song_id, song_notes)
        self.robot.playSong(song_id)

    @rr
    def add_motion(self, vel_rot):
        """
        key event that adds motion in a tuple to the velocity/rotation of the bot.  Primarily, this unwraps the argument
        tuple and forwards to send_motion
        :param vel_rot: tuple containing linear and angular acceleration (vel,rot)
        """
        self.robot.drive_direct(vel_rot[0], vel_rot[1])

    @rr
    def query_wall_cliff_signals(self):
        sensors = self.robot.get_sensors()
        message = "Wall Signal: {}\n\nCliff Left Signal: {}\n\nCliff Front Left Signal: {}\n\nCliff Front Right Signal: {}\n\nCliff Right Signal: {}".format(
            sensors.wall_signal,
            sensors.cliff_left_signal,
            sensors.cliff_front_left_signal,
            sensors.cliff_front_right_signal,
            sensors.cliff_right_signal
        )

        tkinter.messagebox.showinfo("Wall Signal and Cliff Sensors", message)

    @rr
    def query_group_3(self):
        sensors = self.robot.get_sensors()

        message = "Charging State: {}\n\nVoltage: {} mV\n\nCurrent: {} mA\n\nTemperature: {} C\n\nBattery Charge: {} mAh\n\nBattery Capacity: {} mAh".format(
            ["Not charging", "Reconditioning Charging", "Full Charging",
             "Trickle Charging", "Waiting", "Charging Fault Condition", "Communication Error!"][sensors.charger_state],
            sensors.voltage,
            sensors.current,
            sensors.temperature,
            sensors.battery_charge,
            sensors.battery_capacity
        )

        tkinter.messagebox.showinfo("Group Packet #3", message)

    @rr
    def light_toggle(self):
        """
        toggles light state between two values, tracked with light_state_a boolean
        """
        if self.light_state_a:
            self.robot.led(6, 0, 0)
            self.light_state_a = False
        else:
            self.robot.led(9, 255, 255)
            self.light_state_a = True

    @rr
    def light_toggler_toggle(self):
        if self.light_timer is None:
            self.light_timer = cl.RepeatTimer(1.0, self.light_toggle, autostart=True)
        else:
            self.light_timer.stop()
            del self.light_timer
            self.light_timer = None


class KeyAction:
    def __init__(self, help_str, press_callback, release_callback, **kwargs):
        self.help = help_str
        self.press_callback = press_callback
        self.release_callback = release_callback

        self.press_argument = None
        self.release_argument = None

        self.enabled = True

        if "press_arg" in kwargs:
            self.press_argument = kwargs["press_arg"]

        if "release_arg" in kwargs:
            self.release_argument = kwargs["release_arg"]

    def press(self):
        """
        press handler for a key action, forwards to a defined callback if set
        """
        if self.press_callback is not None:
            if self.press_argument is None:
                self.press_callback()
            else:
                self.press_callback(self.press_argument)

    def release(self):
        """
        release handler for a key action, forwards to a defined callback if set
        """
        if self.release_callback is not None:
            if self.release_argument is None:
                self.release_callback()
            else:
                self.release_callback(self.release_argument)


if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
