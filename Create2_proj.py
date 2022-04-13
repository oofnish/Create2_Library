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

import struct
import sys, glob # for listing serial ports
import time

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
    # static variables for keyboard callback -- I know, this is icky
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ''

    # Initialize a "Robot" object
    robot = None

    # Mapping of the supported keys (User should modify this to reflect key callbacks)
    supported_keys = {
                        "P": "Passive",
                        "S": "Safe",
                        "F": "Full",
                        "C": "Clean",
                        "D": "Dock",
                        "R": "Reset",
                        "Space": "Beep",
                        "Arrows": "Motion",
                        "Escape": "Quick Shutdown",
                        "B": "Print Sensors",
                     }

    # Project variables appear below this comment

    def help_text(self, key_dict):
        """
        Function that generates "help" based on the supplied Dictionary
        """
        ret_str = "Supported Keys:"
        for key, value in key_dict.items():
            ret_str += f"\n{key}\t{value}"
        ret_str += "\n\nIf nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.\n"
        return ret_str


    def _decorator(foo):
        def require_robot(self,*args, **kwargs):
            if self.robot is None:
                tkinter.messagebox.showinfo('Error', "Robot Not Connected!")
                return
            foo(self, *args, **kwargs)
        return require_robot


    def __init__(self):
        Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        createMenu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.text = Text(self, height = TEXTHEIGHT, width = TEXTWIDTH, wrap = WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=LEFT, fill=BOTH, expand=True)
        self.scroll.pack(side=RIGHT, fill=Y)

        self.text.insert(END, self.help_text(self.supported_keys))

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

    def prettyPrint(self, sensors):
        str = f"{'-'*70}\n"
        str += f"{'Sensor':>40} | {'Value':<5}\n"
        str += f"{'-'*70}\n"
        for k, v in sensors._asdict().items():
            str += f"{k}: {v}\n"
        return str



    @_decorator
    def callbackKey(self, event):
        """
        A handler for keyboard events. Feel free to add more!
        """
        k = event.keysym.upper()
        motionChange = False

        if event.type == '2': # KeyPress; need to figure out how to get constant
            if k == 'P':   # Passive
                print("Passive")
                self.robot.start()
            elif k == 'S': # Safe
                print("Safe")
                self.robot.safe()
            elif k == 'F': # Full
                print("Full")
                self.robot.full()
            elif k == 'C': # Clean
                self.robot.clean()
            elif k == 'D': # Dock
                self.robot.dock()
            elif k == 'SPACE': # Beep
                # self.sendCommandASCII('140 3 1 64 16 141 3')
                # setup beep as song 3
                beep_song = [64, 16]
                self.robot.createSong(3, beep_song)
                self.robot.playSong(3)
            elif k == 'R': # Reset
                self.robot.reset()
            elif k == 'B': # Print Sensors
                sensors = self.robot.get_sensors()
                sensor_str = self.prettyPrint(sensors)
                print(sensor_str)
            elif k == 'UP':
                self.callbackKeyUp = True
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = True
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = True
                motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = True
                motionChange = True
            elif k == 'ESCAPE':
                if self.robot is not None:
                    del self.robot
                self.destroy()
            else:
                print("not handled", repr(k))
        elif event.type == '3': # KeyRelease; need to figure out how to get constant
            if k == 'UP':
                self.callbackKeyUp = False
                motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = False
                motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = False
                motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = False
                motionChange = True

        if motionChange == True:
            velocity = 0
            velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
            velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
            rotation = 0
            rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
            rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0

            # compute left and right wheel velocities
            vr = int(velocity + (rotation/2))
            vl = int(velocity - (rotation/2))

            # create drive command

            cmd = (vl, vr)
            if cmd != self.callbackKeyLastDriveCommand:
                self.robot.drive_direct(vl, vr)
                self.callbackKeyLastDriveCommand = cmd

    def onConnect(self):
        if self.robot is not None:
            tkinter.messagebox.showinfo('Oops', "You're already connected to the robot!")
            return

        try:
            ports = self.getSerialPorts()
            port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        except EnvironmentError:
            port = tkinter.simpledialog.askstring('Port?', 'Enter COM port to open.')

        if port is not None:
            print("Trying " + str(port) + "... ")
            try:
                self.robot = cl.Create2(port=port, baud=115200)
                print("Connected!")
                tkinter.messagebox.showinfo('Connected', "Connection succeeded!")
            except Exception as e:
                print(f"Failed. Exception - {e}")
                tkinter.messagebox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))


    def onHelp(self):
        """
        Display help text
        """
        tkinter.messagebox.showinfo('Help', self.help_text(self.supported_keys))

    def onQuit(self):
        """
        Confirm whether the user wants to quit.
        """
        if tkinter.messagebox.askyesno('Really?', 'Are you sure you want to quit?'):
            if self.robot is not None:
                print("Robot object deleted")
                del self.robot
            self.destroy()

    def getSerialPorts(self):
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



    # ----------------------- Custom functions ------------------------------


    # ----------------------- Main Driver ------------------------------
if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
