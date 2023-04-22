#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import enum
import multiprocessing
import select
import threading
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

            "M":      KeyAction("Toggle Sensor Mode light/bump (default light sensors)", self.toggle_sensor_types, None),
            "N":      KeyAction("Toggle Cancel move or Pause move (cancel default)", self.toggle_pause_cancel, None),
            "G":      KeyAction("Begin Endless Forward Movement", self.move_endless, None),
            "T":      KeyAction("Begin Target Movement (distance=1 meter)", self.move_distance, None, press_arg=1000),
            "W":      KeyAction("Pause Movement and Wait", self.move_pause, None),
            "H":      KeyAction("Halt and Cancel Movement", self.move_halt, None),

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
                                press_arg=(VELOCITYCHANGE, 0), release_arg=(0, 0)),
            "DOWN":   KeyAction("", self.add_motion, self.add_motion,
                                press_arg=(-VELOCITYCHANGE, 0), release_arg=(0, 0)),
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

        self.actions = ActionSequence(500)
        self.bot_events = EventQueue()
        self.actions.register_event(self.bot_events)
        self.manually_paused = False

        self.sensor_poller = None
        # default light sensor mode
        self.bump_sensor_mode = False
        self.collision_event = EventFinish()

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
                pass
                tkinter.messagebox.showinfo('Error', "Robot Not Connected!")
                return
            foo(self, *args, **kwargs)
        return require_robot

    def need_sensors(foo):
        """
         rr is a decorator that imposes a requirement for a connection to a robot before launching a command
        """
        def need_sensors_inner(self, *args, **kwargs):
            if self.robot is not None:
                if self.sensor_poller is None:
                    self.sensor_poller = cl.RepeatTimer(.5, self.safe_drive_monitor, autostart=True)
            foo(self, *args, **kwargs)
        return need_sensors_inner

    def shutdown(self):
        """
        if the robot is configured (connected), this cleanly destroys the controller object before shutting down the ui
        """
        if self.sensor_poller:
            self.sensor_poller.stop()
            del self.sensor_poller
            self.sensor_poller = None
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
    def play_song(self, song_data):
        """
        Play a song on the robot.   If song_notes is None, then it just tries to play the song number.  Otherwise, it
        sets up a song using the given number and assigns it the note values in song_notes
        :param song_id: a song number for the robot
        :param song_notes: a list of song notes.  If None, only an existing song will attempt to play, otherwise it will
        configure the song before playing
        """
        if song_data[1]:
            self.robot.createSong(song_data[0], song_data[1])
        self.robot.playSong(song_data[0])

    @rr
    def add_motion(self, vel_rot):
        """
        key event that adds motion in a tuple to the velocity/rotation of the bot.  Primarily, this unwraps the argument
        tuple and forwards to send_motion
        :param vel_rot: tuple containing linear and angular acceleration (vel,rot)
        """
        self.robot.drive_direct(vel_rot[0], vel_rot[0])

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

    def toggle_pause_cancel(self):
        if isinstance(self.collision_event, EventFinish):
            print("Enabling movement pause upon collision")
            self.collision_event = EventPause()
        else:
            print("Enabling movement termination upon collision")
            self.collision_event = EventFinish()

    def toggle_sensor_types(self):
        if self.bump_sensor_mode:
            print("Enabling light sensor collision mode")
            self.bump_sensor_mode = False
        else:
            print("Enabling bump sensor collision mode")
            self.bump_sensor_mode = True


    @rr
    @need_sensors
    def move_endless(self):
        """
        Begin endless forward motion, that can be paused with bump/light sensors
        """
        self.actions.append(MoveTimeAction(self.robot, 200, 200, 0))


    @rr
    @need_sensors
    def move_distance(self, dist_in_mm):
        """
        Begin forward motion with a provided distance target.  The robot will move to the target, pausing if an obstacle
        is encountered, and halting when it reaches it or the halt event is produced.
        :param dist_in_mm: distance to move in millimeters
        """
        t = (dist_in_mm * 1000) / 200
        self.actions.append(MoveTimeAction(self.robot, 200, 200, t))


    @rr
    def move_halt(self):
        """
        Stop any forward movement by sending a terminate event to the action queue
        """
        self.bot_events.put(EventFinish)
        pass


    @rr
    def move_pause(self):
        """
        pause or resume motion manually
        """
        if not self.manually_paused:
            self.bot_events.put(EventPause())
            self.manually_paused = True
        else:
            self.bot_events.put(EventResume())
            self.manually_paused = False
        pass

    @rr
    def safe_drive_monitor(self):
        sensors = self.robot.get_sensors()
        pause = False

        if self.bump_sensor_mode:
            if sensors.bumps_wheeldrops.bump_left or sensors.bumps_wheeldrops.bump_right:
                pause = True
        else:
            if sensors.light_bumper.left \
                    or sensors.light_bumper.right \
                    or sensors.light_bumper.front_left \
                    or sensors.light_bumper.front_right \
                    or sensors.light_bumper.center_left \
                    or sensors.light_bumper.center_right:
                #print("LIGHT sensors triggered: {}, {}, {}, {}, {}, {}".format(
                #      sensors.light_bumper_left,
                #      sensors.light_bumper_right,
                #      sensors.light_bumper_front_left,
                #      sensors.light_bumper_front_right,
                #      sensors.light_bumper_center_left,
                #      sensors.light_bumper_center_right))
                pause = True

        if pause:
            self.bot_events.put(self.collision_event)
        else:
            if isinstance(self.collision_event, EventPause):
                self.bot_events.put(EventResume())

    @rr
    @need_sensors
    def right_hand_wall(self):
        # light bumper pids
        # pid_lb_left = PIDController(capacity=10, set_point=400, time=0.5, gain_p=2, gain_i=2, gain_d=2)
        pid_lb_front_left = PIDController(capacity=10, set_point=400, time=0.5, gain_p=2, gain_i=2, gain_d=2)
        pid_lb_center_left = PIDController(capacity=10, set_point=400, time=0.5, gain_p=2, gain_i=2, gain_d=2)
        pid_lb_center_right = PIDController(capacity=10, set_point=400, time=0.5, gain_p=2, gain_i=2, gain_d=2)
        pid_lb_front_right = PIDController(capacity=10, set_point=400, time=0.5, gain_p=2, gain_i=2, gain_d=2)
        pid_lb_right = PIDController(capacity=10, set_point=400, time=0.5, gain_p=2, gain_i=2, gain_d=2)

        # start moving until robot hits a wall - every time wall is hit, turn left 90 degrees and start moving
        # if error history is negative, turn left - if positive, turn right
        sensors = self.robot.get_sensors()
        pause = False

        index = pid_lb_front_left.add_sample(sensors.light_bumper_front_left)
        pid_lb_center_left.add_sample(sensors.light_bumper_center_left)
        pid_lb_center_right.add_sample(sensors.light_bumper_center_right)
        pid_lb_front_right.add_sample(sensors.light_bumper_front_right)
        pid_lb_right.add_sample(sensors.light_bumper_right)

        # adjust wheel velocity based on errors 
        if pid_lb_front_left.calculate_output(index) < 0 \
            or pid_lb_center_left.calculate_output(index) < 0 \
            or pid_lb_center_right.calculate_output(index) < 0 \
            or pid_lb_front_right.calculate_output(index) < 0:
            pause = True 
        if pause:
            self.bot_events.put(self.collision_event)
            # left_turn here
        else:
            if isinstance(self.collision_event, EventPause):
                self.bot_events.put(EventResume())

        


        # if sensors.light_bumper.left \
        #         or sensors.light_bumper.right \
        #         or sensors.light_bumper.front_left \
        #         or sensors.light_bumper.front_right \
        #         or sensors.light_bumper.center_left \
        #         or sensors.light_bumper.center_right:
            #print("LIGHT sensors triggered: {}, {}, {}, {}, {}, {}".format(
            #      sensors.light_bumper_left,
            #      sensors.light_bumper_right,
            #      sensors.light_bumper_front_left,
            #      sensors.light_bumper_front_right,
            #      sensors.light_bumper_center_left,
            #      sensors.light_bumper_center_right))
            

        
        

        # make a left turn and start following the wall. 

        # 
        # def adjust_left():

        # def adjust_right():

        # def right_turn():

        # def left_turn():

    


class World:
    """
    World is an internal representation of the environment around the robot, intended to allow the bot to query for
    information (eg, known obstacles, cost-to-go, metrics, etc.), and to allow the bot's experiences to update the
    representation with new data
    """

    def __init__(self):
        # min/max world extents, as viewed from above in 2 dimensions.  y-up is the along the inital forward direction
        # of the robot
        self.max_x = 0
        self.min_x = 0
        self.max_y = 0
        self.min_y = 0
        self.cur_x = 0
        self.cur_y = 0

        # statistics
        # total distance traveled
        self.total_dist = 0

class PIDController:
    # descrete time system PID controlelr

    def __init__(self, capacity: int, set_point: int, time: float, gain_p: float, gain_i: float, gain_d: float):
        self.set_point = set_point
        self.time = time
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d
        self.readings = [0]*capacity
        self.readings_errors = [0]*capacity
        self.current_index = 0

    def add_sample(self, state_sample):
        def calc_error(state_sample):
            return state_sample - self.set_point
        # add error calc to array for pid
        self.readings[self.current_index] = state_sample
        self.readings_errors[self.current_index] = calc_error(state_sample)
        # increment current index
        if self.current_index < len(self.readings)-1:
            self.current_index += 1
        else:
            self.current_index = 0
        
        return self.current_index


    def calculate_output(self, sample_index: int):
        # take a state sample and calculate PID output

        def calc_p():
            # print((self.gain_p*self.readings_errors[sample_index]))
            return (self.gain_p*self.readings_errors[sample_index])
        def calc_i():
            # print(self.gain_i*(self.time*sum(self.readings_errors)))
            return self.gain_i*(self.time*sum(self.readings_errors))
        def calc_d():
            if sample_index > 0:
                # print(self.gain_d*((self.readings[sample_index] - self.readings[sample_index - 1])/self.time))
                return self.gain_d*((self.readings[sample_index] - self.readings[sample_index - 1])/self.time)
            else:
                return self.gain_d*((self.readings[0] - self.readings[9])/self.time)

        # discrete pid calculation
        return calc_p() + calc_i() + calc_d()
    


class MoveTimeAction:
    def __init__(self, robotref, rvel, lvel, t):
        self.rvel = rvel
        self.lvel = lvel
        self.total_t = t
        self.elapsed_t = 0
        self.paused = False
        self.robot = robotref

    def _start_motion(self):
        # send command to start robot with configured velocities
        self.robot.drive_direct(self.rvel, self.lvel)

    def _stop_motion(self):
        # send command to stop robot
        self.robot.drive_stop()
        pass

    def begin(self):
        # send command to drive robot with the configured wheel velocities
        self._start_motion()
        print("B", end="")

    def update(self, evt):
        match evt.type:
            case EVENT_TYPE.PAUSE:
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
            case EVENT_TYPE.FINISH:
                print("E {}s {}m".format(self.elapsed_t/1000, (self.elapsed_t*self.rvel)/(1000*1000)), end="")
                self._stop_motion()
                return UPDATE_RESULT.DONE
            case EVENT_TYPE.TIMER:
                if not self.paused:
                    self.elapsed_t += evt.data
                    if 0 < self.total_t <= self.elapsed_t:
                        print("E {}s {}m".format(self.elapsed_t/1000, (self.elapsed_t*self.rvel)/(1000*1000)), end="")
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


class UPDATE_RESULT(enum.IntEnum):
    OK = 0x01
    DONE = 0x02
    TERMINATE = 0x03


class EVENT_TYPE(enum.IntEnum):
    MESSAGE = 0x01
    TIMER = 0x02
    PAUSE = 0x03
    RESUME = 0x04
    BEGIN = 0x05
    FINISH = 0x06


class EventBeginAction:
    type = EVENT_TYPE.BEGIN

    def __init__(self):
        pass


class EventMessage:
    type = EVENT_TYPE.MESSAGE

    def __init__(self, msg):
        self.data = msg


class EventTimer:
    type = EVENT_TYPE.TIMER

    def __init__(self, delta_t):
        self.data = delta_t


class EventPause:
    type = EVENT_TYPE.PAUSE

    def __init__(self):
        pass


class EventResume:
    type = EVENT_TYPE.RESUME

    def __init__(self):
        pass


class EventFinish:
    type = EVENT_TYPE.FINISH

    def __init__(self):
        pass


class EventQueue:
    def __init__(self):
        self.r, self.w = multiprocessing.Pipe(False)

    def put(self, evt):
        self.w.send(evt)

    def get(self):
        return self.r.recv()

    def fileno(self):
        return self.r.fileno()


class ActionSequence(threading.Thread):
    """
    ActionSequence manages a sequence of activities to be undertaken by the robot.  This is a serial list of actions,
    such as move forward, turn, move back, beep, wait, etc.
    Additionally, one or more EventQueues can be added, which can provide asynchronous signalling to the actions.  By
    default, a single "heartbeat" event queue is created, which triggers regularly, sending a time delta to the active
    action.  Other options for event queues include Cancel and Pause events.
    Upon startup, The ActionSequence runs the first configured action, then waits for events, using a select.select()
    call on all queues.
    """
    def __init__(self, polling_rate):
        self.actions = []
        self.current_action = 0
        self.event_queues = []
        self.exception_queues = []

        self.heartbeat = EventQueue()
        self.next_action = EventQueue()
        self.event_queues.append(self.heartbeat)
        self.event_queues.append(self.next_action)

        self.polling_rate = polling_rate

        super().__init__(daemon=True)
        self.start()

    def append(self, action):
        self.actions.append(action)
        if len(self.actions) == self.current_action + 1:
            self.next_action.put(EventBeginAction())

    def register_event(self, evt):
        self.event_queues.append(evt)

    def register_exception(self, exevt):
        self.exception_queues.append(exevt)

    def run(self):
        print("Starting ActionSequence")
        hb_timer = cl.RepeatTimer(self.polling_rate/1000, self.heartbeat.put, EventTimer(self.polling_rate), autostart=True)
        while True:

            evt_list, _, ex_list = select.select(self.event_queues, [], self.exception_queues)
            for e in ex_list:
                self.actions[self.current_action].cancel(e.get())
                # we only need to respond to the first exception?

                hb_timer.stop()
                return
            # Flag for whether we want to start the next action after all the events are processed. This is set to
            # True if any action update returns Done.  We use a flag to ensure all current events are forwarded to
            # the current action no matter what order they appear.
            start_next_action = False
            for e in evt_list:
                evt = e.get()
                if evt.type == EVENT_TYPE.BEGIN:
                    self.actions[self.current_action].begin()
                    continue

                if self.current_action > len(self.actions) - 1:
                    # no current action, so just consume incoming events
                    continue
                # pass the event details to the current action for processing
                result = self.actions[self.current_action].update(evt)
                match result:
                    case UPDATE_RESULT.OK:
                        # Nothing to do here, the update was all good
                        continue
                    case UPDATE_RESULT.DONE:
                        # Action is done, get ready to move to the next one
                        start_next_action = True
                    case UPDATE_RESULT.TERMINATE:
                        hb_timer.stop()
                        # We possibly want to post an exception here instead, for better cleanup
                        return
                    case _:
                        print("UNDEFINED RESULT!")
            # check our flag; if any of the events resulted in the action reaching completion, we want to go to the next
            if start_next_action:
                self.current_action += 1
                if len(self.actions) > self.current_action:
                    self.next_action.put(EventBeginAction())


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
