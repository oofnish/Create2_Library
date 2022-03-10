##############################################
# The MIT License (MIT)
# Copyright (c) 2017 Kevin Walchko
# Copyright (c) 2021 Nick Stiffler
# see LICENSE for full details
##############################################
# Changelog:
# 	This differs from other implementations as it uses the Python3 Enum class

##############################################

from enum import IntEnum 

# Note for students: 0x?? is equivalent to 0b????????, hex is used for compactness

class BAUD_RATE(IntEnum):
	BAUD_300    = 0
	BAUD_600    = 1
	BAUD_1200   = 2
	BAUD_2400   = 3
	BAUD_4800   = 4
	BAUD_9600   = 5
	BAUD_14400  = 6
	BAUD_19200  = 7
	BAUD_28800  = 8
	BAUD_38400  = 9
	BAUD_57600  = 10
	BAUD_115200 = 11

class DAYS(IntEnum):
	SUNDAY    = 0x01
	MONDAY    = 0x02
	TUESDAY   = 0x04
	WEDNESDAY = 0x08
	THURSDAY  = 0x10
	FRIDAY    = 0x20
	SATURDAY  = 0x40

class DRIVE(IntEnum):
	STRAIGHT     = 0x8000
	STRAIGHT_ALT = 0x7FFF
	TURN_CW      = -1
	TURN_CCW     = 0x0001

class MOTORS(IntEnum):
	SIDE_BRUSH           = 0x01
	VACUUM               = 0x02
	MAIN_BRUSH           = 0x04
	SIDE_BRUSH_DIRECTION = 0x08
	MAIN_BRUSH_DIRECTION = 0x10

class LEDS(IntEnum):
	DEBRIS       = 0x01
	SPOT         = 0x02
	DOCK         = 0x04
	CHECK_ROBOT  = 0x08

class WEEKDAY_LEDS (IntEnum):
	SUNDAY    = 0x01
	MONDAY    = 0x02
	TUESDAY   = 0x04
	WEDNESDAY = 0x08
	THURSDAY  = 0x10
	FRIDAY    = 0x20
	SATURDAY  = 0x40

class SCHEDULING_LEDS(IntEnum):
	COLON    = 0x01
	PM       = 0x02
	AM       = 0x04
	CLOCK    = 0x08
	SCHEDULE = 0x10

class RAW_LED(IntEnum):
	A = 0x01
	B = 0x02
	C = 0x04
	D = 0x08
	E = 0x10
	F = 0x20
	G = 0x40

class BUTTONS(IntEnum):
	CLEAN    = 0x01
	SPOT     = 0x02
	DOCK     = 0x04
	MINUTE   = 0x08
	HOUR     = 0x10
	DAY      = 0x20
	SCHEDULE = 0x40
	CLOCK    = 0x80

class ROBOT(IntEnum):
	TICK_PER_REV     = 508.8
	WHEEL_DIAMETER   = 72
	WHEEL_BASE       = 235
	TICK_TO_DISTANCE = 0.44456499814949904317867595046408

class MODES(IntEnum):
	OFF     = 0
	PASSIVE = 1
	SAFE    = 2
	FULL    = 3

class WHEEL_OVERCURRENT(IntEnum):
	SIDE_BRUSH  = 0x01
	MAIN_BRUSH  = 0x02
	RIGHT_WHEEL = 0x04
	LEFT_WHEEL  = 0x08

class BUMPS_WHEEL_DROPS   (IntEnum):
	BUMP_RIGHT       = 0x01 
	BUMP_LEFT        = 0x02
	WHEEL_DROP_RIGHT = 0x04
	WHEEL_DROP_LEFT  = 0x08

class CHARGE_SOURCE(IntEnum):
	INTERNAL  =0x01 
	HOME_BASE =0x02

class LIGHT_BUMPER(IntEnum):
	LEFT         = 0x01
	FRONT_LEFT   = 0x02
	CENTER_LEFT  = 0x04
	CENTER_RIGHT = 0x08
	FRONT_RIGHT  = 0x10
	RIGHT        = 0x20

class STASIS(IntEnum):
	TOGGLING = 0x01
	DISABLED = 0x02


class CHARGING_STATE(IntEnum):
	NOT_CHARGING     = 0
	RECONDITIONING   = 1
	FULL_CHARGING    = 2
	TRICKLE_CHARGING = 3 
	WAITING          = 4
	CHARGING_FAULT   = 5 

class OPCODES(IntEnum):
	# Getting started commands (pg 8)
    START   = 128
    RESET   = 7
    STOP    = 173
    BAUD    = 129
    # Mode Commands (pg 10)
    # CONTROL=130  # oi spec p 10 this is the same as SAFE
    SAFE = 131
    FULL = 132
    # cleaning commands (pg 11)
    CLEAN        = 135
    MAX          = 136
    SPOT         = 134
    SEEK_DOCK    = 143
    POWER        = 133
    SCHEDULE     = 167
    SET_DAY_TIME = 168
    # Actuator commands (page 13)
    DRIVE          = 137
    DRIVE_DIRECT   = 145
    DRIVE_PWM      = 146
    MOTORS         = 138
    MOTORS_PWM     = 144
    LED            = 139
    SCHEDULING_LED = 162
    # DIGIT_LED_RAW=163  # doesn't work
    BUTTONS         = 165
    DIGIT_LED_ASCII = 164      
    SONG            = 140
    PLAY            = 141
    # Input commands (page 21)
    SENSORS       = 142
    QUERY_LIST    = 149
    STREAM        = 148
    PAUSE_RESUME_STREAM = 150

class SENSOR_PACKETS(IntEnum):
	BUMPS_AND_WHEELDROPS      = 7
	WALL                      = 8 
	CLIFF_LEFT                = 9  
	CLIFF_FRONT_LEFT          = 10 
	CLIFF_FRONT_RIGHT         = 11 
	CLIFF_RIGHT               = 12 
	VIRTUAL_WALL              = 13 
	WHEEL_OVERCURRENTS        = 14 
	DIRT_DETECT               = 15 
	IR_OPCODE                 = 17 
	BUTTONS                   = 18 
	DISTANCE                  = 19 
	ANGLE                     = 20 
	CHARGE_STATE              = 21 
	VOLTAGE                   = 22 
	CURRENT                   = 23 
	TEMPERATURE               = 24 
	BATTERY_CHARGE            = 25 
	BATTERY_CAPACITY          = 26 
	WALL_SIGNAL               = 27 
	CLIFF_LEFT_SIGNAL         = 28 
	CLIFF_FRONT_LEFT_SIGNAL   = 29 
	CLIFF_FRONT_RIGHT_SIGNAL  = 30 
	CLIFF_RIGHT_SIGNAL        = 31 
	CHARGING_SOURCES          = 34 
	OI_MODE                   = 35 
	SONG_NUMBER               = 36 
	SONG_PLAYING              = 37 
	OI_STREAM_PACKET_SIZE     = 38 
	VELOCITY                  = 39 
	TURN_RADIUS               = 40 
	VELOCITY_RIGHT            = 41 
	VELOCITY_LEFT             = 42 
	ENCODER_LEFT              = 43 
	ENCODER_RIGHT             = 44 
	LIGHT_BUMPER              = 45 
	LIGHT_BUMP_LEFT           = 46 
	LIGHT_BUMP_FRONT_LEFT     = 47 
	LIGHT_BUMP_CENTER_LEFT    = 48 
	LIGHT_BUMP_CENTER_RIGHT   = 49 
	LIGHT_BUMP_FRONT_RIGHT    = 50 
	LIGHT_BUMP_RIGHT          = 51 
	IR_OPCODE_LEFT            = 52 
	IR_OPCODE_RIGHT           = 53 
	LEFT_MOTOR_CURRENT        = 54 
	RIGHT_MOTOR_CURRENT       = 55 
	MAIN_BRUSH_CURRENT        = 56 
	SIDE_BRUSH_CURRENT        = 57 
	STASIS                    = 58