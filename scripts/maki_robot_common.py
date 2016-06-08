## Common definitions for maki_robot

## NOTE: These globals are #define at the top of the Arduino servo driver
## MAKIv1_4_servo_controller_LITE.ino
INVALID_INT = 9999
DELIMITER_RECV = ':'	## syntax for FEEDBACK delimiter
TERM_CHAR_RECV = ';'	## syntax for end of FEEDBACK
TERM_CHAR_SEND = 'Z'	## syntax for end of servo command
SC_SET_GP = "GP"	## servo command syntax for setting a specified servo's GOAL POSITION
SC_SET_GS = "GS"	## servo command syntax for setting a specified servo's GOAL SPEED
SC_SET_IPT = "IPT"	## servo command syntax for setting the INTERPOLATION POSE TIME
SC_SET_TM = "TM"  	## servo command syntax for setting a specified servo's MAX TORQUE. Note: This value exists in EEPROM and persists when power is removed; default 1023
SC_SET_TL = "TL"  	## servo command syntax for setting a specified servo's TORQUE LIMIT. Note: If set to 0 by ALARM SHUTDOWN, servo won't move until set to another value[1,1023]; default MAX TORQUE
SC_SET_TS = "TS"  	## servo command syntax for setting a specified servo's TORQUE ENABLE. Note: Boolean [0, 1]; default 1. Doesn't appear to disable servo's movement when set to 0

SC_FEEDBACK = "F"	## servo command syntax for FEEDBACK or status of all servos
SC_GET_MX = "MX"	## servo command syntax for FEEDBACK of all servo MAXIMUM POSITION
SC_GET_MN = "MN"	## servo command syntax for FEEDBACK of all servo MINIMUM POSITION
SC_GET_PP = "PP"	## servo command syntax for feedback with PRESENT POSITION
SC_GET_GP = "GP"	## servo command syntax for feedback with GOAL POSITION
SC_GET_PS = "PS"	## servo command syntax for feedback with PRESENT SPEED
SC_GET_GS = "GS"	## servo command syntax for feedback with GOAL SPEED
SC_GET_PT = "PT"	## servo command syntax for feedback with PRESENT TEMPERATURE (in Celsius)
SC_GET_PL = "PL"	## servo command syntax for feedback with PRESENT LOAD
SC_GET_TM = "TM" 	## servo command syntax for feedback with MAX TORQUE
SC_GET_TL = "TL"  	## servo command syntax for feedback with TORQUE LIMIT
SC_GET_TS = "TS"  	## servo command syntax for feedback with TORQUE ENABLE
SC_GET_ER = "ER"  	## servo command syntax for feedback with error returned from AX_ALARM_LED
SC_GET_DP = "DP"  	## servo command syntax for default positions
SC_GET_DS = "DS"  	## servo command syntax for default speed
baud_rate = 19200	#9600

## from MAKIv14.h
SERVOCOUNT = 6	## MAKIv1.4 has 6 servos
LR = 1	## EYELID_RIGHT
LL = 2	## EYELID_LEFT
EP = 3	## EYE_PAN
ET = 4	## EYE_TILT
HT = 5	## HEAD_TILT
HP = 6	## HEAD_PAN
#F_VAL_SEQ = [ "LR", "LL", "EP", "ET", "HT", "HP" ]
F_VAL_SEQ = ( "LR", "LL", "EP", "ET", "HT", "HP" )	## tuples are immutable lists

## from MAKI-Arbotix-Interface.py
## servo control infix for type of feedback
FEEDBACK_SC = [ SC_GET_MX,
		SC_GET_MN,
		SC_GET_PP,
		SC_GET_GP,
		SC_GET_PS,
		SC_GET_GS,
		SC_GET_PT,
		SC_GET_PL,
		SC_GET_ER,
		SC_GET_DP,
		SC_GET_DS,
		SC_GET_TM,
		SC_GET_TL,
		SC_GET_TS
		 ]
FEEDBACK_TOPIC = [ "maki_feedback_max_pos",
		    	"maki_feedback_min_pos",
			"maki_feedback_pres_pos",
			"maki_feedback_goal_pos",
		     	"maki_feedback_pres_speed",
		     	"maki_feedback_goal_speed",
		     	"maki_feedback_pres_temp",
		     	"maki_feedback_pres_load",
		     	"maki_feedback_error",
		     	"maki_feedback_default_pos",
		     	"maki_feedback_default_speed",
	 	     	"maki_feedback_torque_max",
	 	     	"maki_feedback_torque_limit",
		     	"maki_feedback_torque_enable"
		]

IPT_FACE = 1000	#ms
IPT_NOD = 1750	#ms

HP_LEFT = 312
HP_FRONT = 512
HP_RIGHT = 712
#HP_LEFT = 404	## demo
#HP_RIGHT = 620	## demo

HT_UP = 540
HT_MIDDLE = 505
HT_DOWN = 460

HP_ALISSA = 620
HT_ALISSA = 540

EP_LEFT = 468
EP_FRONT = 512
EP_RIGHT = 556

ET_UP = 626
ET_MIDDLE = 512
ET_DOWN = 338

LL_OPEN_MAX = 535
LL_OPEN_DEFAULT = 500
LL_CLOSE_HALF = 430
LL_CLOSE_MAX = 361

###################################
#( "LR", "LL", "EP", "ET", "HT", "HP" )	
SERVO_MIN = ( 484, 361, 460, 425, 442, 256 )
SERVO_NEUTRAL = ( 525, 500, 512, 512, 512, 512 )
SERVO_MAX = ( 666, 535, 578, 610, 582, 768 )

DELTA_PP = 5
###################################

## More MAKI related global variables from _2015_05_14_KECK_MAKIv1_4_teleop.ino
#blink_time = [1000, 500, 250, 200, 150]	#, 100, 75]	## ms... 75ms looks even more realistic
blink_time = [2000, 1000, 800, 700, 600]		## ms
#eyelid_offset_blinking = -50	## LL decrement position
#eyelid_offset_open_wide = 30	## LL increment position

## Yet more MAKI related global variables; 2016-02-22
ht_tl_enable = 1023
ht_tl_disable = 0

falling_asleep_time = 3000
waking_up_time = 2000
resetting_time = 5000	#2000

## ktsui, INSPIRE 4, pilot3-1
slow_blink_time = 550	#1000	#ms	
#eye_saccade_time = [500, 250, 150, 100, 75, 50]	#ms
eye_saccade_time = [200, 150, 100, 75]	#ms	## 2016-03-08, ktsui: Scaz thinks somewhere around 150-100ms

