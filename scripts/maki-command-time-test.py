#! /usr/bin/env python

#RUN AS: rosrun maki_robot maki_command_time_test.py

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import os

import math
from time import sleep
import sys
import string
import signal 

from timeit import default_timer as timer	## wall clock. Unix 1/100 second granularity

import threading
from array import *
import re		# see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python
import io

import random

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions

## subset of servo control infix for type of feedback
FEEDBACK_SC = [ #SC_GET_MX,
		#SC_GET_MN,
		SC_GET_PP,
		SC_GET_GP	#,
		#SC_GET_PS,
		#SC_GET_GS,
		#SC_GET_PT,
		#SC_GET_PL,
		#SC_GET_ER,
		#SC_GET_DP,
		#SC_GET_DS,
		#SC_GET_TM,
		#SC_GET_TL,
		#SC_GET_TS
		 ]
FEEDBACK_TOPIC = [ #"maki_feedback_max_pos",
		#     "maki_feedback_min_pos",
			"maki_feedback_pres_pos",
			"maki_feedback_goal_pos"	#,
		#     "maki_feedback_pres_speed",
		#     "maki_feedback_goal_speed",
		#     "maki_feedback_pres_temp",
		#     "maki_feedback_pres_load",
		#     "maki_feedback_error",
		#     "maki_feedback_default_pos",
		#     "maki_feedback_default_speed",
	 	#     "maki_feedback_torque_max",
	 	#     "maki_feedback_torque_limit",
		#     "maki_feedback_torque_enable"
		]

## GLOBAL VARIABLES FOR PYTHON SCRIPT
VERBOSE_DEBUG = False	## default is False
#ALIVE #= False		## make sure that all threads cleanly end
#INIT #= True		## flag to read first message from MAKI (will contain PP)
#makiPP
#makiGP

#last_blink_time = 0
#last_movement_time = 0

def presetGoalSpeed_ticks_duration( servo, ticks, s_duration ):
	global DC_helper
	_gs = int( DC_helper.getGoalSpeed_ticks_duration(ticks, s_duration) ) + 1
	if (_gs > 0) and (_gs < 1024):
		_pub_cmd = servo + str(SC_SET_GS) + str(_gs) + str(TERM_CHAR_SEND)
		rospy.logerr( str(_pub_cmd) )
		pubTo_maki_command( str(_pub_cmd) )
	elif (_gs == 0):
		rospy.logerr("setting GoalSpeed to 0 (unlimited) is not allowed!")
		rospy.logwarn("LL GoalSpeed is not changed")
	else:
		rospy.logerr("calculated GoalSpeed is out of bounds (0, 1023]; _gs = " + str(_gs))

def presetGoalSpeed_ticks_durationMS( servo, ticks, ms_duration ):
	presetGoalSpeed_ticks_duration( servo, ticks, float( ms_duration / 1000.0 ) )

#######################
## To run, publish to /maki_command
##	reset
##	blink_test
## To stop, publish to /maki_command
##	reset
#######################
def macroBlink():
	global mB_INTERUPT
	global ALIVE

	## this is a nested while loop
	_print_once = True
	while ALIVE and not rospy.is_shutdown():
		if _print_once:
			rospy.logdebug("Entering macroBlink outer while loop")
			_print_once = False

		if mB_INTERUPT:	
			#print "start sleep 5"
			sleep(5)	# 5 seconds
			#print "end sleep 5"
			continue	## begin loop again from the beginning skipping below
			#print "shouldn't get here"

		rospy.logdebug("Entering macroBlink inner while loop")
		_blink_count = 0
		while not mB_INTERUPT:

			for _blink_rep in range (1,2):	#range(1,6):	## [1,6)
				#_blink_count = helper_macroBlink( _blink_count, True, _blink_rep, read_time=2.0 )
				#_blink_count = helper_macroBlink( _blink_count, False, _blink_rep, read_time=2.0 )
				#_blink_count = helper_macroBlink( _blink_count, True, 3, read_time=0.5 )
				_blink_count = helper_macroBlink( _blink_count, False, 3, read_time=0.5 )
				sleep(2)
				rospy.logdebug("***************")

				## try to nicely end testing the eye blink
				if mB_INTERUPT:
					sleep(1)	## make sure to wait for message to reach Dynamixel servo
					pubTo_maki_command( "reset" )
					sleep(1)	## make sure to wait for message to reach Dynamixel servo
				else:
					pass
			# end	for _blink_rep in range()

		# end	while not mB_INTERUPT
	# end	while ALIVE and not rospy.is_shutdown():

def helper_macroBlink( blink_count, full_blink, blink_rep, read_time=1.0 ):
	global mB_INTERUPT
	global blink_time		## list of ints

	_start_blink_time = None
	_start_blink_close_time = None
	_finish_blink_close_time = None
	_start_blink_open_time = None
	_finish_blink_open_time = None
	_finish_blink_time = None
	_total_blink_time = 0

	## maki_command prefix
	_m_cmd_prefix = "LL" + SC_SET_GP

	rospy.loginfo("blink_rep=" + str(blink_rep))
	rospy.loginfo("-----------------")
	for _blink_time in blink_time:
		_half_blink_time = int( float(_blink_time) * 0.5 + 0.5 )
		rospy.loginfo( "blink_time=" + str(_blink_time) + "; half_blink_time=" + str(_half_blink_time) )

		#_close_blink_time = int( float(_blink_time) * 0.35 + 0.5 )
		#_open_blink_time = _blink_time - _close_blink_time

		## maki_command suffix
		#_m_cmd_suffix = SC_SET_IPT + str( _half_blink_time ) + TERM_CHAR_SEND
		_m_cmd_suffix = TERM_CHAR_SEND
		_blink_close = _m_cmd_prefix
		if full_blink:
			_blink_close += str(LL_CLOSE_MAX) + _m_cmd_suffix
			presetGoalSpeed_ticks_durationMS( "LL", abs(LL_CLOSE_MAX - LL_OPEN_DEFAULT), _half_blink_time )
		else:
			_blink_close += str(LL_CLOSE_HALF) + _m_cmd_suffix
			presetGoalSpeed_ticks_durationMS( "LL", abs(LL_CLOSE_HALF - LL_OPEN_DEFAULT), _half_blink_time )
		_blink_open = _m_cmd_prefix + str(LL_OPEN_DEFAULT) + _m_cmd_suffix
		
		_start_blink_time = timer()
		#for blink_rep in range(1, 6):
		for _flutter_count in range(0, blink_rep):
			rospy.loginfo( str(_flutter_count) )

			## blink close
			_start_blink_close_time = timer()
			rospy.logdebug( "blink close" )
			pubTo_maki_command( str(_blink_close) )
			sleepWhileWaitingMS( _half_blink_time, 0.01 )
			#sleepWhileWaitingMS( _close_blink_time, 0.01 )
			_finish_blink_close_time = timer()

			## blink open
			_start_blink_open_time = timer()
			rospy.logdebug( "blink open" )
			pubTo_maki_command( str(_blink_open) )
			sleepWhileWaitingMS( _half_blink_time, 0.01 )
			#sleepWhileWaitingMS( _open_blink_time, 0.01 )
			_finish_blink_open_time = timer()
			_finish_blink_time = timer()

			if mB_INTERUPT:
				rospy.logerr("Abort blink_test")
				break	## break out of inner for loop
		# end	for _flutter_count in range(1, blink_rep):
		_finish_blink_time = timer()

		## make it easier to read
		sleepWhileWaiting( read_time, .5 )

		blink_count += 1
		_total_blink_time = abs(_finish_blink_time - _start_blink_time)
		#rospy.loginfo( "Completed " + str(blink_count) + " full eye blinks" )
		rospy.loginfo( "Eye blink #" + str(blink_count) + ": full eye blink = " 
			+ str( _total_blink_time )
			+ "; blink open->close = " + str( abs(_finish_blink_close_time - _start_blink_close_time) )
			+ "; blink close->open = " + str( abs(_finish_blink_open_time - _start_blink_open_time) ) )

		rospy.loginfo("-----------------")

		## try to nicely end testing the eye blink
		if mB_INTERUPT:
			break	## break out of outer for loop
		else:
			## reset timers
			_start_blink_time = None
			_start_blink_close_time = None
			_finish_blink_close_time = None
			_start_blink_open_time = None
			_finish_blink_open_time = None
			_finish_blink_time = None
			_total_blink_time = 0

	# end	for _blink_time in blink_time:
	return blink_count

#######################
## To run, publish to /maki_command
##	reset
##	saccade_test
## To stop, publish to /maki_command
##	reset
#######################
def macroEyeSaccade():
	global mES_INTERUPT
	global ALIVE

	## this is a nested while loop
	_print_once = True
	while ALIVE and not rospy.is_shutdown():
		if _print_once:
			rospy.logdebug("Entering macroEyeSaccade outer while loop")
			_print_once = False

		if mES_INTERUPT:	
			#print "start sleep 5"
			sleep(5)	# 5 seconds
			#print "end sleep 5"
			continue	## begin loop again from the beginning skipping below
			#print "shouldn't get here"

		## where is MAKI's EP closest to currently? EP_LEFT, EP_FRONT, EP_RIGHT
		_ep_pp = makiPP["EP"]
		_delta_ep_pp = abs( _ep_pp - EP_FRONT )
		_ep_macro_pose = EP_FRONT
		if ( abs( _ep_pp - EP_LEFT ) < _delta_ep_pp ):
			_delta_ep_pp = abs( _ep_pp - EP_LEFT )
			_ep_macro_pose = EP_LEFT
		if ( abs( _ep_pp - EP_RIGHT ) < _delta_ep_pp ):
			_delta_ep_pp = abs( _ep_pp - EP_RIGHT )
			_ep_macro_pose = EP_RIGHT
		## publish GP of _ep_macro_pose in case in between poses
		pubTo_maki_command( "EP" + SC_SET_GP + str(_ep_macro_pose) + TERM_CHAR_SEND )

		## raise LL so that it doesn't impede saccade
		pubTo_maki_command( "LL" + SC_SET_GP + str(LL_OPEN_MAX) + TERM_CHAR_SEND )

		rospy.logdebug("Entering macroEyeSaccade inner while loop")
		_saccade_count = 0
		while not mES_INTERUPT:

			_saccade_count = helper_macroEyeSaccade( _saccade_count )

			## try to nicely end testing the eye saccade
			if mES_INTERUPT:
				sleep(1)	## make sure to wait for message to reach Dynamixel servo
				pubTo_maki_command( "reset" )
				sleep(1)	## make sure to wait for message to reach Dynamixel servo
			else:
				pass

		# end	while not mES_INTERUPT
	# end	while ALIVE and not rospy.is_shutdown():

def helper_macroEyeSaccade( saccade_count, read_time=1.0 ):
	global mES_INTERUPT
	global eye_saccade_time		## list of ints

	_start_saccade_time = None
	_start_saccade_side_time = None
	_finish_saccade_side_time = None
	_start_saccade_front_time = None
	_finish_saccade_front_time = None
	_finish_saccade_time = None
	_total_saccade_time = 0

	## maki_command prefix
	_m_cmd_prefix = "EP" + SC_SET_GP

	rospy.loginfo("-----------------")
	for _eye_saccade_time in eye_saccade_time:
		## maki_command suffix
		_m_cmd_suffix = SC_SET_IPT + str( _eye_saccade_time ) + TERM_CHAR_SEND
		_saccade_front_right = _m_cmd_prefix + str(EP_RIGHT) + _m_cmd_suffix
		_saccade_front_left = _m_cmd_prefix + str(EP_LEFT) + _m_cmd_suffix
		_saccade_front = _m_cmd_prefix + str(EP_FRONT) + _m_cmd_suffix

		## used for choosing direction of eye saccade
		## 0 = left, 1 = right
		_rand_saccade_right = random.randint(0,1)

		_start_saccade_time = timer()
		_start_saccade_side_time = timer()
		if _rand_saccade_right == 1:
			rospy.logdebug( "saccade RIGHT" )
			pubTo_maki_command( str(_saccade_front_right) )
		else:
			rospy.logdebug( "saccade LEFT" )
			pubTo_maki_command( str(_saccade_front_left) )
		sleepWhileWaitingMS( _eye_saccade_time, 0.01 )
		_finish_saccade_side_time = timer()
		_finish_saccade_time = timer()
		_total_saccade_time = abs(_finish_saccade_time - _start_saccade_time)

		## make it easier to read
		sleepWhileWaiting( read_time, 0.5 )

		_start_saccade_time = timer()
		## return to eye pan looking front
		_start_saccade_front_time = timer()
		pubTo_maki_command( str(_saccade_front) )
		sleepWhileWaitingMS( _eye_saccade_time, 0.01 )
		_finish_saccade_front_time = timer()
		_finish_saccade_time = timer()
		_total_saccade_time += abs(_finish_saccade_time - _start_saccade_time)

		## make it easier to read
		sleepWhileWaiting( read_time, .5 )

		saccade_count += 1
		#rospy.loginfo( "Completed " + str(saccade_count) + " full eye saccades" )
		rospy.loginfo( "Eye saccade #" + str(saccade_count) + ": full eye saccade = " 
			+ str( _total_saccade_time )
			+ "; eye saccade front->side = " + str( abs(_finish_saccade_side_time - _start_saccade_side_time) )
			+ "; eye saccade side->front = " + str( abs(_finish_saccade_front_time - _start_saccade_front_time) ) )
		rospy.loginfo("-----------------")

		## try to nicely end testing the eye saccade
		if mES_INTERUPT:
			break	## break out of for loop
		else:
			## reset timers
			_start_saccade_time = None
			_start_saccade_side_time = None
			_finish_saccade_side_time = None
			_start_saccade_front_time = None
			_finish_saccade_front_time = None
			_finish_saccade_time = None
			_total_saccade_time = 0

	# end	for _eye_saccade_time in eye_saccade_time:

	return saccade_count

#######################
## To run, publish to /maki_command
##	reset
##	HTTL1023Z
##	nod_test
## To stop, publish to /maki_command
##	reset
##	HTTL0Z
#######################
def macroHeadNod():
	global mHN_INTERUPT
	global ALIVE
	global makiPP
	global ht_tl_enable, ht_tl_disable

	## different versions of head nodding
	_v1 = False
	_v2 = True

	rospy.logdebug("macroHeadNod: BEGIN")

	_start_headnod_time = None
	_start_headnod_up_time = None
	_start_headnod_down_time = None
	_finish_headnod_up_time = None
	_finish_headnod_down_time = None
	_finish_headnod_time = None

	_headnod_max_min_dist = float( abs( HT_UP - HT_DOWN ) )
	_headnod_middle_down_dist = float( abs( HT_MIDDLE - HT_DOWN ) )
	_headnod_middle_up_dist = float( abs( HT_MIDDLE - HT_UP ) )
	## debugging
	#print "_headnod_max_min_dist = " 
	#print _headnod_max_min_dist 
	#print "str(" + str(_headnod_max_min_dist) + ")"
	##
	#print "_headnod_middle_down_dist = " 
	#print _headnod_middle_down_dist 
	#print "str(" + str(_headnod_middle_down_dist) + ")"
	##
	#print "_headnod_middle_up_dist = " 
	#print _headnod_middle_up_dist 
	#print "str(" + str(_headnod_middle_up_dist) + ")"
	##
	#print _headnod_middle_down_dist / _headnod_max_min_dist
	#print _headnod_middle_up_dist / _headnod_max_min_dist
	_ipt_nod_middle_down = float( (_headnod_middle_down_dist / _headnod_max_min_dist) * IPT_NOD ) 
	_ipt_nod_middle_up = float( (_headnod_middle_up_dist / _headnod_max_min_dist) * IPT_NOD ) 
	_ipt_nod_middle_down = int(_ipt_nod_middle_down + 0.5)	## implicit rounding
	_ipt_nod_middle_up = int(_ipt_nod_middle_up + 0.5)	## implicit rounding
	rospy.logdebug( "(full)\tIPT_NOD = " + str(IPT_NOD) + "ms" )
	rospy.logdebug( "(partial)\t_ipt_nod_middle_down = " + str(_ipt_nod_middle_down) + "ms" )
	rospy.logdebug( "(partial)\t_ipt_nod_middle_up = " + str(_ipt_nod_middle_up) + "ms" )
	rospy.logdebug( "(summed partial)\t_ipt_nod_middle_* = " + str( _ipt_nod_middle_down + _ipt_nod_middle_up ) + "ms" )

	## maki_command prefix
	_m_cmd_prefix = "HT" + SC_SET_GP
	## nod macros
	_nod_up_down = _m_cmd_prefix + str(HT_DOWN) + SC_SET_IPT + str(IPT_NOD) + TERM_CHAR_SEND
	_nod_down_up = _m_cmd_prefix + str(HT_UP) + SC_SET_IPT + str(IPT_NOD) + TERM_CHAR_SEND
	_nod_middle_up = _m_cmd_prefix + str(HT_UP) + SC_SET_IPT + str(_ipt_nod_middle_up) + TERM_CHAR_SEND 
	_nod_middle_down = _m_cmd_prefix + str(HT_DOWN) + SC_SET_IPT + str(_ipt_nod_middle_down) + TERM_CHAR_SEND 
	_nod_up_middle = _m_cmd_prefix + str(HT_MIDDLE) + SC_SET_IPT + str(_ipt_nod_middle_up) + TERM_CHAR_SEND 
	_nod_down_middle = _m_cmd_prefix + str(HT_MIDDLE) + SC_SET_IPT + str(_ipt_nod_middle_down) + TERM_CHAR_SEND 
	## this is a nested while loop
	_print_once = True
	while ALIVE and not rospy.is_shutdown():
		if _print_once:
			rospy.logdebug("Entering macroHeadNod outer while loop")
			_print_once = False

		if mHN_INTERUPT:	
			#print "start sleep 5"
			sleep(5)	# 5 seconds
			#print "end sleep 5"
			continue	## begin loop again from the beginning skipping below
			#print "shouldn't get here"

		## try to nicely startup headnod testing without jerking MAKI's head tilt servo
		pubTo_maki_command( "HT" + SC_SET_GP + str(makiPP["HT"]) + TERM_CHAR_SEND )
		sleep(1)	## make sure to wait for message to reach head tilt Dynamixel servo
		pubTo_maki_command( "HT" + SC_SET_TL + str(ht_tl_enable) + TERM_CHAR_SEND )
		sleep(1)	## make sure to wait for message to reach head tilt Dynamixel servo

		## where is MAKI's HT closest to currently? HT_UP, HT_MIDDLE, HT_DOWN
		_ht_pp = makiPP["HT"]
		_delta_ht_pp = abs( _ht_pp - HT_MIDDLE )
		_ht_macro_pose = HT_MIDDLE
		if ( abs( _ht_pp - HT_UP ) < _delta_ht_pp ):
			_delta_ht_pp = abs( _ht_pp - HT_UP )
			_ht_macro_pose = HT_UP
		if ( abs( _ht_pp - HT_DOWN ) < _delta_ht_pp ):
			_delta_ht_pp = abs( _ht_pp - HT_DOWN )
			_ht_macro_pose = HT_DOWN
		## publish GP of _ht_macro_pose in case in between poses
		pubTo_maki_command( "HT" + SC_SET_GP + str(_ht_macro_pose) + TERM_CHAR_SEND )

		if (_ht_macro_pose == HT_MIDDLE):	
			## looking up first has strongest nodding cue
			pubTo_maki_command( str(_nod_middle_up) )
			sleepWhileWaitingMS( _ipt_nod_middle_up )

		rospy.logdebug("Entering macroHeadNod inner while loop")
		_nod_count = 0
		while not mHN_INTERUPT:
			rospy.loginfo("-----------------")

			## VERSION 1: UP --> MIDDLE --> DOWN --> MIDDLE --> UP
			if _v1:
				_start_headnod_time = timer()
				_start_headnod_down_time = timer()
				pubTo_maki_command( str(_nod_up_middle) )
				sleepWhileWaitingMS( _ipt_nod_middle_up )

				pubTo_maki_command( str(_nod_middle_down) )
				sleepWhileWaitingMS( _ipt_nod_middle_down )

				pubTo_maki_command( str(_nod_middle_down) )
				sleepWhileWaitingMS( _ipt_nod_middle_down )
				_finish_headnod_down_time = timer()

				_start_headnod_up_time = timer()
				pubTo_maki_command( str(_nod_down_middle) )
				sleepWhileWaitingMS( _ipt_nod_middle_down )
		
				pubTo_maki_command( str(_nod_middle_up) )
				sleepWhileWaitingMS( _ipt_nod_middle_up )
				_finish_headnod_up_time = timer()
				_finish_headnod_time = timer()

			## VERSION 2: UP --> DOWN --> UP
			if _v2:
				_start_headnod_time = timer()
				_start_headnod_down_time = timer()
				pubTo_maki_command( str(_nod_up_down) )
				sleepWhileWaitingMS( IPT_NOD )
				_finish_headnod_down_time = timer()

				_start_headnod_up_time = timer()
				pubTo_maki_command( str(_nod_down_up) )
				sleepWhileWaitingMS( IPT_NOD )
				_finish_headnod_up_time = timer()
				_finish_headnod_time = timer()

			_nod_count += 1
			#rospy.loginfo( "Completed " + str(_nod_count) + " full head nods" )
			rospy.loginfo( "Head nod #" + str(_nod_count) + ": full nod = " 
				+ str( abs(_finish_headnod_time - _start_headnod_time) )
				+ "; nod up->down = " + str( abs(_finish_headnod_down_time - _start_headnod_down_time) )
				+ "; nod down->up = " + str( abs(_finish_headnod_up_time - _start_headnod_up_time) ) )
			rospy.loginfo("-----------------")


			## try to nicely end testing the headnod
			if mHN_INTERUPT:
				sleep(1)	## make sure to wait for message to reach head tilt Dynamixel servo
				pubTo_maki_command( "reset" )
				sleep(1)	## make sure to wait for message to reach head tilt Dynamixel servo
				pubTo_maki_command( "HT" + SC_SET_TL + str(ht_tl_disable) + TERM_CHAR_SEND )
		#end	while not mHN_INTERUPT:

	rospy.logdebug("macroHeadNod: END")


def macroHeadTurn():
	_face_right = "HP" + SC_SET_GP + str(HP_RIGHT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND
	_face_left = "HP" + SC_SET_GP + str(HP_LEFT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND
	_face_front = "HP" + SC_SET_GP + str(HP_FRONT) + SC_SET_IPT + str(IPT_FACE) + TERM_CHAR_SEND

#####################
def parseRecvMsg ( recv_msg ):
	global makiPP, makiGP
	global init_dict
	global FEEDBACK_SC
	global VERBOSE_DEBUG
	global feedback_template
	global start_movement_time, finish_movement_time
	global expected_movement_duration

	if VERBOSE_DEBUG:
		#rospy.logdebug( "parseRecvMsg: BEGIN" )
		rospy.logdebug( "Received: " + str(recv_msg) )

	tmp = feedback_template.search(recv_msg)
	if tmp != None:
		prefix = tmp.group(1)
		feedback_values = tmp.group(2)
		#print "Validated: prefix='" + prefix + "' and feedback_values='" + feedback_values + "'"
	else:
		rospy.logerr( "Received with ERROR! Invalid message format: " + str(recv_msg) )
		return	## return without an expression argument returns None. Falling off the end of a function also returns None

	values = re.findall("([0-9]+)", feedback_values)	## this is a list of strings
	## need to conver to int (see http://stackoverflow.com/questions/22672598/converting-lists-of-digits-stored-as-strings-into-integers-python-2-7) 
	tmp_dict = dict( zip(F_VAL_SEQ, map(int, values)) )

	if prefix=="PP":	#SC_GET_PP	# present position
		if not ( tmp_dict == makiPP ):
			makiPP.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiPP) )
		if not init_dict[prefix]:
			init_dict[prefix] = True
		else:
			finish_movement_time = timer()
			rospy.logdebug( "---- finish_movement_time = " + str(finish_movement_time) )

	elif prefix=="GP":	#SC_GET_GP	# goal position
		if not ( tmp_dict == makiGP ):
			makiGP.update(tmp_dict)
			if VERBOSE_DEBUG: rospy.logdebug( prefix + "=" + str(makiGP) )
		if not init_dict[prefix]:	init_dict[prefix] = True

	else:
		rospy.logerr( "parseRecvMsg: ERROR! Unknown prefix: " + str(prefix) )
		return

	#print "parseRecvMsg: END"


#####################
def updateMAKICommand( msg ):
	global maki_command
	global mHN_INTERUPT, mES_INTERUPT, mB_INTERUPT
	#rospy.logdebug(rospy.get_caller_id() + ": I heard %s", msg.data)

	## filter out feedback requests using regex
	_tmp = re.search("^F[A-Y]{2}Z$", msg.data)

	## YES, msg.data is a feedback request
	if _tmp != None:
		pass
	else:
		if (msg.data == "reset"):
			mHN_INTERUPT = True
			mES_INTERUPT = True
			mB_INTERUPT = True
			rospy.logdebug( "msg.data = " + msg.data )
		elif (msg.data == "nod_test"):
			mHN_INTERUPT = False
			rospy.logdebug( "msg.data = " + msg.data )
			rospy.logdebug( "mHN_INTERUPT = " + str(mHN_INTERUPT) )
		elif (msg.data == "saccade_test"):
			mES_INTERUPT = False
			rospy.logdebug( "msg.data = " + msg.data )
			rospy.logdebug( "mES_INTERUPT = " + str(mES_INTERUPT) )
		elif (msg.data == "blink_test"):
			mB_INTERUPT = False
			rospy.logdebug( "msg.data = " + msg.data )
			rospy.logdebug( "mB_INTERUPT = " + str(mB_INTERUPT) )
		else:
			maki_command = msg.data 
			parseMAKICommand( maki_command )

	return

def parseMAKICommand( m_cmd ):
	global start_movement_time, finish_movement_time
	global expected_movement_duration
	global mc_count

	_emd_print = ""

	_tmp = re.search("^([A-Y]+GP[0-9]+)+(IPT([0-9]+))*Z$", m_cmd)
	if _tmp != None:
		## if the start_movement_timer already exists
		if start_movement_time != None:
			finish_movement_time = timer()
			reportMovementDuration()

		start_movement_time = timer()
		rospy.logdebug( "----- start_movement_time = " + str(start_movement_time) )
		#print _tmp.group(1)
		#print _tmp.group(2)
		#print _tmp.group(3)

		if _tmp.group(3) != None:
			## convert milliseconds to float seconds
			expected_movement_duration = float(_tmp.group(3)) / 1000.0
			_emd_print = "expected_movement_duration = " + str(expected_movement_duration) 
		else:
			expected_movement_duration = None
			_emd_print = "expected_movement_duration NOT SPECIFIED" 

		mc_count += 1
		rospy.logdebug( "maki_command #" + str(mc_count) + " : " + str(m_cmd) )
		rospy.logdebug( str(_emd_print) )


#####################
def sleepWhileWaitingMS( ms_sleep_time, increment=0.25, early=True):
	## convert from IPT in milliseconds to sleep in seconds

	_new_ms_sleep_time = float(ms_sleep_time)/1000.0 
	if early:
		_new_ms_sleep_time = _new_ms_sleep_time - float(increment)
	
	#print "ms_sleep_time = " + str( ms_sleep_time )
	#print "increment = " + str( increment )
	#print "new_ns_sleep_time = " + str( _new_ms_sleep_time )
	sleepWhileWaiting( _new_ms_sleep_time, increment )

def sleepWhileWaiting( sleep_time, increment=1 ):
	global PIC_INTERUPT
	global ALIVE

	if VERBOSE_DEBUG: rospy.logdebug( "BEGIN: sleepWhileWaiting for " + str(sleep_time) + " seconds" )
	increment = max(0.01, increment)	## previous max values: 1.0, 0.25	## in seconds
	_start_sleep_time = timer()
	sleep(increment)	# ktsui, INSPIRE 4, pilot 2; emulate a do-while
	while ( ALIVE and
		not rospy.is_shutdown() and
		not PIC_INTERUPT and
		((timer() - _start_sleep_time) < sleep_time) ):
		sleep(increment)

	if VERBOSE_DEBUG: rospy.logdebug( "DONE: sleepWhileWaiting for " + str(sleep_time) + " seconds" )

#####################
def pubTo_maki_command( commandOut ):
	global VERBOSE_DEBUG
	global pub_cmd
	global maki_msg_format
	_pub_flag = False

	## make sure that commandOut ends in only one TERM_CHAR_SEND
	_tmp = re.search( maki_msg_format, commandOut )
	if _tmp != None:
		## Yes, commandOut ends in only one TERM_CHAR_SEND
		_pub_flag = True
		#if VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " matched maki_msg_format" )
	elif (commandOut == "reset"):
		## special case handled by MAKI-Arbotix-Interface.py driver
		_pub_flag = True
	elif not commandOut.endswith( str(TERM_CHAR_SEND) ):
		## append the missing TERM_CHAR_SEND
		commandOut += str(TERM_CHAR_SEND)
		_pub_flag = True
		if VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " added TERM_CHAR_SEND" )
	else:
		rospy.logerr( "Incorrect message format" + str(commandOut) )

	if VERBOSE_DEBUG: rospy.logdebug( str(commandOut) )

	if _pub_flag and not rospy.is_shutdown():
		pub_cmd.publish( commandOut )

#####################
def signal_handler(signal, frame):
	global ALIVE

	ALIVE = False
	sleep(1)
	sys.exit()

#####################
def parseFeedback( msg ):
	#rospy.logdebug(rospy.get_caller_id() + ": I heard %s", msg.data)
	parseRecvMsg ( msg.data )
	return

#####################
def initROS():
	global VERBOSE_DEBUG

	## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
	print str(_fname) + ": BEGIN"	## THIS IS BEFORE ROSNODE INIT

	# Initialize ROS node
        # see http://wiki.ros.org/rospy/Overview/Logging
        if VERBOSE_DEBUG:
                rospy.init_node('macro_time_test', log_level=rospy.DEBUG)
		rospy.logdebug("log_level=rospy.DEBUG")
        else:
                rospy.init_node('macro_time_test')       ## defaults to log_level=rospy.INFO
	rospy.logdebug( str(_fname) + ": END")
	return

def initPubMAKI():
	global pub_cmd
	global maki_msg_format

	## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
	rospy.logdebug( str(_fname) + ": BEGIN")

	## make sure that commandOut ends in only one TERM_CHAR_SEND
	maki_msg_format = "\A[a-yA-Y]+[a-yA-Y0-9]*"
	maki_msg_format += str(TERM_CHAR_SEND)
	maki_msg_format += "{1}$"

	# Setup publisher
	pub_cmd = rospy.Publisher("maki_command", String, queue_size = 10)

	rospy.logdebug( str(_fname) + ": END")
	return

def initSubFeedback():
	global feedback_template

	## get function name for logging purposes
        _fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
	rospy.logdebug( str(_fname) + ": BEGIN")

	## STEP 0: Setup regex template for expected feedback syntax
	feedback_msg_format = "\A([A-Z]{2})"	## 2 alphabetic char prefix
	feedback_msg_format += "(([0-9]+" + DELIMITER_RECV + "){" + str(SERVOCOUNT-1) + "}[0-9]+)" 
	feedback_msg_format += TERM_CHAR_RECV + "\Z"
	#print feedback_msg_format
	feedback_template = re.compile(feedback_msg_format)

	# STEP 1: Setup subscribers
	for topic in FEEDBACK_TOPIC:
		rospy.Subscriber( topic, String, parseFeedback )
		rospy.logdebug( "now subscribed to " + str(topic) )

	rospy.Subscriber( "/maki_command", String, updateMAKICommand )
	rospy.logdebug( "now subscribed to /maki_command" )

	rospy.logdebug( str(_fname) + ": END")
	return


#####################
def reportMovementDuration():
	global start_movement_time, finish_movement_time
	global expected_movement_duration

	if ((start_movement_time != None) and
		(finish_movement_time != None) and
		(expected_movement_duration != None)):
		rospy.loginfo( "maki_command #" + str(mc_count) + " : elapsed time = "
			+ str( abs(finish_movement_time - start_movement_time) ) 
			+ "; expected time = " + str(expected_movement_duration) )
		## reset
		resetTimer()


def resetTimer():
	global start_movement_time, finish_movement_time
	global expected_movement_duration

	start_movement_time = None
	finish_movement_time = None
	expected_movement_duration = None

#####################
if __name__ == '__main__':

	global ALIVE
	global VERBOSE_DEBUG
	global INIT, init_dict
	global makiPP, makiGP
	global maki_command
	global pub_cmd
	global start_movement_time, finish_movement_time
	global expected_movement_duration
	global mc_count
	global mHN_INTERUPT, mES_INTERUPT, mB_INTERUPT
	global PIC_INTERUPT
	global DC_helper

	## ---------------------------------
	## INITIALIZATION
	## ---------------------------------
	## STEP 0: INIT ROS
	initROS()

	## STEP 1: INIT GLOBAL VARIABLES
	ALIVE = False
	INIT = True
	init_dict = dict( zip(FEEDBACK_SC, [ False ] * len(FEEDBACK_SC) ) )
	makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	makiGP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )
	maki_command = ""
	mc_count = 0
	resetTimer()
	mHN_INTERUPT = True
	mES_INTERUPT = True
	mB_INTERUPT = True
	PIC_INTERUPT = False
	DC_helper = dynamixelConversions()

	## STEP 2: SIGNAL HANDLER
	#to allow closing the program using ctrl+C
	signal.signal(signal.SIGINT, signal_handler)

	## STEP 3: INIT ROBOT STATE
	## establish communication with the robot via rostopics
	initPubMAKI()
	initSubFeedback()
	## wait here if nothing has been published to rostopics yet
	while INIT and not rospy.is_shutdown():
		my_init = INIT
		for _servo_command in FEEDBACK_SC:
			#print _servo_command
			#print my_init
			my_init = my_init and init_dict[ _servo_command ]
			#print my_init
			sleep(0.5)	#0.5s
			pubTo_maki_command( str(SC_FEEDBACK + _servo_command + TERM_CHAR_SEND) )

		if not my_init:
			sleep(1.0)
		else:
			INIT = False
			rospy.loginfo( "Init MAKI state -- DONE" )

	## STEP 4: START THREAD FOR TESTING HEAD NODDING TIMING
	ALIVE = True
	thread_macroHeadNod = threading.Thread(target=macroHeadNod, args=())
	thread_macroHeadNod.setDaemon(True)	# make sure to set this; otherwise, stuck with this thread open
	thread_macroHeadNod.start()

	## STEP 5: START THREAD FOR TESTING EYE SACCADE TIMING
	thread_macroEyeSaccade = threading.Thread(target=macroEyeSaccade, args=())
	thread_macroEyeSaccade.setDaemon(True)	# make sure to set this; otherwise, stuck with this thread open
	thread_macroEyeSaccade.start()

	## STEP 6: START THREAD FOR TESTING BLINKING TIMING
	thread_macroBlink = threading.Thread(target=macroBlink, args=())
	thread_macroBlink.setDaemon(True)	# make sure to set this; otherwise, stuck with this thread open
	thread_macroBlink.start()

	## ---------------------------------
	## END OF INITIALIZATION
	## ---------------------------------

	resetTimer()
	ALIVE = True
	while ALIVE and not rospy.is_shutdown():
		reportMovementDuration()
		pass	## nop to allow stdin to pass through

	#rospy.spin()	## keeps python from exiting until this node is stopped

	print "__main__: END"



