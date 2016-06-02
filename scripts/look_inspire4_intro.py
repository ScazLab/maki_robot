#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string
import random

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * 	## classes baseBehavior, headTiltBaseBehavior, headPanBaseBehavior
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Maki-ro's "lookAt" locations during the INSPIRE4 intro 
##
## Description of INSPIRE4 intro:
##	- START: Maki-ro is asleep
##	- Experimenter wakes Maki-ro up by touching shoulder
##	- Maki-ro wakes up
##	[*] Maki-ro looks at experimenter
##	- Experimenter waves hello
##	- Maki-ro bows its head in return
##	- Experimenter covers own eyes and plays peek-a-boo once
##	- Maki-ro responds with startle
##	- Maki-ro relaxes from startle
##	- Experimenter show Maki-ro strobing LED ball
##	- Maki-ro performs small, quick nod
##	- Experimenter moves ball to upper right calibration point
##	- Experimenter shakes the ball
##	[*] Maki-ro looks to (ball in) upper right calibration point
##	- Experimenter moves ball to lower right calibration point
##	- Experimenter shakes the ball
##	[*] Maki-ro looks to (ball in) lower right calibration point
##	- Experimenter moves ball in front of infant
##	- Experimenter shakes the ball
##	[*] Maki-ro looks to (ball in) front of infant
##	- Experimenter removes ball and leaves
##	- Maki-ro looks at infant, as if studying infant's face
########################
class lookINSPIRE4Intro( eyelidHeadTiltBaseBehavior, headPanBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	__is_intro_running = None
	__is_startled = None

	HP_EXPERIMENTER = None
	HT_EXPERIMENTER = None
	
	HP_BALL_LOWER_RIGHT = None
	HT_BALL_LOWER_RIGHT = None

	HP_BALL_UPPER_RIGHT = None
	HT_BALL_UPPER_RIGHT = None

	HP_ASLEEP = None
	HT_ASLEEP = None
	LL_ASLEEP = None

	HP_FACE_INFANT = None
	HT_FACE_INFANT = None

	FACING_EXPERIMENTER = None
	FACING_BALL_LOWER_RIGHT = None
	FACING_BALL_UPPER_RIGHT = None
	FACING_ASLEEP = None
	FACING_INFANT = None

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		eyelidHeadTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		headPanBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		if lookINSPIRE4Intro.__is_intro_running == None:
			lookINSPIRE4Intro.__is_intro_running = False

		## These values should match asleep_awake.py
		##	HP_AWAKE_EXP and HT_AWAKE_EXP
		if lookINSPIRE4Intro.HP_EXPERIMENTER == None:	
			lookINSPIRE4Intro.HP_EXPERIMENTER = 620	## ticks
		if lookINSPIRE4Intro.HT_EXPERIMENTER == None:	
			lookINSPIRE4Intro.HT_EXPERIMENTER = 570	## ticks

		if lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT == None:
			lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT = 404	## ticks
		if lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT == None:
			lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT = lookINSPIRE4Intro.HT_EXPERIMENTER

		if lookINSPIRE4Intro.HP_BALL_LOWER_RIGHT == None:
			lookINSPIRE4Intro.HP_BALL_LOWER_RIGHT = lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT
		if lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT == None:
			lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT = 480	## ticks

		if lookINSPIRE4Intro.HP_ASLEEP == None:
			lookINSPIRE4Intro.HP_ASLEEP = HP_LEFT
		if lookINSPIRE4Intro.HT_ASLEEP == None:
			lookINSPIRE4Intro.HT_ASLEEP = HT_DOWN
		if lookINSPIRE4Intro.LL_ASLEEP == None:
			lookINSPIRE4Intro.LL_ASLEEP = LL_CLOSE_MAX

		## NOTE: This should be the same value as lookINSPIRE4Interaction
		if lookINSPIRE4Intro.HP_FACE_INFANT == None:
			lookINSPIRE4Intro.HP_FACE_INFANT = HP_FRONT
		if lookINSPIRE4Intro.HT_FACE_INFANT == None:
			lookINSPIRE4Intro.HT_FACE_INFANT = HT_MIDDLE	

		if lookINSPIRE4Intro.FACING_EXPERIMENTER == None:
			lookINSPIRE4Intro.FACING_EXPERIMENTER = "experimenter"
		if lookINSPIRE4Intro.FACING_BALL_UPPER_RIGHT == None:
			lookINSPIRE4Intro.FACING_BALL_UPPER_RIGHT = "ballUpperRight"
		if lookINSPIRE4Intro.FACING_BALL_LOWER_RIGHT == None:
			lookINSPIRE4Intro.FACING_BALL_LOWER_RIGHT = "ballLowerRight"
		if lookINSPIRE4Intro.FACING_ASLEEP == None:
			lookINSPIRE4Intro.FACING_ASLEP = "asleep"
		if lookINSPIRE4Intro.FACING_INFANT == None:
			lookINSPIRE4Intro.FACING_INFANT = "infant"

		self.facing = None

		self.ipt_turn = 1000	## ms
		#self.delta_ht = 10	## ticks
		#self.ht_rand_min = lookINSPIRE4Intro.HT_EXPERIMENTER - self.delta_ht
		#self.ht_rand_max = lookINSPIRE4Intro.HT_EXPERIMENTER +- self.delta_ht

		## from lookINSPIRE4Intro
		if lookINSPIRE4Intro.__is_startled == None:
			lookINSPIRE4Intro.__is_startled = False
		self.HT_STARTLE = 525	#530	#525
		self.HT_NEUTRAL = HT_MIDDLE
		self.HT_GS_DEFAULT = 15		## as set in Arbotix-M driver
		self.HT_GS_MAX = 75	#60	#50
		self.HT_GS_MIN = 10

		self.LL_STARTLE = LL_OPEN_MAX
		self.LL_NEUTRAL = LL_OPEN_DEFAULT
		self.LL_GS_DEFAULT = 100	## as set in Arbotix-M driver
		self.LL_GS_MIN = 10

		### Game variables
		#if lookINSPIRE4Intro.__is_game_running == None:
		#	lookINSPIRE4Intro.__is_game_running = False
		#if lookINSPIRE4Intro.__is_game_exit == None:
		#	lookINSPIRE4Intro.__is_game_exit = False
		#self.ll_startle = LL_OPEN_MAX
		#
		#self.last_startle_time = None
		#self.next_startle_time = None
		#
		#self.game_state = None
		#
		#self.repetitions = 6	## do 6 rounds of infant engagement behavior max
		#
		#self.duration_between_startle_min = 2.0	## seconds
		#self.duration_between_startle_max = 5.0	## seconds

		lookINSPIRE4Intro.calculatePose( self )

		self.ALIVE = True
		return

	## override base class
	def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.5):
		rospy.logdebug("lookINSPIRE4Intro.pubTo_maki_command(): BEGIN")
		## call base class' pubTo_maki_command
		headPanBaseBehavior.pubTo_maki_command( self, commandOut, fixed_gaze=fixed_gaze, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )
		rospy.logdebug("lookINSPIRE4Intro.pubTo_maki_command(): END")
		return

	## override base class
	def start( self, enable_ht=True ):
		## call base class' start
		eyelidHeadTiltBaseBehavior.start( self, enable_ht=enable_ht )

		## CHECK TO SEE WHICH POSITION IS CLOSEST
		lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
		_hp_pp = self.makiPP["HP"]	
		_ht_pp = self.makiPP["HT"]
		_ll_pp = self.makiPP["LL"]
		_delta_pp = DELTA_PP
		if (abs(_hp_pp - lookINSPIRE4Intro.HP_FACE_INFANT) < _delta_pp):
			self.facing = lookINSPIRE4Intro.FACING_INFANT
			rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing infant")

		elif (abs(_hp_pp - lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT) < _delta_pp):
			if (abs(_ht_pp - lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT) < _delta_pp):
				self.facing = lookINSPIRE4Intro.FACING_BALL_UPPER_RIGHT
				rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing upper right calibration point")
			elif ((abs(_ht_pp - lookINSPIRE4Intro.HT_ASLEEP) < _delta_pp) and
				(abs(_ll_pp - lookINSPIRE4Intro.LL_ASLEEP) < _delta_pp)):
				self.facing = lookINSPIRE4Intro.FACING_ASLEEP
				rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing asleep")
			elif (abs(_ht_pp - lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT) < _delta_pp):
				self.facing = lookINSPIRE4Intro.FACING_BALL_LOWER_RIGHT
				rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing lower right calibration point")
			else:
				rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing toward rightScreen")

		elif (abs(_hp_pp - lookINSPIRE4Intro.HP_EXPERIMENTER) < _delta_pp):
			if (abs(_ht_pp - lookINSPIRE4Intro.HT_EXPERIMENTER) < _delta_pp):
				self.facing = lookINSPIRE4Intro.FACING_EXPERIMENTER
				rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing Experimenter")
			else:
				rospy.logdebug("lookINSPIRE4Intro.start(): Maki-ro already facing leftScreen")

		else:
			pass


		if self.facing == None:
			rospy.logwarn("lookINSPIRE4Intro.start(): Maki-ro not asleep or facing experimenter, upper right calibration point, lower right calibration point, or infant")
			## TODO: Update with something relevant for the intro script, 
			##	likely put Maki-ro to sleep
			#rospy.logwarn("lookINSPIRE4Intro.start(): resetting Maki-ro to neutral (facing infant)")
			#lookINSPIRE4Intro.turnToInfant( self )

		return

	## override base class
	def stop( self, disable_ht=True ):
		## call base class' stop
		return eyelidHeadTiltBaseBehavior.stop( self, disable_ht=disable_ht )


	def calculatePose( self ):
		_pub_cmd = ""

		## from facing infant, look to experimenter
		_ipt = self.ipt_turn
		_pub_cmd = ""
		_pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_EXPERIMENTER)
		_pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_EXPERIMENTER)
		_pub_cmd += SC_SET_IPT + str(_ipt)
		_pub_cmd += TERM_CHAR_SEND
		self.pub_cmd_look_fromInfant_toExperimenter = _pub_cmd

		## from asleep, look to experimenter
		_ipt = int( 2 * self.ipt_turn )
		_pub_cmd = ""
		_pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_EXPERIMENTER)
		_pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_EXPERIMENTER)
		_pub_cmd += SC_SET_IPT + str(_ipt)
		_pub_cmd += TERM_CHAR_SEND
		self.pub_cmd_look_fromAsleep_toExperimenter = _pub_cmd

		## from looking at experimenter, look to upper right calibration point
		_ipt = int( 2 * self.ipt_turn)
		_pub_cmd = ""
		_pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_BALL_UPPER_RIGHT)
		_pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_BALL_UPPER_RIGHT)
		_pub_cmd += SC_SET_IPT + str(_ipt)
		_pub_cmd += TERM_CHAR_SEND
		self.pub_cmd_look_fromExperimenter_toBallUpperRight = _pub_cmd

		## TODO: manipulate LL... raise???
		## from looking at upper right calibration point, look to lower right calibration point
		_ipt = self.ipt_turn
		_pub_cmd = ""
		_pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_BALL_LOWER_RIGHT)
		_pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_BALL_LOWER_RIGHT)
		_pub_cmd += SC_SET_IPT + str(_ipt)
		_pub_cmd += TERM_CHAR_SEND
		self.pub_cmd_look_fromBallUpperRight_toBallLowerRight = _pub_cmd

		## from looking at lower right calibration point, look to infant
		_ipt = self.ipt_turn
		_pub_cmd = ""
		_pub_cmd += "HP" + SC_SET_GP + str(lookINSPIRE4Intro.HP_FACE_INFANT)
		_pub_cmd += "HT" + SC_SET_GP + str(lookINSPIRE4Intro.HT_FACE_INFANT)
		_pub_cmd += SC_SET_IPT + str(_ipt)
		_pub_cmd += TERM_CHAR_SEND
		self.pub_cmd_look_fromBallLowerRight_toInfant = _pub_cmd

		return

	###########################
	##
	##	To run, publish to /maki_macro
	##		lookAtExperimenter
	##
	###########################
	def macroLookAtExperimenter( self ):
		rospy.logdebug("macrolookAtExperimenter: BEGIN")

		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				return

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
				return

			if not self.mTT_INTERRUPT:
				rospy.loginfo("-----------------")

				baseBehavior.pubTo_maki_command( self, str(self.look_at) )
				self.sww_wi.sleepWhileWaitingMS( 2000 )

				rospy.loginfo("-----------------")

			#end	if not self.mTT_INTERRUPT:
		#end	if self.ALIVE:

		rospy.logdebug("macroLookAtExperimenter: END")
		return

	###########################
	##
	##	To run, publish to /maki_macro
	##		lookAtBallLocationLowerRight
	##
	###########################
	def macroLookAtBallLocationLowerRight( self, shift=True ):
		rospy.logdebug("macroLookAtBallLocationLowerRight: BEGIN")

		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				return

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
				return

			if not self.mTT_INTERRUPT:
				rospy.loginfo("-----------------")

				baseBehavior.pubTo_maki_command( self, str(self.look_ball_loc_lower_right) )
				self.sww_wi.sleepWhileWaitingMS( 2000 )

				rospy.loginfo("-----------------")

			#end	if not self.mTT_INTERRUPT:
		#end	if self.ALIVE:

		rospy.logdebug("macroLookAtBallLocationLowerRight: END")
		return

	###########################
	##
	##	To run, publish to /maki_macro
	##		lookNeutral
	##
	###########################
	def macroLookNeutral( self ):
		rospy.logdebug("macroNeutral: BEGIN")

		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				return

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(mTT_INTERRUPT))
				return

			if not self.mTT_INTERRUPT:
				rospy.loginfo("-----------------")

				baseBehavior.pubTo_maki_command( self, str(self.look_neutral) )
				self.sww_wi.sleepWhileWaitingMS( 2000 )

				rospy.loginfo("-----------------")

			#end	if not self.mTT_INTERRUPT:
		#end	if self.ALIVE:

		rospy.logdebug("macroLookNeutral: END")
		return

	###########################################
	## This is a work in progress
	def turnToScreen( self, right_screen=True ):
		rospy.logdebug("turnToScreen(): BEGIN")

		if not isinstance(right_screen, bool):
			rospy.logerr("turnToScreen(): INVALID INPUT: right_screen should be boolean")
			return

		_pub_hp = True
		_pub_ep = False
		_pub_ht = True
		_pub_ipt = True
		_ipt_turn = self.ipt_turn

		if right_screen and (self.facing == lookINSPIRE4Intro.FACING_RIGHT_SCREEN):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Intro.FACING_RIGHT_SCREEN)

		elif (not right_screen) and (self.facing == lookINSPIRE4Intro.FACING_EXPERIMENTER):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Intro.FACING_EXPERIMENTER)
		
		elif ((right_screen and (self.facing == lookINSPIRE4Intro.FACING_EXPERIMENTER)) or
			((not right_screen) and (self.facing == lookINSPIRE4Intro.FACING_RIGHT_SCREEN))):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is NOT intended to move from looking at one screen to the other")
			rospy.logwarn("turnToScreen(): _ipt_turn adjusted to 2x")
			_ipt_turn = 2 * _ipt_turn

		else:
			pass

		## from infant perspective <==> from robot perspective 
		EP_RIGHT_SCREEN = EP_LEFT
		EP_EXPERIMENTER = EP_RIGHT

		_pub_cmd = ""

		_start_time = rospy.get_time()
		if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			_loop_count = 0

			if _pub_ep:	_ep_gp_saccade = EP_FRONT
			if _pub_hp:	_hp_gp = HP_FRONT
			if right_screen:	
				if _pub_ep:	_ep_gp_saccade = EP_RIGHT_SCREEN
				if _pub_hp:	_hp_gp = lookINSPIRE4Intro.HP_RIGHT_SCREEN
			else:
				if _pub_ep:	_ep_gp_saccade = EP_EXPERIMENTER
				if _pub_hp:	_hp_gp = lookINSPIRE4Intro.HP_EXPERIMENTER
			if _pub_ht:	_ht_gp = random.randint(self.ht_rand_min, self.ht_rand_max)

			## set first goal positions including eye saccade
			_pub_cmd = ""
			if _pub_hp:	_pub_cmd += "HPGP" + str(_hp_gp)
			if _pub_ht:	_pub_cmd += "HTGP" + str(_ht_gp)
			if _pub_ep:	_pub_cmd += "EPGP" + str(_ep_gp_saccade)
			if _pub_ipt:	_pub_cmd += "IPT" + str(_ipt_turn) 
			_pub_cmd += TERM_CHAR_SEND
			rospy.logwarn( _pub_cmd )
			#lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, hp_gp=_hp_gp, ht_gp=_ht_gp )

			#lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
			#if _pub_ep:	_ep_start_pp = self.makiPP["EP"]
			#if _pub_hp:	_hp_start_pp = self.makiPP["HP"]
			#if _pub_ht:
			#	_ht_start_pp = self.makiPP["HT"] 
			#	_gs_ht = self.DC_helper.getGoalSpeed_ticks_durationMS( abs(_ht_gp - _ht_start_pp), _ipt_turn )
			#
			#_pub_cmd = ""
			#if _pub_hp:	_pub_cmd += "HPGS" + str(_gs_hp)
			#if _pub_ht:	_pub_cmd += "HTGS" + str(_gs_ht)
			#if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
			#_pub_cmd += TERM_CHAR_SEND
			#rospy.loginfo( _pub_cmd )
			#lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			### has 100 ms delay for propogation to motors
			#
			## COMMENTED OUT becuase couldn't find a smooth velocity profile
			### (_gs_hp, _gs_ep)
			##_gs_sequence = ((48,97), (95,24), (59,24), (24,24), (12,24))
			##_gs_sequence = ((44,10), (132,29), (176,39), (88,19))
			##_gs_sequence = ((44,10), (99,22), (154,34), (99,22), (44,10))
			##_gs_sequence = ((66,15), (176,39), (121,27), (66,15), (11,2))
			#_gs_sequence = ((88,19), (88,19), (88,19), (88,19), (88,19))
			#_step_duration = float( _ipt_turn / len(_gs_sequence) )
			#
			#_loop_count = 0
			#_first_pass = True
			#if _pub_ep:	_ep_phase0 = True	## True = saccade towards screen; otherise fixate
			#for _gs_hp, _gs_ep in _gs_sequence:
			#	_start_time_step = rospy.get_time()
			#
			#	_pub_cmd = ""
			#	if _pub_hp:	_pub_cmd += "HPGS" + str(_gs_hp)
			#	if _pub_ht:	_pub_cmd += "HTGS" + str(_gs_ht)
			#	if _pub_ep:	_pub_cmd += "EPGS" + str(_gs_ep)
			#	_pub_cmd += TERM_CHAR_SEND
			#	rospy.loginfo( _pub_cmd )
			#	lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			#	## has 100 ms delay for propogation to motors
			#
			#	## set first goal positions including eye saccade
			#	if _first_pass:
			#		_pub_cmd = ""
			#		if _pub_hp:	_pub_cmd += "HPGP" + str(_hp_gp)
			#		if _pub_ht:	_pub_cmd += "HTGP" + str(_ht_gp)
			#		if _pub_ht:	_pub_cmd += "EPGP" + str(_ep_gp_saccade)
			#		_pub_cmd += TERM_CHAR_SEND
			#		rospy.logwarn( _pub_cmd )
			#		lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			#		_first_pass = False
			#
			#	## set eye pan goal position after completing eye saccade
			#	elif _pub_ep and _ep_phase0:
			#		_pub_cmd = ""
			#		_pub_cmd += "EPGP" + str(EP_FRONT)
			#		_pub_cmd += TERM_CHAR_SEND
			#		rospy.logwarn( _pub_cmd )
			#		lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			#		_ep_phase0 = False
			#	else:
			#		pass
			#
			#	if (abs(rospy.get_time() - _start_time) < _ipt_turn):
			#		## has 100 ms delay for propogation to motors
			#		lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
			#
			#		## compensate to maintain pacing of 200 ms apart
			#		_adjusted_sleep = _step_duration - abs(rospy.get_time() - _start_time_step)
			#		if (_adjusted_sleep <= 0):
			#			rospy.logdebug("... no sleep _step_duration adjustment")
			#		else:	
			#			rospy.logdebug( str(_adjusted_sleep) + " milliseconds more are needed to fulfill _step_duration pacing")
			#			self.SWW_WI.sleepWhileWaitingMS( _adjusted_sleep, end_early=False )
			#
			#		_loop_count = _loop_count +1
			#	else:
			#		rospy.logdebug("TIME IS UP")
			#		break	
			#
			#end	for _gs_hp, _gs_ep in _gs_sequence:

			if right_screen:
				self.facing = lookINSPIRE4Intro.FACING_RIGHT_SCREEN
			else:
				self.facing = lookINSPIRE4Intro.FACING_EXPERIMENTER

			## TODO: keep track of INSPIRE4 state
		else:
			rospy.logwarn("Cannot turnToScreen. Publish 'interaction start' first")
			return
		#end	if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):

		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo( "NUMBER OF TIMESTEPS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("turnToScreen(): END")
		return

	## TODO: lookAt should shift to new ground state
	## This is a work in progress
	def lookAt( self, commandOut, monitor=True, hp_gp=None, ht_gp=None ):
		rospy.logdebug("lookAt(): BEGIN")

		if (monitor and
			((hp_gp != None) and isinstance(hp_gp, int)) and
			((ht_gp != None) and isinstance(ht_gp, int))):
			pass
		else:
			rospy.logwarn("lookAt(): INVALID INPUTS: monitor must be True AND hp_gp and ht_gp must be valid integers")
			monitor=False

		## TODO: Check validity of changing from pose to another... INTENDED vs. not
		#if (self.facing == lookINSPIRE4Intro.FACING_INFANT):
		#	rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Intro.FACING_INFANT)

		_pub_cmd = ""

		_start_time = rospy.get_time()
		if (lookINSPIRE4Intro.__is_intro_running and (self.ALIVE) and 
			(not self.mTT_INTERRUPT) and (not rospy.is_shutdown())):
			_loop_count = 0


			## NOTE: unnecssary to specify eye pan since monitorMoveToGP() and
			## pubTo_maki_command() are inherited from headPanBaseBehavior
			## Will automatically adjust eye pan based on head pan, if not specified
			_pub_cmd = commandOut
			rospy.logwarn( _pub_cmd )
			if monitor:
				try:
					lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, hp_gp=hp_gp, ht_gp=ht_gp )
				except rospy.exceptions.ROSException as _e:
					rospy.logerr("lookAt(): " + str(_e))
			else:
				_tmp = re.search( "IPT([0-9]+)", _pub_cmd )
				lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
				if _tmp != None:
					self.SWW_WI.sleepWhileWaitingMS( int(_tmp.group(1))-100, end_early=False )
		
		else:
			rospy.logwarn("Cannot lookAt. Publish 'intro start' first")
			return
		#end	if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):

		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo( "NUMBER OF TIMESTEPS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("lookAt(): END")
		return

	## NOTE: during intro, we expect Maki-ro's head to be tilted up to 
	##	face the experimenter
	def macroGreeting( self, nod_angle=15.0, duration=600, repetitions=1 ):
		rospy.logdebug("macroGreeting(): BEGIN")

		_start_time = rospy.get_time()
		## check the inputs
		if isinstance(nod_angle, float) or isinstance(nod_angle, int):
			pass
		else:
			rospy.logwarn("macroGreeting(): INVALID VALUE: nod_angle=" + str(nod_angle) + "; updated to 15 degrees")
			nod_angle = 15.0	## degrees
		if isinstance(duration, float) or isinstance(duration, int):
			pass
		else:
			rospy.logwarn("macroGreeting(): INVALID VALUE: duration=" + str(duration) + "; updated to 600 milliseconds")
			duration = 600	## milliseconds
		if isinstance(repetitions, int) and (repetitions > 0):
			pass
		else:
			rospy.logwarn("macroGreeting(): INVALID VALUE: repetitions=" + str(repetitions) + "; updated to 1")
			repetitions = 1

		_pub_ipt = False #True
		_monitor = True	#False
		_duration_nod = duration

		_my_ticks_ht = self.DC_helper.convertToTicks_degrees( nod_angle )
		rospy.logdebug("_my_ticks_ht: " + str(nod_angle) + " degrees is " + str(_my_ticks_ht) + " ticks")
		_gs_ht = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _my_ticks_ht, _duration_nod) )
		_my_ticks_ht = int( float(_my_ticks_ht * 0.5) + 0.5 )

		_my_ticks_ll = self.DC_helper.convertToTicks_degrees( nod_angle * 0.36 )	## same constant Todorovic 2009
		rospy.logdebug("_my_ticks_ll: " + str(nod_angle) + " degrees is " + str(_my_ticks_ll) + " ticks")
		_gs_ll = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _my_ticks_ll, _duration_nod) )
		_my_ticks_ll = int( float(_my_ticks_ll * 0.5) + 0.5 )
		rospy.logdebug("DIVIDED IN HALF: _my_ticks_ht=" + str(_my_ticks_ht) + "; _my_ticks_ll=" + str(_my_ticks_ll))

		## Store initial pose
		lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
		self.previous_ll = self.makiPP["LL"]
		self.previous_ht = self.makiPP["HT"]

		_my_head_nod_up_ht = self.previous_ht + int( float(_my_ticks_ht * 0.5) + 0.5 )
		_my_head_nod_down_ht = self.previous_ht - _my_ticks_ht
		_my_head_nod_up_ll = self.previous_ll - _my_ticks_ll	## head up, eyelids down
		_my_head_nod_down_ll = self.previous_ll + _my_ticks_ll	## head down, eyelids up

		## generate servo control command to set goal positions
		## NOTE: on the Arbotix-M side, a sync_write function is used
		## to simultaneously broadcast the updated goal positions
		_duration_nod_up = float(_duration_nod) * 0.2	#0.25
		_head_nod_up_gp_cmd = ""
		_head_nod_up_gp_cmd += "LL" + SC_SET_GP + str(_my_head_nod_up_ll)
		_head_nod_up_gp_cmd += "HT" + SC_SET_GP + str(_my_head_nod_up_ht)
		if _pub_ipt:	_head_nod_up_gp_cmd += SC_SET_IPT + str( int(_duration_nod_up + 0.5) )
		_head_nod_up_gp_cmd += TERM_CHAR_SEND

		_duration_nod_down = float(_duration_nod) * 0.6	#0.5
		_head_nod_down_gp_cmd = ""
		_head_nod_down_gp_cmd += "LL" + SC_SET_GP + str(_my_head_nod_down_ll)
		_head_nod_down_gp_cmd += "HT" + SC_SET_GP + str(_my_head_nod_down_ht)
		if _pub_ipt:	_head_nod_down_gp_cmd += SC_SET_IPT + str( int(_duration_nod_down + 0.5) )
		_head_nod_down_gp_cmd += TERM_CHAR_SEND

		_duration_nod_center = float(_duration_nod) * 0.3	#0.2	#0.25
		_head_nod_center_gp_cmd = ""
		_head_nod_center_gp_cmd += "LL" + SC_SET_GP + str(self.previous_ll)
		_head_nod_center_gp_cmd += "HT" + SC_SET_GP + str(self.previous_ht)
		if _pub_ipt:	_head_nod_center_gp_cmd += SC_SET_IPT + str( int(_duration_nod_center + 0.5) )
		_head_nod_center_gp_cmd += TERM_CHAR_SEND

		_my_tuple = (("NOD UP", _duration_nod_up, _my_head_nod_up_ll, _my_head_nod_up_ht, _head_nod_up_gp_cmd), ("NOD DOWN", _duration_nod_down, _my_head_nod_down_ll, _my_head_nod_down_ht, _head_nod_down_gp_cmd), ("NOD CENTER", _duration_nod_center, self.previous_ll, self.previous_ht, _head_nod_center_gp_cmd))
		_pub_cmd = ""

		## preset the desired goal speeds BEFORE sending the goal positions
		_pub_cmd = ""
		_pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
		_pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
		## NOTE: pubTo_maki_command will automatically add TERM_CHAR_SEND postfix
		## publish and give time for the command to propogate to the servo motors
		lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd), cmd_prop=True )

		_duration = abs(rospy.get_time() -_start_time)
		rospy.loginfo("OVERHEAD SETUP TIME: " + str(_duration) + " seconds")

		_loop_count = 0
		_start_time = rospy.get_time()
		while (_loop_count < repetitions) and (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			rospy.logdebug("-------------------")

			for _print, _duration_nod, _my_ll, _my_ht, _head_nod_gp_cmd in _my_tuple:
				rospy.loginfo("====> " + str(_print))
				#lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
				#rospy.logdebug( str(self.makiPP) )

				#if not _pub_ipt:
				#	## Calculate goal speed base on distance and duration (in milliseconds)
				#	_distance_to_head_nod_ll = abs( self.makiPP["LL"] - _my_ll )
				#	rospy.logdebug("_distance_to_head_nod_ll=" + str(_distance_to_head_nod_ll))
				#	if (_distance_to_head_nod_ll > 0):
				#		_gs_ll = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_head_nod_ll, _duration_nod) )
				#		rospy.loginfo("_gs_ll=" + str(_gs_ll))
				#
				#	_distance_to_head_nod_ht = abs( self.makiPP["HT"] - _my_ht )
				#	rospy.logdebug("_distance_head_nod_ht=" + str(_distance_to_head_nod_ht))
				#	if (_distance_to_head_nod_ht > 0):
				#		_gs_ht = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_head_nod_ht, _duration_nod) )
				#		rospy.loginfo("_gs_ht=" + str(_gs_ht))
				#		_gs_ht = min(_gs_ht, self.HT_GS_MAX)
				#		rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))
				#
				#	## preset the desired goal speeds BEFORE sending the goal positions
				#	_pub_cmd = ""
				#	if (_distance_to_head_nod_ll>0):	_pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
				#	if (_distance_to_head_nod_ht>0):	_pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
				#	## NOTE: pubTo_maki_command will automatically add TERM_CHAR_SEND postfix
				#
				#	## publish and give time for the command to propogate to the servo motors
				#	lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd), cmd_prop=True )

				## set servo control command to set goal positions
				_pub_cmd = _head_nod_gp_cmd

				_start_time_head_nod = rospy.get_time()
				try:
					if _monitor and (_duration_nod >= 200):
						## NOTE: publish and give time for the command to propogate to the servo motors,
						## but DO NOT MONITOR (excess overhead of minimum 200ms, which is greater
						## than some _duration_nod and will cause delay)
						lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, ll_gp=_my_ll, ht_gp=_my_ht )
					else:
						lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd )	## default 100ms to propogate
						if _duration_nod > 100:
							self.SWW_WI.sleepWhileWaitingMS( _duration_nod - 100, end_early=False)
				except rospy.exceptions.ROSException as e1:
					rospy.logerr( str(e1) )
				_duration = abs(_start_time_head_nod - rospy.get_time())
				rospy.loginfo( str(_print) + "\tDURATION: " + str(_duration) + " seconds; EXPECTED " + str(_duration_nod))
			#end	for _print, _duration_nod, _my_ll, _my_ht, _head_nod_gp_cmd in _my_tuple:

			_loop_count = _loop_count +1

			## debugging
			#rospy.loginfo(".............P A U S E ...")
			#self.SWW_WI.sleepWhileWaiting( 1 )	## 1 second
		# end	while not rospy.is_shutdown():

		_duration = abs(rospy.get_time() - _start_time)
		rospy.logdebug( "NUMBER OF GREETING MOVMENTS: " + str(_loop_count) )
		rospy.logdebug( "Duration: " + str(_duration) + " seconds" )
		return


	def startStartle( self, relax=False ):
		rospy.logdebug("startStartle(): BEGIN")
		### call base class' start function
		#eyelidHeadTiltBaseBehavior.start(self)
		#rospy.logdebug("startStartle(): After eyelidHeadTiltBaseBehavior.start()")
		lookINSPIRE4Intro.macroStartleRelax( self, startle=True, relax=relax )
		rospy.logdebug("startStartle(): END")


	## similar to the engagment game but uses current HT and LL values
	## startle and relax are relative to the current HT and LL values
	def macroStartleRelax( self, startle=True, relax=True, repetitions=1 ):
		rospy.logdebug("macroStartleRelax(): BEGIN")

		_start_time = rospy.get_time()
		## check the inputs
		if isinstance(startle, bool) and (not startle) and isinstance(relax, bool) and (not relax):	return
		if isinstance(repetitions, int) and (repetitions > 0):
			pass
		else:
			rospy.logwarn("macroStartleRelax(): INVALID VALUE: repetitions=" + str(repetitions) + "; updated to 1")
			repetitions = 1

		_expected_delta_ll = self.LL_STARTLE - self.LL_NEUTRAL
		_expected_delta_ht = self.HT_STARTLE - self.HT_NEUTRAL
		rospy.logdebug("_expected_delta_ll=" + str(_expected_delta_ll) + " ticks, _expected_delta_ht=" + str(_expected_delta_ht) + " ticks")

		if startle:
			## Store initial pose
			lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
			self.previous_ll = self.makiPP["LL"]
			self.previous_ht = self.makiPP["HT"]

		## generate servo control command to set goal positions
		## NOTE: on the Arbotix-M side, a sync_write function is used
		## to simultaneously broadcast the updated goal positions
		_my_startle_ht = self.HT_STARTLE
		_my_startle_ll = self.LL_STARTLE
		_startle_gp_cmd = ""
		if startle:
			#_startle_gp_cmd += "LL" + SC_SET_GP + str(self.LL_STARTLE)
			#_startle_gp_cmd += "HT" + SC_SET_GP + str(self.HT_STARTLE)

			if (self.previous_ll >= _my_startle_ll):
				_my_startle_ll = self.previous_ll + _expected_delta_ll
				rospy.logdebug("adjusted _my_startle_ll to " + str(_my_startle_ll) + " ticks")
			if (self.previous_ht >= _my_startle_ht):
				_my_startle_ht = self.previous_ht + _expected_delta_ht
				rospy.logdebug("adjusted _my_startle_ht to " + str(_my_startle_ht) + " ticks")
			_startle_gp_cmd += "LL" + SC_SET_GP + str(_my_startle_ll)
			_startle_gp_cmd += "HT" + SC_SET_GP + str(_my_startle_ht)
			_startle_gp_cmd += TERM_CHAR_SEND
		_relax_gp_cmd  = ""
		if relax:
			#_relax_gp_cmd += "LL" + SC_SET_GP + str(self.LL_NEUTRAL)
			#_relax_gp_cmd += "HT" + SC_SET_GP + str(self.HT_NEUTRAL)
			_relax_gp_cmd += "LL" + SC_SET_GP + str(self.previous_ll)
			_relax_gp_cmd += "HT" + SC_SET_GP + str(self.previous_ht)
			_relax_gp_cmd += TERM_CHAR_SEND
		_pub_cmd = ""

		## NOTE: during intro, we expect Maki-ro's head to be tilted up to 
		##	face the experimenter
		#
		#if relax and (not startle):
		#	rospy.loginfo("relax ONLY... skip alignment")
		#	pass
		#else:
		#	## Move to neutral eyelid and head tilt pose
		#	rospy.loginfo("BEFORE startle, adjust LL and HT to NeutralPose")
		#	_pub_cmd = ""
		#	if (abs(self.makiPP["LL"] - self.LL_NEUTRAL) > DELTA_PP):
		#		_pub_cmd += "LLGP" + str(self.LL_NEUTRAL)
		#	if (abs(self.makiPP["HT"] - self.HT_NEUTRAL) > DELTA_PP):
		#		_pub_cmd += "HTGP" + str(self.HT_NEUTRAL) 
		#	if ( len(_pub_cmd) > 0 ):
		#		_pub_cmd += TERM_CHAR_SEND
		#		try:
		#			lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, ll_gp=self.LL_NEUTRAL, ht_gp=self.HT_NEUTRAL )
		#		except rospy.exceptions.ROSException as _e:
		#			rospy.logerr( str(_e) )
		#		#self.SWW_WI.sleepWhileWaiting(1)	## 1 second	## debugging

		_duration = abs(rospy.get_time() -_start_time)
		rospy.loginfo("OVERHEAD SETUP TIME: " + str(_duration) + " seconds")

		## TODO: unify duration_startle
		#_duration_startle = 100		## millisecond
		_duration_relax = 1000		## milliseconds
		_duration_relax_wait = 250
		_loop_count = 0
		_tmp_scale = 1.0
		_start_time = rospy.get_time()
		while (_loop_count < repetitions) and (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			rospy.logdebug("-------------------")

			if startle:
				rospy.loginfo("====> STARTLE")
				lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
				rospy.logdebug( str(self.makiPP) )

				## Calculate goal speed base on distance and duration (in milliseconds)
				_duration_startle = 100		## millisecond
				_distance_to_startle_ll = abs( self.makiPP["LL"] - _my_startle_ll )
				rospy.logdebug("_distance_startle_ll=" + str(_distance_to_startle_ll))
				_tmp_duration_startle = _duration_startle
				if (abs(_distance_to_startle_ll - _expected_delta_ll) > DELTA_PP):
					_tmp_scale = float(_distance_to_startle_ll) / float(_expected_delta_ll) 
					_tmp_duration_startle = _tmp_scale * _tmp_duration_startle
				rospy.logdebug("_tmp_scale=" + str(_tmp_scale) + "; _tmp_duration_startle=" + str(_tmp_duration_startle))
				if (_distance_to_startle_ll > 0):
					_gs_ll = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_startle_ll, _tmp_duration_startle) )
					rospy.loginfo("_gs_ll=" + str(_gs_ll))

				_duration_startle = 150	#200	#250		## millisecond
				_distance_to_startle_ht = abs( self.makiPP["HT"] - _my_startle_ht )
				rospy.logdebug("_distance_startle_ht=" + str(_distance_to_startle_ht))
				_tmp_duration_startle = _duration_startle
				if (abs(_distance_to_startle_ht - _expected_delta_ht) > DELTA_PP):
					_tmp_scale = float(_distance_to_startle_ht) / float(_expected_delta_ht) 
					_tmp_duration_startle = _tmp_scale * _tmp_duration_startle
				rospy.logdebug("_tmp_scale=" + str(_tmp_scale) + "; _tmp_duration_startle=" + str(_tmp_duration_startle))
				if (_distance_to_startle_ht > 0):
					_gs_ht = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_startle_ht, _tmp_duration_startle) )
					rospy.loginfo("_gs_ht=" + str(_gs_ht))
					_gs_ht = min(_gs_ht, self.HT_GS_MAX)
					rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))

				## preset the desired goal speeds BEFORE sending the goal positions
				_pub_cmd = ""
				if (_distance_to_startle_ll>0):	_pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
				if (_distance_to_startle_ht>0):	_pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
				## NOTE: pubTo_maki_command will automatically add TERM_CHAR_SEND postfix

				## publish and give time for the command to propogate to the servo motors
				lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd), cmd_prop=True )

				## set servo control command to set goal positions
				_pub_cmd = _startle_gp_cmd

				_start_time_startle = rospy.get_time()
				try:
					## Maki-ro open eyes wide
					## and "jerks" head back
					lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd )	## default 100ms to propogate
					## NOTE: publish and give time for the command to propogate to the servo motors,
					## but DO NOT MONITOR (excess overhead of minimum 200ms, which is greater
					## than _duration_startle and will cause delay)
					#lookINSPIRE4Intro.monitorMoveToGP( self, _pub_cmd, ll_gp=self.LL_STARTLE, ht_gp=self.HT_STARTLE)
					if _tmp_duration_startle > 100:
						self.SWW_WI.sleepWhileWaitingMS( _tmp_duration_startle - 100)
				except rospy.exceptions.ROSException as e1:
					rospy.logerr( str(e1) )
				_duration = abs(_start_time_startle - rospy.get_time())
				rospy.logwarn( "Startle duration: " + str(_duration) + " seconds" )

				lookINSPIRE4Intro.__is_startled = True
				rospy.loginfo("Done: STARTLE ====")
			#end	if startle:

			if relax:
				rospy.loginfo("====> RELAX")
				rospy.loginfo( str(self.makiPP) )
				_first_pass = True
				_start_time_relax = rospy.get_time()

				## own version of monitorMoveToGP
				## adjusts speed based on difference
				## between current position and goal
				## position to stay within _duration_relax
				while relax and (not rospy.is_shutdown()):
					lookINSPIRE4Intro.requestFeedback( self, SC_GET_PP )
					rospy.loginfo( str(self.makiPP) )

					## computer difference between current and goal positions
					## TODO: do this calculation using map
					#_distance_to_relax_ll = abs( self.makiPP["LL"] - self.LL_NEUTRAL )
					#rospy.loginfo("_distance_to_relax_ll=" + str(_distance_to_relax_ll))
					#_distance_to_relax_ht = abs( self.makiPP["HT"] - self.HT_NEUTRAL )
					#rospy.loginfo("_distance_to_relax_ht=" + str(_distance_to_relax_ht))
					_distance_to_relax_ll = abs( self.makiPP["LL"] - self.previous_ll )
					rospy.loginfo("_distance_to_relax_ll=" + str(_distance_to_relax_ll))
					_distance_to_relax_ht = abs( self.makiPP["HT"] - self.previous_ht )
					rospy.loginfo("_distance_to_relax_ht=" + str(_distance_to_relax_ht))
					#_tmp_duration_startle = _duration_startle
					#if (abs(_distance_to_startle_ht - _expected_delta_ht) > DELTA_PP):
					#	_tmp_scale = float( _distance_to_startle_ht / _expected_delta_ht )
					#	_tmp_duration_startle = _tmp_scale * _tmp_duration_startle
					#rospy.logdebug("_tmp_scale=" + str(_tmp_scale) + "_tmp_duration_startle=" + str(_tmp_duration_startle))

					## adjust duration to stay within _duration_relax
					if _first_pass:
						rospy.logdebug("duration_relax = " + str(_duration_relax))
						_first_pass=False
						pass
					elif (_distance_to_relax_ll > DELTA_PP) or (_distance_to_relax_ht > DELTA_PP):
						_duration_relax = _duration_relax - _duration_relax_wait
						rospy.logdebug("duration_relax = " + str(_duration_relax))
					else:
						rospy.logdebug("close enough...done relax while loop")
						break
					if (_duration_relax <= 0):
						rospy.logdebug("negative time...done relax while loop")
						break

					## calculate new goal speeds
					_gs_ll = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_relax_ll, _duration_relax)
					rospy.loginfo("_gs_ll=" + str(_gs_ll))
					_gs_ll = max(_gs_ll, self.LL_GS_MIN)
					rospy.loginfo("adjusted _gs_ll=" + str(_gs_ll))
					_gs_ht = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_relax_ht, _duration_relax)
					rospy.loginfo("_gs_ht=" + str(_gs_ht))
					_gs_ht = max(_gs_ht, self.HT_GS_MIN)
					rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))
					#_duration_relax_wait = self.DC_helper.getTurnDurationMS_ticks_goalSpeed( _distance_to_relax, _gs_ll )
					#rospy.loginfo("waitMS = " + str(_duration_relax_wait))
			
					## generate servo control command to set new goal speeds
					_pub_cmd = ""
					_pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
					_pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
					lookINSPIRE4Intro.pubTo_maki_command( self, str(_pub_cmd) )

					## servo control command to set goal positions
					_pub_cmd = _relax_gp_cmd
		
					#_start_time_relax = rospy.get_time()
					try:
						## Maki-ro relaxes wide open eyes
						## and head comes back forward to neutral
						lookINSPIRE4Intro.pubTo_maki_command( self, _pub_cmd )
						self.SWW_WI.sleepWhileWaitingMS( _duration_relax_wait, end_early=False)
					except rospy.exceptions.ROSException as e2:
						rospy.logerr( str(e2) )
				#end	while relax and (not rospy.is_shutdown()):

				_duration = abs(_start_time_relax - rospy.get_time())
				rospy.logwarn( "Relax duration: " + str(_duration) + " seconds" )
				rospy.loginfo( str(self.makiPP) )

				lookINSPIRE4Intro.__is_startled = False
				rospy.loginfo("Done: RELAX ====")
			#end	if relax:

			_loop_count = _loop_count +1

			## debugging
			#rospy.loginfo(".............P A U S E ...")
			#self.SWW_WI.sleepWhileWaiting( 1 )	## 1 second
		# end	while not rospy.is_shutdown():

		_duration = abs(rospy.get_time() - _start_time)
		rospy.logdebug( "NUMBER OF STARTLE/RELAX MOVMENTS: " + str(_loop_count) )
		rospy.logdebug( "Duration: " + str(_duration) + " seconds" )
		return


	def startStartle( self, relax=False ):
		rospy.logdebug("startStartle(): BEGIN")
		### call base class' start function
		#eyelidHeadTiltBaseBehavior.start(self)
		#rospy.logdebug("startStartle(): After eyelidHeadTiltBaseBehavior.start()")
		lookINSPIRE4Intro.macroStartleRelax( self, startle=True, relax=relax )
		rospy.logdebug("startStartle(): END")
		return

	def stopStartle( self ):
		## shift into eyelid and headtilt neutral
		lookINSPIRE4Intro.macroStartleRelax( self, startle=False, relax=True )

		## call base class' stop function
		eyelidHeadTiltBaseBehavior.stop(self)
		return

	def introStart( self ):
		## try to nicely startup without jerking MAKI's head tilt servo
		lookINSPIRE4Intro.start(self)
		lookINSPIRE4Intro.__is_intro_running = True
		return

	def introStop( self, disable_ht=True ):
		rospy.logdebug("introStop(): BEGIN")
		lookINSPIRE4Intro.__is_intro_running = False
		lookINSPIRE4Intro.stop(self, disable_ht=disable_ht)
		rospy.logdebug("introStop(): END")
		return


	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "intro start":
			lookINSPIRE4Intro.introStart( self )

		elif msg.data.startswith( "intro stop"):
			if msg.data.endswith( "disable_ht=False" ):
				lookINSPIRE4Intro.introStop( self, disable_ht=False )
				## TODO: May need to change to False when whole INSPIRE4
				##	script and control program are in place
			else:
				lookINSPIRE4Intro.introStop( self, disable_ht=True )

		elif msg.data == "intro greet":
			lookINSPIRE4Intro.macroGreeting( self )

		elif msg.data == "intro startle":
			## perform one startle then immediately relax
			lookINSPIRE4Intro.startStartle( self, relax=True )

		elif msg.data == "intro startle start":
			## perform one startle and hold
			lookINSPIRE4Intro.startStartle( self, relax=False )

		elif msg.data == "intro startle stop":
			## from startle, immediately relax
			lookINSPIRE4Intro.stopStartle( self )

		elif msg.data == "intro lookAtExperimenter":
			lookINSPIRE4Intro.lookAt( self, self.pub_cmd_look_fromInfant_toExperimenter )

		elif msg.data == "intro lookAtBallLocationUpperRight":
			lookINSPIRE4Intro.lookAt( self, self.pub_cmd_look_fromExperimenter_toBallUpperRight )

		elif msg.data == "intro lookAtBallLocationLowerRight":
			lookINSPIRE4Intro.lookAt( self, self.pub_cmd_look_fromBallUpperRight_toBallLowerRight )

		elif msg.data == "intro lookAtInfant":
			lookINSPIRE4Intro.lookAt( self, self.pub_cmd_look_fromBallLowerRight_toInfant )
			## TODO: Maki-ro doesn't quite return to HT_MIDDLE
			lookINSPIRE4Intro.pubTo_maki_command( self, "reset" )

		elif msg.data == "lookNeutral":
			#self.macroLookNeutral()
			pass

		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	lookIntro = lookINSPIRE4Intro( True, None )

	rospy.Subscriber( "/maki_macro", String, lookIntro.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"





