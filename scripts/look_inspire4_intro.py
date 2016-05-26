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

		if lookINSPIRE4Intro.HP_EXPERIMENTER == None:	
			lookINSPIRE4Intro.HP_EXPERIMENTER = 620	## ticks
		if lookINSPIRE4Intro.HT_EXPERIMENTER == None:	
			lookINSPIRE4Intro.HT_EXPERIMENTER = 540	## ticks

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



	def introStart( self ):
		## try to nicely startup without jerking MAKI's head tilt servo
		lookINSPIRE4Intro.start(self)
		lookINSPIRE4Intro.__is_intro_running = True
		return

	def introStop( self, disable_ht=True ):
		lookINSPIRE4Intro.__is_intro_running = False
		lookINSPIRE4Intro.stop(self, disable_ht=disable_ht)
		return


	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "intro start":
			lookINSPIRE4Intro.introStart( self )

		elif msg.data == "intro stop":
			lookINSPIRE4Intro.introStop( self, disable_ht=True )
			## TODO: May need to change to False when whole INSPIRE4
			##	script and control program are in place

		elif msg.data == "lookAtExperimenter":
			#self.macroLookAtExperimenter()
			lookINSPIRE4Intro.lookAt( self, self.pub_cmd_look_fromInfant_toExperimenter )

		elif msg.data == "lookAtBallLocationUpperRight":
			lookINSPIRE4Intro.lookAt( self, self.pub_cmd_look_fromExperimenter_toBallUpperRight )

		elif msg.data == "lookAtBallLocationLowerRight":
			#self.macroLookAtBallLocationLowerRight()
			lookINSPIRE4Intro.lookAt( self, self.pub_cmd_look_fromBallUpperRight_toBallLowerRight )

		elif msg.data == "lookAtInfant":
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




