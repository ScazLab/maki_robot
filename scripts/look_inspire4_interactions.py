#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string
import random

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * 	## classes baseBehavior and headTiltBaseBehavior
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## Maki-ro's "lookAt" locations during the INSPIRE4 interaction
##
## Description of INSPIRE4 interaction:
##	- Maki-ro starts face center
##	- Maki-ro performs ONE startle behavior
##	- If the infant fixates on Maki-ro, then Maki-ro will turn towards
##		the designated left or right screen. (1 second)
##	- Maki-ro will watch the stimuli for 8 seconds
##	- Maki-ro will return to face center (1 second)
########################
class lookINSPIRE4Interaction( headTiltBaseBehavior, headPanBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	HP_LEFT_SCREEN = None
	HT_LEFT_SCREEN = None
	EP_LEFT_SCREEN_SACCADE = None
	
	HP_RIGHT_SCREEN = None
	HT_RIGHT_SCREEN = None
	EP_RIGHT_SCREEN_SACCADE = None

	HP_FACE_INFANT = None
	HT_FACE_INFANT = None
	EP_FACE_INFANT_FROM_LEFT_SCREEN = None
	EP_FACE_INFANT_FROM_RIGHT_SCREEN = None

	FACING_LEFT_SCREEN = None
	FACING_RIGHT_SCREEN = None
	FACING_INFANT = None

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		headPanBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		if lookINSPIRE4Interaction.HP_LEFT_SCREEN == None:	
			lookINSPIRE4Interaction.HP_LEFT_SCREEN = 675	#650	#620
		if lookINSPIRE4Interaction.HT_LEFT_SCREEN == None:	
			lookINSPIRE4Interaction.HT_LEFT_SCREEN = 540
		if lookINSPIRE4Interaction.EP_LEFT_SCREEN_SACCADE == None:
			lookINSPIRE4Interaction.EP_LEFT_SCREEN_SACCADE = EP_RIGHT

		if lookINSPIRE4Interaction.HP_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.HP_RIGHT_SCREEN = 349	#404
		if lookINSPIRE4Interaction.HT_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.HT_RIGHT_SCREEN = lookINSPIRE4Interaction.HT_LEFT_SCREEN
		if lookINSPIRE4Interaction.EP_RIGHT_SCREEN_SACCADE == None:
			lookINSPIRE4Interaction.EP_RIGHT_SCREEN_SACCADE = EP_LEFT

		if lookINSPIRE4Interaction.HP_FACE_INFANT == None:
			lookINSPIRE4Interaction.HP_FACE_INFANT = HP_FRONT
		if lookINSPIRE4Interaction.HT_FACE_INFANT == None:
			lookINSPIRE4Interaction.HT_FACE_INFANT = HT_MIDDLE	
		if lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_LEFT_SCREEN == None:
			lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_LEFT_SCREEN = EP_LEFT
		if lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_RIGHT_SCREEN = EP_RIGHT

		if lookINSPIRE4Interaction.FACING_LEFT_SCREEN == None:
			lookINSPIRE4Interaction.FACING_LEFT_SCREEN = "leftScreen"
		if lookINSPIRE4Interaction.FACING_RIGHT_SCREEN == None:
			lookINSPIRE4Interaction.FACING_RIGHT_SCREEN = "rightScreen"
		if lookINSPIRE4Interaction.FACING_INFANT == None:
			lookINSPIRE4Interaction.FACING_INFANT = "infant"

		self.facing = None

		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		self.ipt_turn = 1000	## ms
		self.delta_ht = 10	## ticks
		self.ht_rand_min = lookINSPIRE4Interaction.HT_LEFT_SCREEN - self.delta_ht
		self.ht_rand_max = lookINSPIRE4Interaction.HT_LEFT_SCREEN +- self.delta_ht

		self.ALIVE = True
		return

	## override base class
	def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.5):
		rospy.logdebug("lookINSPIRE4Interaction.pubTo_maki_command(): BEGIN")
		## call base class' pubTo_maki_command
		headPanBaseBehavior.pubTo_maki_command( self, commandOut, fixed_gaze=fixed_gaze, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )
		rospy.logdebug("lookINSPIRE4Interaction.pubTo_maki_command(): END")
		return

	## override base class
	def start( self, enable_ht=True ):
		## call base class' start
		headTiltBaseBehavior.start( self, enable_ht=enable_ht )

		## CHECK TO SEE WHICH HEAD PAN POSITION IS CLOSEST
		lookINSPIRE4Interaction.requestFeedback( self, SC_GET_PP )
		_hp_pp = self.makiPP["HP"]	
		_delta_pp = DELTA_PP
		if (abs(_hp_pp - lookINSPIRE4Interaction.HP_FACE_INFANT) < _delta_pp):
			self.facing = lookINSPIRE4Interaction.FACING_INFANT
			rospy.logdebug("lookINSPIRE4Interaction.start(): Maki-ro already facing infant")

		elif (abs(_hp_pp - lookINSPIRE4Interaction.HP_RIGHT_SCREEN) < _delta_pp):
			self.facing = lookINSPIRE4Interaction.FACING_RIGHT_SCREEN
			rospy.logdebug("lookINSPIRE4Interaction.start(): Maki-ro already facing rightScreen")

		elif (abs(_hp_pp - lookINSPIRE4Interaction.HP_LEFT_SCREEN) < _delta_pp):
			self.facing = lookINSPIRE4Interaction.FACING_LEFT_SCREEN
			rospy.logdebug("lookINSPIRE4Interaction.start(): Maki-ro already facing leftScreen")

		else:
			rospy.logwarn("lookINSPIRE4Interaction.start(): Maki-ro not facing rightScreen, leftScreen, or infant")
			rospy.logwarn("lookINSPIRE4Interaction.start(): resetting Maki-ro to neutral (facing infant)")
			lookINSPIRE4Interaction.turnToInfant( self )

		return

	## override base class
	def stop( self, disable_ht=True ):
		## call base class' stop
		return headTiltBaseBehavior.stop( self, disable_ht=disable_ht )

	###########################################
	##
	##	To run, publish to /maki_macro:
	##		turnToScreen right
	##		turnToScreen left
	##
	## Default is right_screen == True, Maki-ro faces to rightScreen; 
	##	otherwise if False, Maki-ro faces to leftScreen
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

		if right_screen and (self.facing == lookINSPIRE4Interaction.FACING_RIGHT_SCREEN):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Interaction.FACING_RIGHT_SCREEN)

		elif (not right_screen) and (self.facing == lookINSPIRE4Interaction.FACING_LEFT_SCREEN):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Interaction.FACING_LEFT_SCREEN)
		
		elif ((right_screen and (self.facing == lookINSPIRE4Interaction.FACING_LEFT_SCREEN)) or
			((not right_screen) and (self.facing == lookINSPIRE4Interaction.FACING_RIGHT_SCREEN))):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is NOT intended to move from looking at one screen to the other")
			rospy.logwarn("turnToScreen(): _ipt_turn adjusted to 2x")
			_ipt_turn = 2 * _ipt_turn

		else:
			pass

		## from infant perspective <==> from robot perspective 
		EP_RIGHT_SCREEN = EP_LEFT
		EP_LEFT_SCREEN = EP_RIGHT

		_pub_cmd = ""

		_start_time = rospy.get_time()
		if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			_loop_count = 0

			if _pub_ep:	_ep_gp_saccade = EP_FRONT
			if _pub_hp:	_hp_gp = HP_FRONT
			if right_screen:	
				if _pub_ep:	_ep_gp_saccade = EP_RIGHT_SCREEN
				if _pub_hp:	_hp_gp = lookINSPIRE4Interaction.HP_RIGHT_SCREEN
			else:
				if _pub_ep:	_ep_gp_saccade = EP_LEFT_SCREEN
				if _pub_hp:	_hp_gp = lookINSPIRE4Interaction.HP_LEFT_SCREEN
			if _pub_ht:	_ht_gp = random.randint(self.ht_rand_min, self.ht_rand_max)

			## set first goal positions including eye saccade
			_pub_cmd = ""
			if _pub_hp:	_pub_cmd += "HPGP" + str(_hp_gp)
			if _pub_ht:	_pub_cmd += "HTGP" + str(_ht_gp)
			if _pub_ep:	_pub_cmd += "EPGP" + str(_ep_gp_saccade)
			if _pub_ipt:	_pub_cmd += "IPT" + str(_ipt_turn) 
			_pub_cmd += TERM_CHAR_SEND
			rospy.logwarn( _pub_cmd )
			#lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			lookINSPIRE4Interaction.monitorMoveToGP( self, _pub_cmd, hp_gp=_hp_gp, ht_gp=_ht_gp )

			#lookINSPIRE4Interaction.requestFeedback( self, SC_GET_PP )
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
			#lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
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
			#	lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
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
			#		lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			#		_first_pass = False
			#
			#	## set eye pan goal position after completing eye saccade
			#	elif _pub_ep and _ep_phase0:
			#		_pub_cmd = ""
			#		_pub_cmd += "EPGP" + str(EP_FRONT)
			#		_pub_cmd += TERM_CHAR_SEND
			#		rospy.logwarn( _pub_cmd )
			#		lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
			#		_ep_phase0 = False
			#	else:
			#		pass
			#
			#	if (abs(rospy.get_time() - _start_time) < _ipt_turn):
			#		## has 100 ms delay for propogation to motors
			#		lookINSPIRE4Interaction.requestFeedback( self, SC_GET_PP )
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
				self.facing = lookINSPIRE4Interaction.FACING_RIGHT_SCREEN
			else:
				self.facing = lookINSPIRE4Interaction.FACING_LEFT_SCREEN

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

	###########################################
	##
	##	To run, publish to /maki_macro:
	##		turnToInfant
	##		
	## Maki-ro turns back to facing the infant from previously looking at DIRECTION screen
	###########################################
	## This is a work in progress
	def turnToInfant( self ):
		rospy.logdebug("turnToInfant(): BEGIN")

		#lookINSPIRE4Interaction.pubTo_maki_command( self, "reset" )
		#self.SWW_WI.sleepWhileWaitingMS( self.ipt_turn )
		#if True:	return
	
		if (self.facing == lookINSPIRE4Interaction.FACING_INFANT):
			rospy.logwarn("turnToScreen(): WARNING: Maki-ro is reported as already facing " + lookINSPIRE4Interaction.FACING_INFANT)

		_pub_hp = True
		_pub_ht = True
		_pub_ep = False
		_pub_ipt = True
		_ipt_turn = self.ipt_turn

		_pub_cmd = ""

		_start_time = rospy.get_time()
		if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			_loop_count = 0

			if _pub_hp:	_hp_gp = lookINSPIRE4Interaction.HP_FACE_INFANT
			if _pub_ht:	_ht_gp = lookINSPIRE4Interaction.HT_FACE_INFANT
			if _pub_ep:
				### saccade in the direction of head turn
				#if self.facing == lookINSPIRE4Interaction.FACING_LEFT_SCREEN:
				#	_ep_gp = lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_LEFT_SCREEN
				#elif self.facing == lookINSPIRE4Interaction.FACING_RIGHT_SCREEN:
				#	_ep_gp = lookINSPIRE4Interaction.EP_FACE_INFANT_FROM_RIGHT_SCREEN
				### otherwise, return to gazing forward
				#else:
				#	_ep_gp = EP_FRONT
				_ep_gp = EP_FRONT

			if _pub_ipt:
				lookINSPIRE4Interaction.requestFeedback( self, SC_GET_PP )
				if _pub_ep:	_delta_ep_pp = abs(self.makiPP["EP"] - _ep_gp)
				if _pub_hp:	_delta_hp_pp = abs(self.makiPP["HP"] - _hp_gp)
				if _pub_ht:	_delta_ht_pp = abs(self.makiPP["HT"] - _ht_gp)

				if _pub_ep:	_gs_ep = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_ep_pp, _ipt_turn )
				if _pub_hp:	_gs_hp = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_hp_pp, _ipt_turn )
				if _pub_ht:	_gs_ht = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_ht_pp, _ipt_turn )

			_pub_cmd = ""
			if _pub_hp:	_pub_cmd += "HPGP" + str(_hp_gp)
			if _pub_ht:	_pub_cmd += "HTGP" + str(_ht_gp)
			if _pub_ep:	_pub_cmd += "EPGP" + str(_ep_gp)
			if _pub_ipt:	_pub_cmd += str(SC_SET_IPT) + str(_ipt_turn)
			_pub_cmd += TERM_CHAR_SEND
			rospy.logwarn( _pub_cmd )
			#_start_time = rospy.get_time()
			#lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )

			_try_count = 0
			_try_max = 3
			while (self.ALIVE and (not rospy.is_shutdown()) and 
				(_pub_cmd != TERM_CHAR_SEND) and (_try_count <= _try_max)):
				try:
					## NOTE: This extra pubTo_maki_command is a bit of a cheat
					lookINSPIRE4Interaction.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )

					lookINSPIRE4Interaction.monitorMoveToGP( self, _pub_cmd, hp_gp=_hp_gp )
					break	## exit while loop
				except rospy.exceptions.ROSException as _e:
					_try_count += 1
					if (_try_count < _try_max):
						rospy.logwarn("turnToInfant(): WARNING: May not have successfully moved to face infant... TRYING AGAIN: " + str(_try_count) + " of " + str(_try_max))
					else:
						rospy.logerr("turnToInfant(): ERROR: Unable to face infant!!!!!!!!!!!!!!!!! " + str(_e))
						return
		
			### now move eye pan back to center from saccade
			#_ep_gp = EP_FRONT
			#_delta_ep_pp = abs(self.makiPP["EP"] - _ep_gp)
			#_gs_ep = self.DC_helper.getGoalSpeed_ticks_durationMS( _delta_ep_pp, abs(_ipt_turn - abs(rospy.get_time() - _start_time )) )
			#_pub_cmd = ""
			#_pub_cmd += "EPGS" + str(_gs_ep)
			#_pub_cmd += "EPGP" + str(_ep_gp)
			#_pub_cmd += "HPGP" + str(_hp_gp)	## superfluous
			#_pub_cmd += "HTGP" + str(_ht_gp)	## superfluous
			#_pub_cmd += TERM_CHAR_SEND
			#rospy.logwarn( _pub_cmd )
			#lookINSPIRE4Interaction.monitorMoveToGP( self, _pub_cmd, ep_gp=_ep_gp, hp_gp=_hp_gp, ht_gp=_ht_gp )

			self.facing = lookINSPIRE4Interaction.FACING_INFANT
		else:
			rospy.logwarn("Cannot turnToInfant. Publish 'interaction start' first")
			return
		#end	if (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):

		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo( "NUMBER OF TIMESTEPS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("turnToInfant(): END")
		return


	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "interaction start":
			## try to nicely startup without jerking MAKI's head tilt servo
			lookINSPIRE4Interaction.start( self )

		elif msg.data.startswith( "interaction stop" ):
			if msg.data.endswith( "disable_ht=False" ):
				lookINSPIRE4Interaction.stop( self, disable_ht=False )
			else:
				lookINSPIRE4Interaction.stop( self )

		elif msg.data.startswith( "turnToScreen left" ):
			lookINSPIRE4Interaction.turnToScreen( self, right_screen=False )

		elif msg.data.startswith( "turnToScreen right" ):
			lookINSPIRE4Interaction.turnToScreen( self, right_screen=True )

		elif msg.data == "turnToInfant":
			lookINSPIRE4Interaction.turnToInfant( self )

		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	lookStimuli = lookINSPIRE4Interaction( True, None )

	rospy.Subscriber( "/maki_macro", String, lookStimuli.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"

