#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * 	## classes baseBehavior and headTiltBaseBehavior


########################
## Maki-ro's "lookAt" head / eye pan coordination
##
## Description:
##	Maki-ro's eye pan will move rapidly towards "gaze target"
##	Then as head pan moves and comes into alignment with "gaze
##	target", eye pan will counter rotate
##
## TODO: Add corresponding head / eye tilt
##
## Thanks much to Ale!
########################
class lookAt( headTiltBaseBehavior, headPanBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	EP_MIN_LEFT = 460
	EP_MAX_RIGHT = 578

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		headPanBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		## DEFAULT MOTOR SPEEDS
		self.LL_GS_DEFAULT = 100
		self.EP_GS_DEFAULT = 100
		self.ET_GS_DEFAULT = 200
		self.HP_GS_DEFAULT = 15
		self.HT_GS_DEFAULT = 51

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
		return

	## override base class
	def stop( self, disable_ht=True ):
		## call base class' stop
		return headTiltBaseBehavior.stop( self, disable_ht=disable_ht )

	def shiftGazeVelocity( self, hp_gp=None, ep_gp_shift=None, ep_gp_fixed=None, hp_pp=None, ep_pp=None, duration_s=1.0 ):
		## check validity of inputs
		if ((hp_gp == None) or (not isinstance(hp_gp, int))):
			rospy.logerr("shiftGazeVelocity(): INVALID INPUT: hp_gp = " + str(hp_gp) + "; expected int")
			return None
		if (ep_gp_fixed == None):
			rospy.logwarn("shiftGazeVelocity(): INPUT NOT SPECIFIED: ep_gp_fixed = " + str(ep_gp_fixed) + "; auto correcting to neutral")
			ep_gp_fixed = EP_FRONT
			
		if ((hp_pp == None) or (not isinstance(hp_pp, int)) or
			(ep_pp == None) or (not isinstance(ep_pp, int))):
			## get most recent values
			lookAt.requestFeedback( SC_GET_PP )
		## if present position is not specified, use the feedback values
		if ((hp_pp == None) or (not isinstance(hp_pp, int))):	hp_pp = self.makiPP["HP"]
		if ((ep_pp == None) or (not isinstance(ep_pp, int))):	ep_pp = self.makiPP["EP"]

		if (ep_gp_shift == None):
			rospy.logwarn("shiftGazeVelocity(): INPUT NOT SPECIFIED: ep_gp_shift = " + str(ep_gp_shift) + "; inferring from head pan present and goal positions")
			if (hp_gp < hp_pp):	## turning to LEFT
				ep_gp_shift = lookAt.EP_MIN_LEFT
			else:	## turning to RIGHT
				ep_gp_shift = lookAt.EP_MAX_RIGHT

		## STEP 0: calculate _delta_hp_pp
		_delta_hp_pp = abs( hp_pp - hp_gp )

		## STEP 1: calculate HP degrees
		_hp_degrees = self.DC_helper.convertToDegrees_ticks( _delta_hp_pp )

		## STEP 2: calculate EP degrees
		_delta_ep_pp_shift = abs( ep_pp - _ep_gp_shift )
		#print _delta_ep_pp_shift
		_ep_shift_degrees = self.DC_helper.convertToDegrees_ticks( _delta_ep_pp_shift )

		## STEP 3: subtract EP degrees
		_counter_angle = abs( _hp_degrees - _ep_shift_degrees )
		#print _counter_angle

		## STEP 4: convert to duration 
		##	AMOUNT OF TIME TO ELAPSE BEFORE COUNTERING EYE PAN
		_counter_ticks = self.DC_helper.convertToTicks_degrees( _counter_angle )
		_counter_ratio = float(_counter_ticks) / float(_delta_hp_pp)
		_counter_duration = _counter_ratio * duration_s	## in seconds
		#print _counter_duration

		## STEP 5: calculate HP goal speed
		_hp_gs = self.DC_helper.getGoalSpeed_ticks_duration( _delta_hp_pp, duration_s )

		## STEP 6: calculate EP goal speed for counter rotation
		##	2*_ep_gs is a good heuristic for shift movement
		_ep_gs = self.DC_helper.getGoalSpeed_ticks_duration( abs(ep_gp_fixed - _ep_gp_shift), (duration_s - _counter_duration) )
		#_ep_gs_shift = 2 * _ep_gs
		_ep_gs_shift = self.DC_helper.getGoalSpeed_ticks_duration( _delta_ep_pp_shift, _counter_duration )


		## STEP 7: print
		rospy.logdebug("t(0): hp_gp=" + str(hp_gp) + ", _hp_gs=" + str(_hp_gs) + "; ep_gp=" + str(_ep_gp_shift) + ", _ep_gs=" + str( _ep_gs_shift ))
		rospy.logdebug("t(" + str(_counter_duration) + "): _ep_gp=" + str(ep_gp_fixed) + ", _ep_gs=" + str(_ep_gs))
		rospy.logdebug("t(" + str(duration_s) + "): DONE")
	

		## NOW ACTUATE THE MOVEMENT
		## send the goal speeds first
		lookAt.pubTo_maki_command( "HPGS" + str(_hp_gs) + "EPGS" + str(_ep_gs_shift) + TERM_CHAR_SEND, fixed_gaze=False )
		_start_time = rospy.get_time()
		## next send the goal positions
		lookAt.pubTo_maki_command( "HPGP" + str(_hp_gp) + "EPGP" + str(_ep_gp_shift) + TERM_CHAR_SEND, fixed_gaze=False )
		## wait until counter duration
		_sleep_duration = _counter_duration - 0.1 	## compensate for the command propogation delay
		rospy.sleep( _sleep_duration )
		lookAt.pubTo_maki_command( "EPGS" + str(_ep_gs) + "EPGP" + str(EP_FRONT) + TERM_CHAR_SEND, fixed_gaze=False )
		## wait until duration_s
		_sleep_duration = duration_s - _sleep_duration - 0.1 	## compensate for the command propogation delay
		if (_sleep_duration > 0):	rospy.sleep( _sleep_duration )

		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo( "Duration: Expected = " + str(duration_s) + " seconds; elapsed = " + str(_duration) + " seconds" )
		return



