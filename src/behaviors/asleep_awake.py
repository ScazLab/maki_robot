#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string
import thread
import random

from makicom.msg import MakiCommand, MakiFeedback
from makicom import INVALID_INT, EYELID_RIGHT, EYELID_LEFT, EYE_PAN, EYE_TILT, HEAD_TILT, HEAD_PAN
from makicom import MAX_POS, MIN_POS, PRESENT_POS, GOAL_POS, PRESENT_SPEED, GOAL_SPEED, PRESENT_TEMP, PRESENT_LOAD, TORQUE_MAX, TORQUE_LIM, TORQUE_ENABLE, MOVING, ERROR, DEFAULT_POS, DEFAULT_SPEED
from makicom import feedbackCommand, setCommand, newMakiFeedback, getFeedback, updateFeeback

import dynamixel_conversions as DX_Conv

__is_asleep = None
__is_awake = None

########################
## Description: 
##	INSPIRE4 experiment begins with Maki-ro asleep and waking up
##	and ends with Maki-ro going back to sleep
##
##	ASLEEP: from neutral, MAKI closes eyes and 
##	puts head down and to its left
##
##	AWAKE: from ASLEEP, MAKI opens eyes and raises head 
##	to neutral and faces front
########################
class asleepAwake( eyelidHeadTiltBaseBehavior, headPanBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value

	## specify base class
	def start( self, enable_ht=True ):
		return eyelidHeadTiltBaseBehavior.start( self, enable_ht=enable_ht )

	## specify base blass
	def stop( self, disable_ht=True ):
		return eyelidHeadTiltBaseBehavior.stop( self, disable_ht=disable_ht )

	## specify base class
	def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.01 ):
		return headPanBaseBehavior.pubTo_maki_command( self, commandOut=commandOut, fixed_gaze=fixed_gaze, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )

	## specify base class
	def monitorMoveToGP( self, commandOut, hp_gp=None, ht_gp=None, ep_gp=None, et_gp=None, ll_gp=None, lr_gp=None, delta_pp=DELTA_PP, cmd_prop=True ):
		return headPanBaseBehavior.monitorMoveToGP( self, gp_cmd=commandOut, hp_gp=hp_gp, ht_gp=ht_gp, ep_gp=ep_gp, et_gp=et_gp, ll_gp=ll_gp, lr_gp=lr_gp, delta_pp=delta_pp, cmd_prop=cmd_prop )


	## --------------------------
	def asleep_p( self ):
		if (isinstance( asleepAwake.__is_asleep, bool )):
			return asleepAwake.__is_asleep
		else:
			return False

	def awake_p( self ):
		if (isinstance( asleepAwake.__is_awake, bool )):
			return asleepAwake.__is_awake
		else:
			return False

	####################
	##
	##	To run, publish to /maki_macro:
	##		asleep 
	##
	####################
	def macroAsleep( self ):
		rospy.logdebug("macroAsleep(): BEGIN")

		_pub_hp = True
		_pub_ht = True
		_pub_ll = True
		_pub_ep = False
		_pub_et = True
		_pub_ipt = True	#False

		_ms_duration = 2000
		_pub_cmd = ""
		_start_time = rospy.get_time()

		if (self.ALIVE and (not self.mTT_INTERRUPT) and
			(not rospy.is_shutdown())):

			## FIRST! Check to see if we are already positioned in asleep
			if asleepAwake.verifyPose( self, ll=asleepAwake.LL_ASLEEP, ht=asleepAwake.HT_ASLEEP, hp=asleepAwake.HP_ASLEEP ):	
				rospy.logdebug("macroAsleep(): already asleep... SKIP!!!!!")
				asleepAwake.__is_asleep = True
				asleepAwake.__is_awake = False
				return

			asleepAwake.requestFeedback( self, SC_GET_PP )
			if _pub_ll:
				## present position to LL_CLOSE_MAX
				_delta_ticks_LL = abs( self.makiPP["LL"] - asleepAwake.LL_ASLEEP )
			if _pub_ht:
				## present position to HT_DOWN
				_delta_ticks_HT = abs( self.makiPP["HT"] - asleepAwake.HT_ASLEEP )
			if _pub_hp:
				## present position to HP_LEFT
				_delta_ticks_HP = abs( self.makiPP["HP"] - asleepAwake.HP_ASLEEP )
			if _pub_et:
				## present position to ET_DOWN
				_delta_ticks_ET = abs( self.makiPP["ET"] - asleepAwake.ET_ASLEEP )
			if _pub_ep:
				## present position to EP_LEFT
				_delta_ticks_EP = abs( self.makiPP["EP"] - asleepAwake.EP_ASLEEP )
			#rospy.logerr("delta_ticks[LL, HT, HP, ET, EP] = " + str(_delta_ticks_LL)
			#	+ ", " + str(_delta_ticks_HT)
			#	+ ", " + str(_delta_ticks_HP)
			#	+ ", " + str(_delta_ticks_ET)
			#	+ ", " + str(_delta_ticks_EP) )

			if not _pub_ipt:
				## calculate GoalSpeed
				if _pub_ht and (_delta_ticks_HT>0):	_gs_HT = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_HT, _ms_duration) ) + 1
				if _pub_hp and (_delta_ticks_HP>0):	_gs_HP = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_HP, _ms_duration) ) + 1
				## eyes should finish first
				_ms_duration_eyes = int( (float(_ms_duration) * 0.3) + 0.5 )
				if _pub_ll and (_delta_ticks_LL>0):	_gs_LL = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_LL, _ms_duration_eyes) ) + 1
				if _pub_et and (_delta_ticks_ET>0):	_gs_ET = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_ET, _ms_duration_eyes) ) + 1
				if _pub_ep and (_delta_ticks_EP>0):	_gs_EP = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_EP, _ms_duration_eyes) ) + 1
				rospy.logdebug("ms_duration_eyes = " + str(_ms_duration_eyes) + "ms")

				## preset eyelid to open GoalSpeed
				if _pub_ll and (_delta_ticks_LL>0):	_pub_cmd += "LL" + SC_SET_GS + str(_gs_LL) 
				if _pub_ht and (_delta_ticks_HT>0):	_pub_cmd += "HT" + SC_SET_GS + str(_gs_HT)
				if _pub_hp and (_delta_ticks_HP>0):	_pub_cmd += "HP" + SC_SET_GS + str(_gs_HP)
				if _pub_et and (_delta_ticks_ET>0):	_pub_cmd += "ET" + SC_SET_GS + str(_gs_ET)
				if _pub_ep and (_delta_ticks_EP>0):	_pub_cmd += "EP" + SC_SET_GS + str(_gs_EP)
				_pub_cmd += str(TERM_CHAR_SEND)
				asleepAwake.pubTo_maki_command( self, str(_pub_cmd) )

			## publish GoalPosition: head down, eyes down, eyelids closed
			_pub_cmd = ""
			if _pub_ht:	_pub_cmd += "HT" + SC_SET_GP + str(asleepAwake.HT_ASLEEP)
			if _pub_hp:	_pub_cmd += "HP" + SC_SET_GP + str(asleepAwake.HP_ASLEEP)
			if _pub_ll:	_pub_cmd += "LL" + SC_SET_GP + str(asleepAwake.LL_ASLEEP)
			if _pub_et:	_pub_cmd += "ET" + SC_SET_GP + str(asleepAwake.ET_ASLEEP)
			if _pub_ep:	_pub_cmd += "EP" + SC_SET_GP + str(asleepAwake.EP_ASLEEP)
			if _pub_ipt:	_pub_cmd += SC_SET_IPT + str(_ms_duration)
			_pub_cmd += TERM_CHAR_SEND
			#asleepAwake.pubTo_maki_command( self, str(_pub_cmd) )
			try:
				_start_time_pub = rospy.get_time()
				asleepAwake.monitorMoveToGP( self, str(_pub_cmd), hp_gp=asleepAwake.HP_ASLEEP, ht_gp=asleepAwake.HT_ASLEEP, ll_gp=asleepAwake.LL_ASLEEP )
			except Exception as _e:
				#rospy.logerr("macroAsleep(): ERROR: " + str(_e) )
				rospy.logwarn("macroAsleep(): WARNING: " + str(_e) )
				asleepAwake.pubTo_maki_command( self, str(_pub_cmd) )
				_adjust_sleep_ms = _ms_duration - int(((rospy.get_time() - _start_time_pub) * 1000) + 0.5)
				rospy.logdebug("macroAsleep(): _adjust_sleep_ms = " + str(_adjust_sleep_ms))
				self.SWW_WI.sleepWhileWaitingMS( _adjust_sleep_ms, increment=0.01 )

		else:
			return

		_duration = rospy.get_time() - _start_time
		rospy.logdebug( "DURATION: " + str(_duration) + " seconds" )

		asleepAwake.__is_asleep = True
		asleepAwake.__is_awake = False

		rospy.logdebug("macroAsleep(): END")


	####################
	##
	##	To run, publish to /maki_macro:
	##		awake
	##	This behavior will result with Maki-ro in neutral position
	##
	##	To run, publish to /maki_macro:
	##		awake experimenter
	##	This behavior will result with Maki-ro looking at the experimenter
	##
	####################
	def macroAwake( self, look_experimenter=False ):
		rospy.logdebug("macroAwake(): BEGIN")

		_pub_hp = True
		_pub_ht = True
		_pub_ll = True
		_pub_ep = False
		_pub_et = True
		_pub_ipt = True	#False

		_ms_duration = 1000
		if look_experimenter:	_ms_duration = 2 * _ms_duration
		_pub_cmd = ""
		_start_time = rospy.get_time()

		if (self.ALIVE and (not self.mTT_INTERRUPT) and
			(not rospy.is_shutdown())):

			asleepAwake.requestFeedback( self, SC_GET_PP )
			if _pub_ll:
				## present position to LL_AWAKE
				LL_AWAKE = asleepAwake.LL_AWAKE
				if look_experimenter:	LL_AWAKE = asleepAwake.LL_AWAKE_EXP
				_delta_ticks_LL = abs( self.makiPP["LL"] - LL_AWAKE )
			if _pub_ht:
				## present position to HT_AWAKE
				HT_AWAKE = asleepAwake.HT_AWAKE
				if look_experimenter:	HT_AWAKE = asleepAwake.HT_AWAKE_EXP
				_delta_ticks_HT = abs( self.makiPP["HT"] - HT_AWAKE )
			if _pub_hp:
				## present position to HP_AWAKE
				HP_AWAKE = asleepAwake.HP_AWAKE
				if look_experimenter:	HP_AWAKE = asleepAwake.HP_AWAKE_EXP
				_delta_ticks_HP = abs( self.makiPP["HP"] - HP_AWAKE )
			if _pub_et:
				## present position to ET_AWAKE
				ET_AWAKE = asleepAwake.ET_AWAKE
				if look_experimenter:	ET_AWAKE = asleepAwake.ET_AWAKE_EXP
				_delta_ticks_ET = abs( self.makiPP["ET"] - ET_AWAKE )
			if _pub_ep:
				## present position to EP_AWAKE
				EP_AWAKE = asleepAwake.EP_AWAKE
				if look_experimenter:	EP_AWAKE = asleepAwake.EP_AWAKE_EXP
				_delta_ticks_EP = abs( self.makiPP["EP"] - EP_AWAKE )
			#rospy.logerr("delta_ticks[LL, HT, HP, ET, EP] = " + str(_delta_ticks_LL)
			#	+ ", " + str(_delta_ticks_HT)
			#	+ ", " + str(_delta_ticks_HP)
			#	+ ", " + str(_delta_ticks_ET)
			#	+ ", " + str(_delta_ticks_EP) )

			if not _pub_ipt:
				## calculate GoalSpeed
				if _pub_ht and (_delta_ticks_HT>0):	_gs_HT = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_HT, _ms_duration) ) + 1
				if _pub_hp and (_delta_ticks_HP>0):	_gs_HP = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_HP, _ms_duration) ) + 1
				## eyes should finish first
				_ms_duration_eyes = int( (float(_ms_duration) * 0.3) + 0.5 )
				if _pub_ll and (_delta_ticks_LL>0):	_gs_LL = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_LL, _ms_duration_eyes) ) + 1
				if _pub_et and (_delta_ticks_ET>0):	_gs_ET = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_ET, _ms_duration_eyes) ) + 1
				if _pub_ep and (_delta_ticks_EP>0):	_gs_EP = int( DX_Conv.getGoalSpeed_ticks_durationMS(_delta_ticks_EP, _ms_duration_eyes) ) + 1
				rospy.logdebug("ms_duration_eyes = " + str(_ms_duration_eyes) + "ms")

				## preset eyelid to open GoalSpeed
				if _pub_ll and (_delta_ticks_LL>0):	_pub_cmd += "LL" + SC_SET_GS + str(_gs_LL) 
				if _pub_ht and (_delta_ticks_HT>0):	_pub_cmd += "HT" + SC_SET_GS + str(_gs_HT)
				if _pub_hp and (_delta_ticks_HP>0):	_pub_cmd += "HP" + SC_SET_GS + str(_gs_HP)
				if _pub_et and (_delta_ticks_ET>0):	_pub_cmd += "ET" + SC_SET_GS + str(_gs_ET)
				if _pub_ep and (_delta_ticks_EP>0):	_pub_cmd += "EP" + SC_SET_GS + str(_gs_EP)
				_pub_cmd += str(TERM_CHAR_SEND)
				asleepAwake.pubTo_maki_command( self, str(_pub_cmd) )

			## publish GoalPosition: head down, eyes down, eyelids open
			_pub_cmd = ""
			if _pub_ht:	_pub_cmd += "HT" + SC_SET_GP + str(HT_AWAKE)
			if _pub_hp:	_pub_cmd += "HP" + SC_SET_GP + str(HP_AWAKE)
			if _pub_ll:	_pub_cmd += "LL" + SC_SET_GP + str(LL_AWAKE)
			if _pub_et:	_pub_cmd += "ET" + SC_SET_GP + str(ET_AWAKE)
			if _pub_ep:	_pub_cmd += "EP" + SC_SET_GP + str(EP_AWAKE)
			if _pub_ipt:	_pub_cmd += SC_SET_IPT + str(_ms_duration)
			_pub_cmd += TERM_CHAR_SEND
			rospy.logdebug( "macroAwake(): _pub_cmd = " + _pub_cmd )

## KATE
			## TODO: Try uncommented to see if Maki-ro responds more quickly to the issued command
			#asleepAwake.pubTo_maki_command( self, str(_pub_cmd) )
			_start_time_pub = rospy.get_time()
			try:
				asleepAwake.monitorMoveToGP( self, str(_pub_cmd), hp_gp=HP_AWAKE, ht_gp=HT_AWAKE, ll_gp=LL_AWAKE )
			except Exception as _e:
				#rospy.logerr("macroAwake(): ERROR: " + str(_e) )
				rospy.logwarn("macroAwake(): WARNING: " + str(_e) )
				asleepAwake.pubTo_maki_command( self, str(_pub_cmd) )
				_adjust_sleep_ms = _ms_duration - int(((rospy.get_time() - _start_time_pub) * 1000) + 0.5)
				rospy.logdebug("macroAwake(): _adjust_sleep_ms = " + str(_adjust_sleep_ms))
				if _adjust_sleep_ms > 0:
					self.SWW_WI.sleepWhileWaitingMS( _adjust_sleep_ms, increment=0.01 )

		else:
			return

		_duration = rospy.get_time() - _start_time
		rospy.logdebug( "DURATION: " + str(_duration) + " seconds" )

		asleepAwake.__is_awake = True
		asleepAwake.__is_asleep = False

		rospy.logdebug("macroAwake(): END")

	def runAsleep( self ):
		asleepAwake.start( self )
		asleepAwake.macroAsleep( self )
		asleepAwake.stop( self, disable_ht=True )
		## TODO: May need to change to False when whole INSPIRE4
		##	script and control program are in place

	def runAwake( self, disable_ht=True ):
		asleepAwake.start( self )
		asleepAwake.macroAwake( self, look_experimenter=False )
		## TODO: Maki-ro doesn't quite return to HT_MIDDLE
		asleepAwake.pubTo_maki_command( self, "reset" )
		asleepAwake.stop( self, disable_ht=disable_ht )
		## TODO: May need to change to False when whole INSPIRE4
		##	script and control program are in place
        

	def runAwakeExperimenter( self, disable_ht=True ):
		asleepAwake.start( self )
		asleepAwake.macroAwake( self, look_experimenter=True )
		asleepAwake.stop( self, disable_ht=disable_ht )

	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "asleep":
			asleepAwake.runAsleep( self )

		elif msg.data == "awake":
			asleepAwake.runAwake( self )

		elif msg.data.startswith( "awake experimenter" ):
			if msg.data.endswith( "disable_ht=False" ):
				asleepAwake.runAwakeExperimenter( self, disable_ht=False )
			## TODO: May need to change to False when whole INSPIRE4
			##	script and control program are in place
			else:
				asleepAwake.runAwakeExperimenter( self, disable_ht=True )

if __name__ == '__main__':
    print "__main__: BEGIN"
	AA = asleepAwake( True, None )

	rospy.Subscriber( "/maki_macro", String, AA.parse_maki_macro )
    rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

    print "__main__: END"




