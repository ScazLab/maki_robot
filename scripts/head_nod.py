#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

from timeit import default_timer as timer	## wall clock. Unix 1/100 second granularity

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import * 	## classes baseBehavior and headTiltBaseBehavior
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterupt


########################
## Head nodding behavior
##
## Description: from neutral, MAKI nods up (HT_UP)
##	and  then moves to HT_DOWN
##	TODO: Eye tilt (ET) and/or eyelid (LL) compensates for HT
########################
class headNod( headTiltBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	v1 = None
	v2 = None

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		## different versions of head nodding
		headNod.v1 = False
		headNod.v2 = True

		self.sww_wi = ROS_sleepWhileWaiting_withInterupt( verbose_debug=self.VERBOSE_DEBUG )
		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		self.ALIVE = True

		self.headnod_max_min_dist = float( abs( HT_UP - HT_DOWN ) )
		self.headnod_middle_down_dist = float( abs( HT_MIDDLE - HT_DOWN ) )
		self.headnod_middle_up_dist = float( abs( HT_MIDDLE - HT_UP ) )
		## debugging
		#print "headnod_max_min_dist = " 
		#print self.headnod_max_min_dist 
		#print "str(" + str(self.headnod_max_min_dist) + ")"
		##
		#print "headnod_middle_down_dist = " 
		#print self.headnod_middle_down_dist 
		#print "str(" + str(self.headnod_middle_down_dist) + ")"
		##
		#print "headnod_middle_up_dist = " 
		#print self.headnod_middle_up_dist 
		#print "str(" + str(self.headnod_middle_up_dist) + ")"
		##
		#print self.headnod_middle_down_dist / self.headnod_max_min_dist
		#print self.headnod_middle_up_dist / self.headnod_max_min_dist
		self.ipt_nod_middle_down = float( (self.headnod_middle_down_dist / self.headnod_max_min_dist) * IPT_NOD ) 
		self.ipt_nod_middle_up = float( (self.headnod_middle_up_dist / self.headnod_max_min_dist) * IPT_NOD ) 
		self.ipt_nod_middle_down = int(self.ipt_nod_middle_down + 0.5)	## implicit rounding
		self.ipt_nod_middle_up = int(self.ipt_nod_middle_up + 0.5)	## implicit rounding
		## debugging
		#rospy.logdebug( "(full)\tIPT_NOD = " + str(IPT_NOD) + "ms" )
		#rospy.logdebug( "(partial)\t ipt_nod_middle_down = " + str(self.ipt_nod_middle_down) + "ms" )
		#rospy.logdebug( "(partial)\t ipt_nod_middle_up = " + str(self.ipt_nod_middle_up) + "ms" )
		#rospy.logdebug( "(summed partial)\t ipt_nod_middle_* = " + str( self.ipt_nod_middle_down + self.ipt_nod_middle_up ) + "ms" )

		## maki_command prefix
		_m_cmd_prefix = "HT" + SC_SET_GP
		## nod macros
		self.nod_up_down = _m_cmd_prefix + str(HT_DOWN) + SC_SET_IPT + str(IPT_NOD) + TERM_CHAR_SEND
		self.nod_down_up = _m_cmd_prefix + str(HT_UP) + SC_SET_IPT + str(IPT_NOD) + TERM_CHAR_SEND
		self.nod_middle_up = _m_cmd_prefix + str(HT_UP) + SC_SET_IPT + str(self.ipt_nod_middle_up) + TERM_CHAR_SEND 
		self.nod_middle_down = _m_cmd_prefix + str(HT_DOWN) + SC_SET_IPT + str(self.ipt_nod_middle_down) + TERM_CHAR_SEND 
		self.nod_up_middle = _m_cmd_prefix + str(HT_MIDDLE) + SC_SET_IPT + str(self.ipt_nod_middle_up) + TERM_CHAR_SEND 
		self.nod_down_middle = _m_cmd_prefix + str(HT_MIDDLE) + SC_SET_IPT + str(self.ipt_nod_middle_down) + TERM_CHAR_SEND 


	###########################
	##
	##	To run, publish to /maki_macro
	##		nod
	##
	###########################
	def macroHeadNod( self ):
		rospy.logdebug("macroHeadNod: BEGIN")

		## this is a nested while loop
		#_print_once = True
		#while self.ALIVE:
		if self.ALIVE:
			if not rospy.is_shutdown():
				rospy.logdebug("NOT rospy.is_shutdown()")
				pass
			else:
				#break	## break out of outer while loop
				return

			#if _print_once:
			#	rospy.logdebug("Entering macroHeadNod outer while loop")
			#	_print_once = False

			if self.mTT_INTERUPT:	
				rospy.logdebug("mTT_INTERUPT=" + str(mTT_INTERUPT))
				##print "start sleep 5"
				#self.sww_wi.sleepWhileWaiting(5)	# 5 seconds
				##print "end sleep 5"
				#continue	## begin loop again from the beginning skipping below
				##print "shouldn't get here"
				return

			## where is MAKI's HT closest to currently? HT_UP, HT_MIDDLE, HT_DOWN
			_ht_pp = self.makiPP["HT"]
			_delta_ht_pp = abs( _ht_pp - HT_MIDDLE )
			_ht_macro_pose = HT_MIDDLE
			if ( abs( _ht_pp - HT_UP ) < _delta_ht_pp ):
				_delta_ht_pp = abs( _ht_pp - HT_UP )
				_ht_macro_pose = HT_UP
			if ( abs( _ht_pp - HT_DOWN ) < _delta_ht_pp ):
				_delta_ht_pp = abs( _ht_pp - HT_DOWN )
				_ht_macro_pose = HT_DOWN
			## publish GP of _ht_macro_pose in case in between poses
			baseBehavior.pubTo_maki_command( self, "HT" + SC_SET_GP + str(_ht_macro_pose) + TERM_CHAR_SEND )

			if (_ht_macro_pose == HT_MIDDLE):	
				## looking up first has strongest nodding cue
				baseBehavior.pubTo_maki_command( self, str(self.nod_middle_up) )
				self.sww_wi.sleepWhileWaitingMS( self.ipt_nod_middle_up )

			#rospy.logdebug("Entering macroHeadNod inner while loop")
			if not self.mTT_INTERUPT:
				rospy.loginfo("-----------------")

				## VERSION 1: UP --> MIDDLE --> DOWN --> MIDDLE --> UP
				if headNod.v1:
					baseBehavior.pubTo_maki_command( self, str(self.nod_up_middle) )
					self.sww_wi.sleepWhileWaitingMS( self.ipt_nod_middle_up )
	
					baseBehavior.pubTo_maki_command( self, str(self.nod_middle_down) )
					self.sww_wi.sleepWhileWaitingMS( self.ipt_nod_middle_down )

					baseBehavior.pubTo_maki_command( self, str(self.nod_middle_down) )
					self.sww_wi.sleepWhileWaitingMS( self.ipt_nod_middle_down )

					baseBehavior.pubTo_maki_command( self, str(self.nod_down_middle) )
					self.sww_wi.sleepWhileWaitingMS( self.ipt_nod_middle_down )
		
					baseBehavior.pubTo_maki_command( self, str(self.nod_middle_up) )
					self.sww_wi.sleepWhileWaitingMS( self.ipt_nod_middle_up )

				## VERSION 2: UP --> DOWN --> MIDDLE
				if headNod.v2:
					baseBehavior.pubTo_maki_command( self, str(self.nod_down_up) )
					self.sww_wi.sleepWhileWaitingMS( IPT_NOD )

					baseBehavior.pubTo_maki_command( self, str(self.nod_up_down) )
					self.sww_wi.sleepWhileWaitingMS( IPT_NOD )

					baseBehavior.pubTo_maki_command( self, str(self.nod_down_middle) )
					#self.sww_wi.sleepWhileWaitingMS( self.ipt_nod_middle_down )
					self.sww_wi.sleepWhileWaitingMS( IPT_NOD )

				rospy.loginfo("-----------------")


				### try to nicely end testing the headnod
				#if self.mTT_INTERUPT:
				#	self.sww_wi.sleepWhileWaiting(1)	## make sure to wait for message to reach head tilt Dynamixel servo
				#	baseBehavior.pubTo_maki_command( self, "reset" )
				#	self.sww_wi.sleepWhileWaiting(1)	## make sure to wait for message to reach head tilt Dynamixel servo
				#	headTiltBaseBehavior.disableHT( self )

			#end	while not self.mTT_INTERUPT:
			#headTiltBaseBehavior.disableHT( self )

		#end	while self.ALIVE:
		#headTiltBaseBehavior.disableHT( self )

		rospy.logdebug("macroHeadNod: END")
		return

	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "nod":
			### try to nicely startup headnod testing without jerking MAKI's head tilt servo
			headTiltBaseBehavior.start(self, self.makiPP)
			self.macroHeadNod()
			headTiltBaseBehavior.stop(self)
		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	nod = headNod( True, None )

	rospy.Subscriber( "/maki_macro", String, nod.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	## TODO: subscribe to /maki_feedback_pres_pos
	## TODO: update self.makiPP from /maki_feedback_pres_pos

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


