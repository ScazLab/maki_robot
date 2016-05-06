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
	v3 = None

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		## different versions of head nodding
		headNod.v1 = False
		headNod.v2 = False
		headNod.v3 = True

		self.repetition = 5	#1

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
			#_delta_ht_pp = abs( _ht_pp - HT_MIDDLE )
			_delta_ht_pp = 5
			_ht_macro_pose = HT_MIDDLE
			if ( abs( _ht_pp - HT_UP ) < _delta_ht_pp ):
				_delta_ht_pp = abs( _ht_pp - HT_UP )
				_ht_macro_pose = HT_UP
			if ( abs( _ht_pp - HT_DOWN ) < _delta_ht_pp ):
				_delta_ht_pp = abs( _ht_pp - HT_DOWN )
				_ht_macro_pose = HT_DOWN
			rospy.logdebug( "_ht_macro_pose = " + str(_ht_macro_pose) )
			## publish GP of _ht_macro_pose in case in between poses
			baseBehavior.pubTo_maki_command( self, "HT" + SC_SET_GP + str(_ht_macro_pose) + TERM_CHAR_SEND, cmd_prop=False )

			if (_ht_macro_pose == HT_MIDDLE):	
				## START NOD FROM HEAD UP POSITION
				## looking up first has strongest nodding cue
				while ( abs( self.makiPP["HT"] - HT_UP) > _delta_ht_pp ):
					baseBehavior.pubTo_maki_command( self, str(self.nod_middle_up), time_ms=self.ipt_nod_middle_up, time_inc=0.005 )
					#baseBehavior.pubTo_maki_command( self, str(self.nod_middle_up), time_ms=IPT_NOD, time_inc=0.005 )

			rospy.logdebug("Entering macroHeadNod inner while loop")
			_loop_count = 0
			while (not self.mTT_INTERUPT) and (_loop_count < self.repetition):
				_loop_count = _loop_count + 1
				rospy.loginfo("-------" + str(_loop_count) + " start ----------")

				## VERSION 1: FROM UP --> MIDDLE --> DOWN --> 
				if headNod.v1:
					baseBehavior.pubTo_maki_command( self, str(self.nod_up_middle), time_ms=self.ipt_nod_middle_up, time_inc=0.001 )
	
					baseBehavior.pubTo_maki_command( self, str(self.nod_middle_down), time_ms=self.ipt_nod_middle_down, time_inc=0.001 )

					baseBehavior.pubTo_maki_command( self, str(self.nod_middle_down), time_ms=self.ipt_nod_middle_down, time_inc=0.001 )

		

				## VERSION 2: UP --> DOWN --> 
				if headNod.v2:
					#baseBehavior.pubTo_maki_command( self, str(self.nod_down_up), time_ms=IPT_NOD )
					baseBehavior.pubTo_maki_command( self, str(self.nod_down_up), time_ms=self.ipt_nod_middle_up )

					baseBehavior.pubTo_maki_command( self, str(self.nod_up_down), time_ms=IPT_NOD )


				## VERSION 3: FROM UP --> DOWN --> 
				if headNod.v3:
					baseBehavior.pubTo_maki_command( self, str(self.nod_up_down), time_ms=IPT_NOD )


				## ALL: DOWN --> UP -->
				if (headNod.v1 or headNod.v2 or headNod.v3) and (_loop_count < self.repetition):
					baseBehavior.pubTo_maki_command( self, str(self.nod_down_up), time_ms=IPT_NOD )

			#end	while not self.mTT_INTERUPT:

			## ALL: DOWN --> MIDDLE
			### time_ms = IPT_NOD needed to allow head tilt to recenter to middle
			#baseBehavior.pubTo_maki_command( self, str(self.nod_down_middle), time_ms=IPT_NOD )
			while ( abs( self.makiPP["HT"] - HT_MIDDLE) > _delta_ht_pp ):
				baseBehavior.pubTo_maki_command( self, str(self.nod_down_middle), time_ms=self.ipt_nod_middle_up, time_inc=0.005 )

			rospy.loginfo("-----------------")
		#end	if self.ALIVE:

		rospy.logdebug("macroHeadNod: END")
		return

	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "nod":
			### try to nicely startup headnod testing without jerking MAKI's head tilt servo
			headTiltBaseBehavior.start(self)
			self.macroHeadNod()
			headTiltBaseBehavior.stop(self)
		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	nod = headNod( True, None )

	rospy.Subscriber( "/maki_macro", String, nod.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


