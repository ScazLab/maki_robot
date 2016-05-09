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
## Selective attention behaviors
##
## VISUAL SCANNING:
##	Definition: From "Encyclopedia of Child Behavior and Development"
##	by Benjamin K. Barton and Paul Jorritsma, pages 1546-1548.
##	Visual scanning refers to the pattern of fixations and saccades
##	while an individual is examiining visual stimuli. Scanning is
##	first rather disorganized in infants, then becomes more
##	exploratory but fairly limited, and finally develops into
##	controlled, goal-directed behavior. In older children and adults
##	visual scanning emerges as more sohpisticated visual search skills
##	and is part of a larger set of selective attention abilities.
##
##	Description: Maki-ro moves its eyes +/- delta around its neutral
##	eye pan and eye tilt positions
##
## TODO: 
##	* Eye tilt (ET) and/or eyelid (LL) compensates for HT
## 	* Gaussiaan distribution towards eye pan/tilt neutral
##
## MOTION CRITIQUE:
##
########################
class selectiveAttention( headTiltBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	# none

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass
		self.DC_helper = dynamixelConversions()

		#self.sww_wi = ROS_sleepWhileWaiting_withInterrupt( verbose_debug=self.VERBOSE_DEBUG )
		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		## set random range for eye pan/tilt location centered around neutral
		## 30 ticks looks paranoid
		#self.ep_delta_range = 30	## ticks
		#self.et_delta_range = 30	## ticks	
		## 10 ticks looks like visual scanning on a focused point
		self.ep_delta_range = 10	## ticks
		self.et_delta_range = 20	#15	#10	## ticks	
		## in addition to 100ms command propogation delay, provide option
		## additional rest period
		self.visual_scan_rest_occurence_percent = 35	#25	## [1,100)
		self.visual_scan_rest_enabled = True
		self.visual_scan_rest_min = 100	#50	## milliseconds
		self.visual_scan_rest_max = 400	#300	## milliseconds
		self.ep_rand_min = EP_FRONT - self.ep_delta_range
		self.ep_rand_max = EP_FRONT + self.ep_delta_range
		self.et_rand_min = ET_MIDDLE - self.et_delta_range
		self.et_rand_max = ET_MIDDLE + self.et_delta_range
		## check rand range values
		if self.ep_rand_min < EP_LEFT:	self.ep_rand_min = EP_LEFT
		if self.ep_rand_max > EP_RIGHT:	self.ep_rand_max = EP_RIGHT
		if self.et_rand_min < ET_DOWN:	self.et_rand_min = ET_DOWN
		if self.et_rand_max > ET_UP:	self.et_rand_max = ET_UP
		rospy.logdebug("EPan range: (" + str(self.ep_rand_min) + ", " + str(self.ep_rand_max) + ")")
		rospy.logdebug("ETilt range: (" + str(self.et_rand_min) + ", " + str(self.et_rand_max) + ")")

		self.ALIVE = True
		return


	###########################
	##
	##	To run, publish to /maki_macro
	##		visualScan start
	##
	##	To stop, publish to /maki_macro
	##		visualScan stop
	##
	###########################
	def macroVisualScan( self ):
		rospy.logdebug("macroVisualScan: BEGIN")

		## this is a nested while loop
		selectiveAttention.requestFeedback( self, SC_GET_PP )
		_previous_ep = self.makiPP["EP"]
		_previous_et = self.makiPP["ET"]
		_loop_count = 0
		_start_time = rospy.get_time()
		while self.ALIVE and not rospy.is_shutdown():

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(self.mTT_INTERRUPT))
				##print "start sleep 5"
				#self.sww_wi.sleepWhileWaiting(5)	# 5 seconds
				##print "end sleep 5"
				#continue	## begin loop again from the beginning skipping below
				##print "shouldn't get here"
				return

			### FPPZ request published every 100ms (10Hz)
			selectiveAttention.requestFeedback( self, SC_GET_PP )
			#selectiveAttention.requestFeedback( self, SC_GET_PP, time_ms=200 )

			## choose a random eye pan/tilt location within full range
			#self.et_rand = random.randint(ET_DOWN, ET_UP)
			#self.ep_rand = random.randint(EP_LEFT, EP_RIGHT)

			## TODO: Gaussiaan distribution towards eye pan/tilt neutral
			## random.randint is a uniform distribution
			self.et_rand = random.randint(self.et_rand_min, self.et_rand_max)
			self.ep_rand = random.randint(self.ep_rand_min, self.ep_rand_max)

			## TODO: Compensate eyelid position based on eye tilt
			_delta_et = self.et_rand - _previous_et
			_delta_ep = self.ep_rand - _previous_ep
			rospy.logdebug( "EPan = (" + str(_previous_ep) + ", " + str(self.ep_rand) + ", _delta_ep=" + str(_delta_ep) + ")")
			rospy.logdebug( "ETilt = (" + str(_previous_et) + ", " + str(self.et_rand) + ", _delta_et=" + str(_delta_et) + ")")

			_pub_cmd = "EPGP" + str(self.ep_rand) + "ETGP" + str(self.et_rand) + str(TERM_CHAR_SEND) 
			selectiveAttention.monitorMoveToGP( self, _pub_cmd, ep_gp=self.ep_rand, et_gp=self.et_rand )

			## in addition to 100ms command propogation delay, provide option
			## additional rest period
			if self.visual_scan_rest_enabled:
				_tmp_rest = random.randint(self.visual_scan_rest_min, self.visual_scan_rest_max)
				#rospy.logdebug("Extra rest is " + str(_tmp_rest) + " ms")
				rospy.logerr("Extra rest is " + str(_tmp_rest) + " ms")
				self.SWW_WI.sleepWhileWaitingMS( _tmp_rest, end_early=False)

			if (_loop_count % random.randint(5,10) == 0):
				_tmp_rest = random.randint(300, 800)
				rospy.logerr("_loop_count extra rest is " + str(_tmp_rest) + " ms")
				self.SWW_WI.sleepWhileWaitingMS( _tmp_rest, end_early=False)
				#_pub_cmd = "EPGP" + str(self.ep_rand) + "ETGP" + str(self.etrand) + str(TERM_CHAR_SEND) 
				#selectiveAttention.monitorMoveToGP( self, _pub_cmd, ep_gp=self.ep_rand, et_gp=self.etrand )

			if (random.randrange(1,100) < self.visual_scan_rest_occurence_percent):	
				self.visual_scan_rest_enabled=True
			else:
				self.visual_scan_rest_enabled=False

			## store previous values
			_previous_ep = self.makiPP["EP"]
			_previous_et = self.makiPP["ET"]

			_loop_count = _loop_count +1
			_duration = abs(rospy.get_time() - _start_time)
		#end	while self.ALIVE and not rospy.is_shutdown():

		rospy.loginfo( "NUMBER OF VISUAL SCAN MOVMENTS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )
		return

	def parse_maki_macro( self, msg ):
		print msg.data

		if msg.data == "visualScan start":
			## Maki-ro doesn't need head tilt for visual scanning
			headTiltBaseBehavior.start(self, enable_ht=False)
			self.macroVisualScan()
		elif msg.data == "visualScan stop":
			headTiltBaseBehavior.stop(self)
		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	SA = selectiveAttention( True, None )

	rospy.Subscriber( "/maki_macro", String, SA.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


