#! /usr/bin/env python

import rospy
import re
from std_msgs.msg import String
from std_srvs.srv import Empty

import signal
import sys
import string
import random
import thread
#import threading

from maki_robot_common import *
#from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

from base_behavior import *

###################################################
##
##	idleHeadPan class will create slight head pan (HP) movement
##		around its present position. Will terminate if the
##		'for loop' is interrupted or times out (100 repetitions)
##
###################################################
class idleHeadPan( headPanBaseBehavior ):

	__is_idling = None
	__pause_idling = None

	def __init__( self, verbose_debug, ros_pub ):
		headPanBaseBehavior.__init__( self, verbose_debug, ros_pub )

		self.ALIVE = False

		idleHeadPan.__is_idling = False
		idleHeadPan.__pause_idling = False
		self.origin_hp = HP_FRONT

		self.delta_HP_idle = 10
        	self.idle_ipt = 600
        	self.delta_idle_ipt = 100

		self.idle_hp_pause_timer = None
		self.idle_hp_resume_timer = None
		return

	def start( self ):
		self.ALIVE = True
		idleHeadPan.__is_idling = False
		idleHeadPan.__pause_idling = False
		return

	def abort( self ):
		self.ALIVE = False
		idleHeadPan.__is_idling = False
		idleHeadPan.__pause_idling = False
		idleHeadPan.cancelIdlingPauseResumeTimers( self )
		return

	def isIdling( self ):
		return idleHeadPan.__is_idling

	def pauseIdling( self ):
		idleHeadPan.__pause_idling = True
		return

	def resumeIdling( self ):
		idleHeadPan.__pause_idling = False
		return

	def idlingPauseTimer_callback( self, event ):
		return idleHeadPan.pauseIdling( self )

	def idlingResumeTimer_callback( self, event ):
		return idleHeadPan.resumeIdling( self )

	## * set timers to pause and resume the idling HP behavior
	## * used for withholding messages published to /maki_command
	## 	while another behavior publishes there
	##
	## NOTE: pause_duration > 0	## seconds
	def setIdlingPauseResumeTimers( self, pause_duration=1, resume_duration=1 ):
		idleHeadPan.cancelIdlingPauseResumeTimers( self )

		if idleHeadPan.isIdling( self ):
			self.idle_hp_pause_timer = rospy.Timer(rospy.Duration(pause_duration), self.idlingPauseTimer_callback, oneshot=True)
			self.idle_hp_resume_timer = rospy.Timer(rospy.Duration(pause_duration+resume_duration), self.idlingResumeTimer_callback, oneshot=True)
		return

	def cancelIdlingPauseResumeTimers( self ):
		## neutralize outstanding timers
		if self.idle_hp_pause_timer != None:
			self.idle_hp_pause_timer.shutdown()
			self.idle_hp_pause_timer = None
			rospy.logdebug("CANCELLED self.idle_hp_pause_timer")

		if self.idle_hp_resume_timer != None:
			self.idle_hp_resume_timer.shutdown()
			self.idle_hp_resume_timer = None
			rospy.logdebug("CANCELLED self.idle_hp_resume_timer")
		return

	## Long loop (interruptible) that generates various small changes in
	## head motor position
	def doIdle( self, repeat=100, delta_HP_idle=None, idle_ipt=None, delay_s=0.5 ):
		if idleHeadPan.isIdling( self ):
			return
		else:
			idleHeadPan.__is_idling = True

		if (repeat != None and isinstance(repeat, int) and repeat > 1):
			pass
		else:
			repeat = 100

		if delta_HP_idle == None:
			delta_HP_idle = self.delta_HP_idle

		if idle_ipt == None:
			idle_ipt = self.idle_ipt

		## need time for any outstanding head pan to compelete
		##otherwise, headpan recenters
		rospy.sleep(delay_s)

		idleHeadPan.requestFeedback( self, SC_GET_PP )
		try:
			self.origin_hp = self.makiPP["HP"]
		except TypeError as e_DI:
			rospy.logwarn("[WARNING]: UNABLE TO RUN doIdle(): self.makiPP has not yet been populated... Active connection with maki_arbotix_interface?")
			return
		rospy.loginfo("self.origin_hp=" + str(self.origin_hp))

		## THIS IS MEANT TO BE A LONG LOOP, but not infinite
		for i in range(1,repeat):
			## break loop early
			if (not self.ALIVE or not idleHeadPan.__is_idling):
				i = repeat
				continue

			if idleHeadPan.__is_idling and idleHeadPan.__pause_idling:
				i -= 1	## counter act incrementing
				rospy.sleep(0.5)
				continue

			my_idle_ipt = random.randrange( idle_ipt - self.delta_idle_ipt, idle_ipt + self.delta_idle_ipt, 25 )
			hp_gp = random.randint( self.origin_hp - delta_HP_idle, self.origin_hp + delta_HP_idle )
			_pub_cmd = "HP" + SC_SET_GP + str(hp_gp)
			_pub_cmd += SC_SET_IPT + str(my_idle_ipt)
			_pub_cmd += TERM_CHAR_SEND
			rospy.loginfo( "# " +  str(i) + ": " + _pub_cmd )
			idleHeadPan.pubTo_maki_command( self, _pub_cmd )
			rospy.sleep( float(my_idle_ipt) / 1000.0 )

			## break loop early
			if (not self.ALIVE or not idleHeadPan.__is_idling):
				i = repeat
				continue
			else:
				if (random.randint(0,repeat) % random.randint(2,9) == 0):
					rospy.loginfo("BONUS random sleep")
					rospy.sleep( float(my_idle_ipt) / 1000.0 )


		## CLEANUP... repeat 2x
		for pass_count in range(0,2):
			if not idleHeadPan.verifyPose( self, hp=self.origin_hp, delta_pp=2 ):
				_pub_cmd = "HP" + SC_SET_GP + str(self.origin_hp)
				if (pass_count == 0):
					_pub_cmd += SC_SET_IPT + str(idle_ipt)
				_pub_cmd += TERM_CHAR_SEND
				rospy.loginfo( "CLEANUP doIdle(): " + _pub_cmd )
				idleHeadPan.pubTo_maki_command( self, _pub_cmd )
				if (pass_count == 0):
					rospy.sleep( float(idle_ipt) / 1000.0 )
			else:
				break

		if idleHeadPan.isIdling( self ):
			## ONLY IF for loop DID NOT END EARLY
			## reset behavior for next invocation
			idleHeadPan.abort( self )
		return
