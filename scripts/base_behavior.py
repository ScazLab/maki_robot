#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import sys
import string

from timeit import default_timer as timer	## wall clock. Unix 1/100 second granularity
import re		# see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python


from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions

from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterupt


########################
## All behavior macros will use this as base class
########################
class baseBehavior(object):
	## all instances of this class share the same value
	## variables private to this class
	__maki_msg_format = None


	def __init__(self, verbose_debug, ros_pub):
		self.ALIVE = True
		self.mTT_INTERUPT = True
		self.VERBOSE_DEBUG = verbose_debug	## default is False
		self.SWW_WI = ROS_sleepWhileWaiting_withInterupt()
		if ros_pub == None:
			self.initROS( self )
		else:
			self.ros_pub = ros_pub		## can we pass a ros publisher??? Apparently so!
		if baseBehavior.__maki_msg_format == None:
			baseBehavior.initPubMAKIFormat( self )
		#rospy.logdebug( str(baseBehavior.__maki_msg_format) )
		self.initPubMAKI()
		self.makiPP = None

	def start(self, makiPP):
		self.ALIVE = True
		self.mTT_INTERUPT = False
		self.makiPP = makiPP

	#######################
	# stop at a planned break point
	#######################
	def stop(self):
		self.mTT_INTERUPT = True

	#######################
	# stop immediately
	#######################
	def abort(self):
		self.ALIVE = False
		self.mTT_INTERUPT = True

	def update( self, makiPP ):
		self.makiPP = makiPP


	#####################
	## THESE ARE COMMON FOR ALL BEHAVIORS
	#####################
	def pubTo_maki_command( self, commandOut ):
		_pub_flag = False

		## make sure that commandOut ends in only one TERM_CHAR_SEND
		_tmp = re.search( baseBehavior.__maki_msg_format, commandOut )
		if _tmp != None:
			## Yes, commandOut ends in only one TERM_CHAR_SEND
			_pub_flag = True
			#if self.VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " matched maki_msg_format" )
		elif (commandOut == "reset"):
			## special case handled by MAKI-Arbotix-Interface.py driver
			_pub_flag = True
		elif not commandOut.endswith( str(TERM_CHAR_SEND) ):
			## append the missing TERM_CHAR_SEND
			commandOut += str(TERM_CHAR_SEND)
			_pub_flag = True
			if self.VERBOSE_DEBUG:       rospy.logdebug( str(commandOut) + " added TERM_CHAR_SEND" )
		else:
			rospy.logerr( "Incorrect message format" + str(commandOut) )

		if self.VERBOSE_DEBUG: rospy.logdebug( str(commandOut) )

		if _pub_flag and not rospy.is_shutdown():
			self.ros_pub.publish( commandOut )


	#####################
	## Initialize ROS node 
	#####################
	def initROS( self, nodename="anon" ):
		## get function name for logging purposes
		_fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
		print str(_fname) + ": BEGIN"	## THIS IS BEFORE ROSNODE INIT

		_anon_rosnode = False
		if nodename == "anon":	_anon_rosnode = True

        	# see http://wiki.ros.org/rospy/Overview/Logging
        	if self.VERBOSE_DEBUG:
			## TODO: FIX
			## str(nodename) = /<__main__.headNod object at 0x7f24b3e07150>
        	        #self.ros_pub = rospy.init_node(str(nodename), anonymous=_anon_rosnode, log_level=rospy.DEBUG)
        	        self.ros_pub = rospy.init_node("anon", anonymous=True, log_level=rospy.DEBUG)
			rospy.logdebug("log_level=rospy.DEBUG")
        	else:
        	        self.ros_pub = rospy.init_node(nodename, anonymous=_anon_rosnode)       ## defaults to log_level=rospy.INFO
		rospy.logdebug("anonymous=" + str(_anon_rosnode))

		rospy.loginfo( str(_fname) + ": END")
		return

	#####################
	## Set up publisher to /maki_command
	#####################
	def initPubMAKI(self):
		## get function name for logging purposes
		_fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
		rospy.logdebug( str(_fname) + ": BEGIN")

		# Setup publisher
		self.ros_pub = rospy.Publisher("maki_command", String, queue_size = 10)

		rospy.loginfo( str(_fname) + ": END")
		return

	#####################
	## Set up regex format for publishing on /maki_command
	#####################
	def initPubMAKIFormat(self):
		## get function name for logging purposes
		_fname = sys._getframe().f_code.co_name        ## see http://stackoverflow.com/questions/5067604/determine-function-name-from-within-that-function-without-using-tracebacko
		rospy.logdebug( str(_fname) + ": BEGIN")

		## make sure that commandOut ends in only one TERM_CHAR_SEND
		baseBehavior.__maki_msg_format = "\A[a-yA-Y]+[a-yA-Y0-9]*"
		baseBehavior.__maki_msg_format += str(TERM_CHAR_SEND)
		baseBehavior.__maki_msg_format += "{1}$"

		rospy.loginfo( str(_fname) + ": END")
		return



########################
## All behavior macros involving head tilt (HT) will use this as base class
########################
class headTiltBaseBehavior(baseBehavior):
	## all instances of this class share the same value
	## variables private to this class
	__ht_enabled = None
	__ht_enable_cmd = None
	__ht_disable_cmd = None


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		baseBehavior.__init__( self, verbose_debug, ros_pub )
		if headTiltBaseBehavior.__ht_enabled == None:
			headTiltBaseBehavior.__ht_enabled = False
		if headTiltBaseBehavior.__ht_enable_cmd == None:
			headTiltBaseBehavior.__ht_enable_cmd = "HT" + str(SC_SET_TL) + str(ht_tl_enable) + str(TERM_CHAR_SEND)
		if headTiltBaseBehavior.__ht_disable_cmd == None:
			headTiltBaseBehavior.__ht_disable_cmd = "HT" + str(SC_SET_TL) + str(ht_tl_disable) + str(TERM_CHAR_SEND)
	def start( self, makiPP ):
		baseBehavior.start( self, makiPP )
		self.enableHT()

	def stop( self ):
		print "headTiltBaseBehavior: stop()"
		baseBehavior.stop( self )
		self.disableHT()
		print "headTiltBaseBehavior: stop() -- END"

	def enableHT( self ):
		if headTiltBaseBehavior.__ht_enabled == True:
			## already enabled
			return

		if (self.makiPP["HT"] <= HT_UP and self.makiPP["HT"] >= HT_DOWN):
			_pub_cmd_GP = "HT" + str(SC_SET_GP) + str(self.makiPP["HT"]) + str(TERM_CHAR_SEND)
			baseBehavior.pubTo_maki_command( self, str(_pub_cmd_GP) )
			self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		
		baseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd) )
		self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		headTiltBaseBehavior.__ht_enabled = True

	def disableHT( self ):
		if headTiltBaseBehavior.__ht_enabled == False:
			## already disabled
			return

		baseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_disable_cmd) )
		self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		headTiltBaseBehavior.__ht_enabled = False

	## TODO: bug fix -- need to replicate bug first!
	def reset( self ):

		if (self.makiPP["HT"] <= HT_UP and self.makiPP["HT"] >= HT_DOWN):
			_pub_cmd_GP = "HT" + str(SC_SET_GP) + str(self.makiPP["HT"]) + str(TERM_CHAR_SEND)
			baseBehavior.pubTo_maki_command( self, str(_pub_cmd_GP) )
			self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		
		## send enable command
		baseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd) )
		self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
		headTiltBaseBehavior.__ht_enabled = True

		## publish "reset" to /maki_command
		baseBehavior.pubTo_maki_command( self, "reset" )
		self.SWW_WI.sleepWhileWaitingMS( 1000, 0.05 )	## make sure command propogates and reset occurs

