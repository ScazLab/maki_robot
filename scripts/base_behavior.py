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
	__maki_cmd_msg_format = None
	__maki_feedback_format = None
	__maki_feedback_values = {}	## empty dictionary


	def __init__(self, verbose_debug, ros_pub):
		self.ALIVE = True
		self.mTT_INTERUPT = True
		self.VERBOSE_DEBUG = verbose_debug	## default is False
		self.SWW_WI = ROS_sleepWhileWaiting_withInterupt()
		if ros_pub == None:
			self.initROS( self )
		else:
			self.ros_pub = ros_pub		## can we pass a ros publisher??? Apparently so!
		if baseBehavior.__maki_cmd_msg_format == None:
			baseBehavior.initPubMAKIFormat( self )
			#rospy.logdebug( str(baseBehavior.__maki_cmd_msg_format) )
		if baseBehavior.__maki_feedback_format == None:
			baseBehavior.initSubMAKIFormat( self )
		self.initPubMAKI()
		self.initSubMAKIFeedback()
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
		_tmp = re.search( baseBehavior.__maki_cmd_msg_format, commandOut )
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
		if nodename == "anon":
			_anon_rosnode = True
		else:
			## Parse nodename
			## Examples passed: <__main__.headNod object at 0x7f24b3e07150>
			## <__main__.lookAlissa object at 0xb6c91bac>
			#print str(nodename)
			_tmp_nodename = str(nodename)
			_object_name_format = "^\<\_\_main\_\_\.(\w+)"
			#print _object_name_format
			_tmp = re.search( _object_name_format, _tmp_nodename )
			if _tmp != None:
				nodename = _tmp.group(1)
			print nodename

        	# see http://wiki.ros.org/rospy/Overview/Logging
        	if self.VERBOSE_DEBUG:
        	        self.ros_pub = rospy.init_node(str(nodename), anonymous=_anon_rosnode, log_level=rospy.DEBUG)
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
		baseBehavior.__maki_cmd_msg_format = "\A[a-yA-Y]+[a-yA-Y0-9]*"
		baseBehavior.__maki_cmd_msg_format += str(TERM_CHAR_SEND)
		baseBehavior.__maki_cmd_msg_format += "{1}$"

		rospy.loginfo( str(_fname) + ": END")
		return

	def initSubMAKIFormat( self ):
		## Setup regex template for expected feedback syntax
		_feedback_msg_format = "\A([A-Z]{2})"	## 2 alphabetic char prefix
		_feedback_msg_format += "(([0-9]+" + DELIMITER_RECV + "){" + str(SERVOCOUNT-1) + "}[0-9]+)" 
		_feedback_msg_format += TERM_CHAR_RECV + "\Z"
		#print _feedback_msg_format
		baseBehavior.__maki_feedback_format = re.compile(_feedback_msg_format)
		return

	def initROSSub( self, feedback ):
		#print feedback

		# Setup subscribers
		for _prefix, _sub_params in feedback.iteritems():
			_topic = _sub_params[0]
			_type = _sub_params[1]
			_callback = _sub_params[2]
			rospy.Subscriber( _topic, _type, _callback )
			rospy.logdebug( "now subscribed to " + str(_topic) )
		return

	def initSubMAKIFeedback( self ):
		_maki_feedback_sub = {}		## init as empty dictionary
		_maki_feedback_sub[ str(SC_GET_PP) ] = ("maki_feedback_pres_pos", String, self.parseMAKIFeedbackMsg)
		_maki_feedback_sub[ str(SC_GET_ER) ] = ("maki_feedback_error", String, self.parseMAKIFeedbackMsg)
		baseBehavior.initROSSub( self, _maki_feedback_sub )
		return

	def parseMAKIFeedbackMsg ( self, recv_msg ):
		if self.VERBOSE_DEBUG:
			#rospy.logdebug( "parseMAKIFeedbackMsg: BEGIN" )
			rospy.logdebug( "Received: " + str(recv_msg.data) )

		_tmp = baseBehavior.__maki_feedback_format.search( recv_msg.data )
		if _tmp != None:
			_prefix = _tmp.group(1)
			_feedback_values = _tmp.group(2)
			#print "Validated: prefix='" + _prefix + "' and feedback_values='" + _feedback_values + "'"
		else:
			rospy.logerr( "Received with ERROR! Invalid message format: " + str(recv_msg) )
			return	## return without an expression argument returns None. Falling off the end of a function also returns None

		_values = re.findall("([0-9]+)", _feedback_values)	## this is a list of strings
		## need to conver to int (see http://stackoverflow.com/questions/22672598/converting-lists-of-digits-stored-as-strings-into-integers-python-2-7) 
		_tmp_dict = dict( zip(F_VAL_SEQ, map(int, _values)) )

		if (len(baseBehavior.__maki_feedback_values) == 0) or not ( str(_prefix) in baseBehavior.__maki_feedback_values ):
			## if no _prefix entry exists in the dictionary, add new
			baseBehavior.__maki_feedback_values[ str(_prefix) ] = _tmp_dict
			rospy.loginfo( "New entry added to baseBehavior.__maki_feedback_values" + str(recv_msg) )

		elif not (_tmp_dict == baseBehavior.__maki_feedback_values[ str(_prefix) ]):
			## if _prefix entry exists, update
			baseBehavior.__maki_feedback_values[ str(_prefix) ].update( _tmp_dict )
			rospy.loginfo( "Updated entry in baseBehavior.__maki_feedback_values" + str(recv_msg) )
		else:
			pass

		#print "parseMAKIFeedbackMsg: END"
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

		## subscribe to rostopic maki_feedback_torque_limit and maki_feedback_goal_pos
		_maki_feedback_sub = {}		## init as empty dictionary
		_maki_feedback_sub[ str(SC_GET_TL) ] = ("maki_feedback_torque_limit", String, self.parseMAKIFeedbackMsg)
		_maki_feedback_sub[ str(SC_GET_GP) ] = ("maki_feedback_goal_pos", String, self.parseMAKIFeedbackMsg)
		baseBehavior.initROSSub( self, _maki_feedback_sub )

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


