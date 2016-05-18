#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import sys
import string

import re		# see http://stackoverflow.com/questions/5749195/how-can-i-split-and-parse-a-string-in-python


from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions

from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt


########################
## All behavior macros will use this as base class
########################
class baseBehavior(object):
	## all instances of this class share the same value
	## variables private to this class
	__maki_cmd_msg_format = None
	__maki_feedback_format = None


	def __init__(self, verbose_debug, ros_pub):
		self.count_movements = 0
		self.ALIVE = True
		self.mTT_INTERRUPT = True
		self.VERBOSE_DEBUG = verbose_debug	## default is False
		self.SWW_WI = ROS_sleepWhileWaiting_withInterrupt()
		self.DC_helper = dynamixelConversions()

		## Does Maki-ro remain in end position (shift=True)
		## or revert to ground position (shift=False)
		self.shift = False	## default is False

		#print ros_pub
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
		self.maki_feedback_values = {}	## empty dictionary

	def start(self, makiPP=None):
		_invalid_entry = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		## check to see if there is an entry with key "PP"
		while (not rospy.is_shutdown() and self.mTT_INTERRUPT):
			## if we were passed a valid makiPP 
			if (makiPP != None) and (makiPP != _invalid_entry):
				self.makiPP = makiPP
				break	## break the while loop

			## if we got a message on /maki_feedback_pres_pos
			if (self.makiPP != None) and (self.makiPP != _invalid_entry):
				break	## break the while loop
			else:
				rospy.loginfo("Waiting for a message on /maki_feedback_pres_pos...")
				## request a feedback message
				baseBehavior.requestFeedback( self, str(SC_GET_PP) )

			self.SWW_WI.sleepWhileWaiting( 1 )	## 1 second

		self.ALIVE = True
		self.mTT_INTERRUPT = False
		return

	#######################
	# stop at a planned break point
	#######################
	def stop(self):
		self.mTT_INTERRUPT = True

	#######################
	# stop immediately
	#######################
	def abort(self):
		self.ALIVE = False
		self.mTT_INTERRUPT = True

	def update( self, makiPP ):
		self.makiPP = makiPP

	def requestFeedback( self, feedback_type, cmd_prop=True, time_ms=100, time_inc=0.05 ):
		## check /maki_feedback_*
		## request a feedback message
		baseBehavior.pubTo_maki_command( self, str(SC_FEEDBACK) + str(feedback_type) + str(TERM_CHAR_SEND), cmd_prop, time_ms, time_inc )

	#####################
	## THESE ARE COMMON FOR ALL BEHAVIORS
	#####################
	def pubTo_maki_command( self, commandOut, cmd_prop=True, time_ms=100, time_inc=0.05 ):
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
			if (cmd_prop):	self.SWW_WI.sleepWhileWaitingMS( time_ms, time_inc, end_early=False )	## make sure command propogates
		return


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
				#print nodename
			else:
				## Actually... this is already an initialized rosnode, so quick, exit!
				self.ros_pub = nodename
				#print nodename
				return

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
		return baseBehavior.__maki_feedback_format

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

		if (len(self.maki_feedback_values) == 0) or not ( str(_prefix) in self.maki_feedback_values ):
			## if no _prefix entry exists in the dictionary, add new
			self.maki_feedback_values[ str(_prefix) ] = _tmp_dict
			rospy.loginfo( "New entry added to self.maki_feedback_values" + str(recv_msg) )

		elif not (_tmp_dict == self.maki_feedback_values[ str(_prefix) ]):
			## if _prefix entry exists, update
			self.maki_feedback_values[ str(_prefix) ].update( _tmp_dict )
			#rospy.loginfo( "Updated entry in self.maki_feedback_values" + str(recv_msg) )
			rospy.logdebug( "Updated entry in self.maki_feedback_values" + str(recv_msg) )
		else:
			pass

		## update
		if str(_prefix) == str(SC_GET_PP):
			self.makiPP = _tmp_dict

		#print "parseMAKIFeedbackMsg: END"
		return

	#####################
	##
	## This is blocking
	##
	## Raises rospy.exceptions.ROSException when breaks on stall
	##
	## TODO:
	## *  Estimate duration based on the largest difference
	##	between current and goal positions
	#####################
	def monitorMoveToGP( self, gp_cmd, hp_gp=None, ht_gp=None, ll_gp=None, lr_gp=None, ep_gp=None, et_gp=None, delta_pp=DELTA_PP, cmd_prop=True ):
		### SEND THE COMMAND
		baseBehavior.pubTo_maki_command( self, gp_cmd, cmd_prop=cmd_prop )

		_moving_flag = dict( zip(F_VAL_SEQ, [None]*len(F_VAL_SEQ)) )
		_count_moving_flags = 0
		if (hp_gp != None) and (hp_gp != INVALID_INT):	
			_moving_flag["HP"] = True
			_count_moving_flags = _count_moving_flags +1
		if (ht_gp != None) and (ht_gp != INVALID_INT):	
			_moving_flag["HT"] = True
			_count_moving_flags = _count_moving_flags +1
		if (ll_gp != None) and (ll_gp != INVALID_INT):	
			_moving_flag["LL"] = True
			_count_moving_flags = _count_moving_flags +1
		if (lr_gp != None) and (lr_gp != INVALID_INT):	
			_moving_flag["LR"] = True
			_count_moving_flags = _count_moving_flags +1
		if (ep_gp != None) and (ep_gp != INVALID_INT):	
			_moving_flag["EP"] = True
			_count_moving_flags = _count_moving_flags +1
		if (et_gp != None) and (et_gp != INVALID_INT):	
			_moving_flag["ET"] = True
			_count_moving_flags = _count_moving_flags +1

		if _count_moving_flags == 0:	return

		self.count_movements = self.count_movements +1

		_stall_count = 0
		_old_makiPP = self.makiPP
		_start_time = rospy.get_time()
		### SEND THE COMMAND
		#baseBehavior.pubTo_maki_command( self, gp_cmd )
		while not rospy.is_shutdown():
			## There is an implicit sleep in requestFeedback of 100ms (default)
			baseBehavior.requestFeedback( self, SC_GET_PP ) 

			if _moving_flag["HP"] and (abs(self.makiPP["HP"] - hp_gp) < delta_pp):
				rospy.logdebug("HP done moving")
				_moving_flag["HP"] = False
				_count_moving_flags = _count_moving_flags -1

			if _moving_flag["HT"] and (abs(self.makiPP["HT"] - ht_gp) < delta_pp):
				rospy.logdebug("HT done moving")
				_moving_flag["HT"] = False
				_count_moving_flags = _count_moving_flags -1

			if _moving_flag["EP"] and (abs(self.makiPP["EP"] - ep_gp) < delta_pp):
				rospy.logdebug("EP done moving")
				_moving_flag["EP"] = False
				_count_moving_flags = _count_moving_flags -1

			if _moving_flag["ET"] and (abs(self.makiPP["ET"] - et_gp) < delta_pp):
				rospy.logdebug("ET done moving")
				_moving_flag["ET"] = False
				_count_moving_flags = _count_moving_flags -1

			if _moving_flag["LL"] and (abs(self.makiPP["LL"] - ll_gp) < delta_pp):
				rospy.logdebug("LL done moving")
				_moving_flag["LL"] = False
				_count_moving_flags = _count_moving_flags -1

			if _moving_flag["LR"] and (abs(self.makiPP["LR"] - lr_gp) < delta_pp):
				rospy.logdebug("LR done moving")
				_moving_flag["LR"] = False
				_count_moving_flags = _count_moving_flags -1

			if _count_moving_flags == 0:	break

			if (_old_makiPP == self.makiPP):
				_stall_count = _stall_count + 1
				rospy.logdebug("... _stall_count = " + str(_stall_count) )
				baseBehavior.requestFeedback( self, SC_GET_PP ) 
			if (_stall_count == 10):	
				#rospy.logerr("STALLED!!!")
				raise rospy.exceptions.ROSException("STALLED!!!! self.makiPP hasn't changed... is the motor at its limit?")
				break
			_old_makiPP = self.makiPP
		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo("**** DONE! movement took " + str(_duration) + " seconds")
		return


########################
## All behavior macros involving head pan (HP) will use this as base class
########################
class headPanBaseBehavior(baseBehavior):
	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		baseBehavior.__init__( self, verbose_debug, ros_pub )
		## now custom things
		self.prefix_hpgp = "HP" + str(SC_SET_GP)
		self.prefix_epgp = "EP" + str(SC_SET_GP)
		self.hpgp_regex = self.prefix_hpgp + "([0-9]+)"
		self.aCFG_coeff = 0.36		## average of 0.21 and 0.53 from Todorvic 2009

	## Automatically fill in commandOut to adjust eye pan position based on head pan position
	##	if eye pan position is missing
	def pubTo_maki_command( self, commandOut, fixed_gaze=True, cmd_prop=True, time_ms=100, time_inc=0.05 ):
		rospy.logdebug("headPanBaseBehavior.pubTo_maki_command(): BEGIN")
		if fixed_gaze:	commandOut = headPanBaseBehavior.amend_maki_command( self, commandOut, fixed_gaze=fixed_gaze )
		return baseBehavior.pubTo_maki_command( self, commandOut, cmd_prop=cmd_prop, time_ms=time_ms, time_inc=time_inc )

	## Amend to adjust for fixed gaze
	def monitorMoveToGP( self, gp_cmd, fixed_gaze=True, hp_gp=None, ht_gp=None, ll_gp=None, lr_gp=None, ep_gp=None, et_gp=None, delta_pp=DELTA_PP, cmd_prop=True ):
		rospy.logdebug("headPanBaseBehavior.monitorMoveToGP(): BEGIN")
		if fixed_gaze:	gp_cmd = headPanBaseBehavior.amend_maki_command( self, gp_cmd, fixed_gaze=fixed_gaze )
		return baseBehavior.monitorMoveToGP( self, gp_cmd, hp_gp=hp_gp, ht_gp=ht_gp, ll_gp=ll_gp, lr_gp=lr_gp, ep_gp=ep_gp, et_gp=et_gp, delta_pp=delta_pp, cmd_prop=cmd_prop )


	def amend_maki_command( self, commandOut, fixed_gaze=True ):
		if fixed_gaze and (self.prefix_hpgp in commandOut) and not (self.prefix_epgp in commandOut):
			rospy.logdebug("Need to adjust commandOut...")
			## get HPGP value
			_tmp = re.search( self.hpgp_regex, commandOut )
			if _tmp != None:
				_hp_gp = int( _tmp.group(1) )
			else:
				return commandOut

			## calculate EPGP value
			_ep_gp = EP_FRONT	## default
			_ep_gp = headPanBaseBehavior.autoCalculateFixedGazeFromHPGP( self, _hp_gp )

			## update commandOut
			commandOut = self.prefix_epgp + str(_ep_gp) + commandOut	
			rospy.loginfo("headPanBaseBehavior.pubTo_maki_command(): Update commandOut to: " + str(commandOut))
		return commandOut

	## Todorovic 2009: "In order to maintain the perceptions of fixed gaze, every 1% shift of
	##	the facial features from centered, corresponded to an iris shift in the lid
	##	apertures of 0.21-0.53% depending on testing method"
	def autoCalculateFixedGazeFromHPGP( self, hp_gp ):
		rospy.logdebug("autoCalculateFixedGazeFromHPGP(): BEGIN")

		_hp_gp_degrees = self.DC_helper.convertToDegrees_ticks( abs(hp_gp - HP_FRONT) ) 
		_ep_gp_degrees = _hp_gp_degrees * self.aCFG_coeff
		ret = self.DC_helper.convertToTicks_degrees( _ep_gp_degrees )
		if (hp_gp < HP_FRONT):
			ret = EP_FRONT - ret
		else:
			ret = EP_FRONT + ret
		rospy.logdebug("given hp_gp=" + str(hp_gp) + ", computed ep_gp=" + str(ret))
		rospy.logdebug("autoCalculateFixedGazeFromHPGP(): END")
		return ret

########################
## All behavior macros involving head tilt (HT) will use this as base class
########################
class headTiltBaseBehavior(baseBehavior):
	## all instances of this class share the same value
	## variables private to this class
	__ht_enabled = None
	__ht_enable_cmd = None
	__ht_disable_cmd = None

	__maki_feedback_format = None

	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		baseBehavior.__init__( self, verbose_debug, ros_pub )
		if (headTiltBaseBehavior.__maki_feedback_format == None):
			headTiltBaseBehavior.__maki_feedback_format = baseBehavior.initSubMAKIFormat( self )

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


	def start( self, makiPP=None, enable_ht=True ):
		## need to call base class' start function first!!!!
		baseBehavior.start( self, makiPP )

		## check to see if there is an entry with key "TL"
		while (not rospy.is_shutdown()):
			## if we got a message on /maki_feedback_torque_limit
			if ( str(SC_GET_TL) in self.maki_feedback_values ):
				break	## break the while loop
			else:
				rospy.loginfo("Waiting for a message on /maki_feedback_torque_limit...")
				## request a feedback message
				headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )

			#self.SWW_WI.sleepWhileWaiting( 1 )	## 1 second

		if enable_ht:	self.enableHT()

	def stop( self ):
		rospy.logdebug( "headTiltBaseBehavior: stop()" )
		## call base behavior first
		baseBehavior.stop( self )
		self.disableHT()
		rospy.logdebug( "headTiltBaseBehavior: stop() -- END" )

	def isHTEnabled( self ):
		return headTiltBaseBehavior.__ht_enabled

	def enableHT( self ):
		if headTiltBaseBehavior.__ht_enabled == True:
			## already enabled
			## keep enabled; head tilt servo monitor might be running, so reset timer
			headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd), cmd_prop=False )
			return

		## typical result
		## enableHT duration: 0.589332818985 seconds
		## MIN: 0.532744884491 seconds
		_enableHT_start_time = rospy.get_time()

		## re-init values
		self.makiPP = None

		## wait here until feedback is published to rostopics
		_loop_count = 0
		while (self.makiPP == None) and (not rospy.is_shutdown()):
			if (_loop_count == 0):
				## request current servo motor values
				headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )
			elif (_loop_count == 3):
				headTiltBaseBehavior.requestFeedback( self, str(SC_GET_PP) )
			else:
				self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
			_loop_count = (1 + _loop_count) % 20
			rospy.logdebug( "enable, 1st while: " + str(_loop_count) )

		if (self.maki_feedback_values[ str(SC_GET_TL) ]["HT"] == ht_tl_enable):
			## keep enabled; head tilt servo monitor might be running, so reset timer
			headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd), cmd_prop=False )
			## set flag to reflect servo status
			headTiltBaseBehavior.__ht_enabled = True
			return

		if (self.makiPP["HT"] <= HT_UP and self.makiPP["HT"] >= HT_DOWN):
			_pub_cmd_GP = "HT" + str(SC_SET_GP) + str(self.makiPP["HT"]) + str(TERM_CHAR_SEND)
			headTiltBaseBehavior.pubTo_maki_command( self, str(_pub_cmd_GP) )
		
		## wait here until feedback is published to rostopics
		_loop_count = 0
		#while (self.maki_feedback_values[ str(SC_GET_TL) ]["HT"] != ht_tl_enable):
		while (not rospy.is_shutdown()) and (not headTiltBaseBehavior.__ht_enabled):	## value set in parseMAKIFeedbackMsg
			if (_loop_count == 0):
				headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd) )

			## every pass through,
			## request current servo motor values
			headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )

			_loop_count = (1 + _loop_count) % 10
			rospy.logdebug( "enable, 2nd while: " + str(_loop_count) )

		## we only get here if the head tilt enable command has been sent, executed,
		## and status reflected in the servo motor status
		headTiltBaseBehavior.__ht_enabled = True

		rospy.logerr( "enableHT duration: " + str( rospy.get_time() - _enableHT_start_time ) + " seconds" )
		return

	def disableHT( self ):
		if headTiltBaseBehavior.__ht_enabled == False:
			## already disabled
			return

		## typical result:
		## disableHT duration: 0.151942968369 s
		## MAX: 0.203392982483 s
		_disableHT_start_time = rospy.get_time()

		_loop_count = 0
		#while (self.maki_feedback_values[ str(SC_GET_TL) ]["HT"] != ht_tl_disable):
		while (not rospy.is_shutdown()) and headTiltBaseBehavior.__ht_enabled:	## value set in parseMAKIFeedbackMsg
			if (_loop_count % 10) == 0:
				headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_disable_cmd) )
				headTiltBaseBehavior.requestFeedback( self, str(SC_GET_TL) )
			else:
				self.SWW_WI.sleepWhileWaitingMS( 100, 0.05 )	## make sure command propogates
			_loop_count = 1 + _loop_count

		headTiltBaseBehavior.__ht_enabled = False

		rospy.logerr(" disableHT duration: " + str( rospy.get_time() - _disableHT_start_time ) + " seconds" )
		return

	## TODO: bug fix -- need to replicate bug first!
	def reset( self ):

		if (self.makiPP["HT"] <= HT_UP and self.makiPP["HT"] >= HT_DOWN):
			_pub_cmd_GP = "HT" + str(SC_SET_GP) + str(self.makiPP["HT"]) + str(TERM_CHAR_SEND)
			headTiltBaseBehavior.pubTo_maki_command( self, str(_pub_cmd_GP) )
		
		## send enable command
		headTiltBaseBehavior.pubTo_maki_command( self, str(headTiltBaseBehavior.__ht_enable_cmd) )
		headTiltBaseBehavior.__ht_enabled = True

		## publish "reset" to /maki_command
		headTiltBaseBehavior.pubTo_maki_command( self, "reset" )

	def parseMAKIFeedbackMsg ( self, recv_msg ):
		_tmp = headTiltBaseBehavior.__maki_feedback_format.search( recv_msg.data )
		if _tmp != None:
			_prefix = _tmp.group(1)
			if str(_prefix) == str(SC_GET_TL):

				_feedback_values = _tmp.group(2)
				#print "Validated: prefix='" + _prefix + "' and feedback_values='" + _feedback_values + "'"

				_values = re.findall("([0-9]+)", _feedback_values)	## this is a list of strings
				## need to conver to int (see http://stackoverflow.com/questions/22672598/converting-lists-of-digits-stored-as-strings-into-integers-python-2-7) 
				_tmp_dict = dict( zip(F_VAL_SEQ, map(int, _values)) )

				## update
				_ht_tl_val = _tmp_dict["HT"]
				if (_ht_tl_val == ht_tl_disable):
					if (headTiltBaseBehavior.__ht_enabled == True):
						headTiltBaseBehavior.__ht_enabled = False
						rospy.loginfo("TL feedback received; disable headTiltBaseBehavior.__ht_enabled flag: " + str(headTiltBaseBehavior.__ht_enabled))
				elif (_ht_tl_val == ht_tl_enable):
					if (headTiltBaseBehavior.__ht_enabled == False):
						headTiltBaseBehavior.__ht_enabled = True
						rospy.loginfo("TL feedback received; enable headTiltBaseBehavior.__ht_enabled flag: " + str(headTiltBaseBehavior.__ht_enabled))
				else:
					rospy.logerr( "Invalid head tilt torque limit: " + str(_ht_tl_val) )


		## call base class
		baseBehavior.parseMAKIFeedbackMsg( self, recv_msg )
		return


########################
## All behavior macros involving eyelids (LL) will use this as base class
##
## TODO: Incorporate LR even though doesn't exist in our Maki-ro
########################
class eyelidBaseBehavior( baseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	# none


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		baseBehavior.__init__( self, verbose_debug, ros_pub )
		## add anything else needed by an instance of this subclass

		if self.makiPP == None:
			self.makiPP = dict( zip(F_VAL_SEQ, [ INVALID_INT ] * len(F_VAL_SEQ) ) )

		self.origin_ll = LL_OPEN_DEFAULT		## default is neutral
		self.ll_open = LL_OPEN_DEFAULT
		self.ll_close = LL_CLOSE_MAX
		self.ll_delta_range = abs(self.ll_open - self.ll_close)	## ticks

		self.ALIVE = True
		return

	def setEyelidRange( self, ll, ll_delta=None, ll_close=None):
		if ll_delta == None and ll_close == None:	
			ll_delta = self.ll_delta_range	
		elif ll_delta == None and ll_close != None:
			self.ll_delta_range = abs(ll - ll_close)
			ll_delta = self.ll_delta_range
		else:
			## update with new value
			self.ll_delta_range = ll_delta

		self.ll_close = ll - ll_delta
		self.ll_open = ll	## passed position

		## check range values
		if self.ll_close < LL_CLOSE_MAX:	self.ll_close = LL_CLOSE_MAX
		if self.ll_open > LL_OPEN_MAX:	self.ll_open = LL_OPEN_MAX
		rospy.logdebug("Eyelid range: (" + str(self.ll_close) + ", " + str(self.ll_open) + ")")
		return

	def setEyelidNeutralPose( self, ll, monitor=False, cmd_prop=False):
		self.origin_ll = ll
		_pub_cmd = "LLGP" + str(self.origin_ll) + str(TERM_CHAR_SEND) 

		if monitor:
			try:
				eyelidBaseBehavior.monitorMoveToGP( self, _pub_cmd, ll_gp=self.origin_ll )
			except rospy.exceptions.ROSException as e:
				raise e
		else:
			eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)

	def eyelidClose( self, ll_close=None, monitor=False, cmd_prop=False ):
		_pub_cmd = "LLGP" 
		if ll_close != None:	
			_pub_cmd += str(ll_close) 
		else:
			_pub_cmd += str(self.ll_close) 
		_pub_cmd += str(TERM_CHAR_SEND) 

		if monitor:
			try:
				eyelidBaseBehavior.monitorMoveToGP( self, _pub_cmd, ll_gp=self.ll_close )
			except rospy.exceptions.ROSException as e:
				raise e
		else:
			eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)
		return
		
	def eyelidOpen( self, ll_open=None, monitor=False, cmd_prop=False ):
		_pub_cmd = "LLGP" 
		if ll_open !=None:
			_pub_cmd += str(ll_open)
		else:
			_pub_cmd += str(self.ll_open) 
		_pub_cmd += str(TERM_CHAR_SEND) 

		if monitor:
			try:
				eyelidBaseBehavior.monitorMoveToGP( self, _pub_cmd, ll_gp=self.ll_open )
			except rospy.exceptions.ROSException as e:
				raise e
		else:
			eyelidBaseBehavior.pubTo_maki_command( self, _pub_cmd, cmd_prop=cmd_prop)
		return


########################
##
## All behavior macros involving eyelids (LL) and head tilt (HT)
##	will use this as base class. Inherits from both
##	eyelidBaseBehavior and headTiltBaseBehavior
##
########################
class eyelidHeadTiltBaseBehavior( eyelidBaseBehavior, headTiltBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	# none


	def __init__(self, verbose_debug, ros_pub):
		## call eyelid base class' __init__
		eyelidBaseBehavior.__init__( self, verbose_debug, ros_pub )
		## call headTilt base class' __init__
		headTiltBaseBehavior.__init__( self, verbose_debug, self.ros_pub )
		## add anything else needed by an instance of this subclass

		return

	def start( self, makiPP=None, enable_ht=True ):
		## eyelidHeadTiltBaseBehavior inherits from both
		## eyelidBaseBehavior and headTiltBaseBehavior,
		## and both inherit from baseBehavior.
		## eyelidBaseBehavior.start() defaults to
		## baseBehavior.start(), headTiltBaseBehavior.start()
		## overrides this, so  
		## call headTilt base class' start()
		return headTiltBaseBehavior.start( self, makiPP, enable_ht )

	def stop( self ):
		## eyelidHeadTiltBaseBehavior inherits from both
		## eyelidBaseBehavior and headTiltBaseBehavior,
		## and both inherit from baseBehavior.
		## eyelidBaseBehavior.stop() defaults to
		## baseBehavior.stop(), headTiltBaseBehavior.stop()
		## overrides this, so  
		## call headTilt base class' stop()
		return headTiltBaseBehavior.stop( self )

	def parseMAKIFeedbackMsg ( self, recv_msg ):
		## eyelidHeadTiltBaseBehavior inherits from both
		## eyelidBaseBehavior and headTiltBaseBehavior,
		## and both inherit from baseBehavior.
		## eyelidBaseBehavior.parseMAKIFeedbackMsg() defaults to
		## baseBehavior.parseMAKIFeedbackMsg(), headTiltBaseBehavior.parseMAKIFeedbackMsg()
		## overrides this, so  
		## call headTilt base class' parseMAKIFeedbackMsg()
		return headTiltBaseBehavior.parseMAKIFeedbackMsg( self, recv_msg )



