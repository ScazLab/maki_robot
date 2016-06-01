#! /usr/bin/env python

#	RUN AS:	rosrun maki_robot INSPIRE4_controller.py 

# 	NOTE:	To be run in conjunction with master-table.xls

import rospy
import re
from std_msgs.msg import String

import signal
import sys
import string
import random

from maki_robot_common import *
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

from blinking import *
from selective_attention import *
from asleep_awake import *
from look_inspire4_intro import *
from engagement_inspire4 import *
from look_inspire4_interactions import *


## ------------------------------
class INSPIRE4Controller( object ):
	## all instances of this class will share the same value
	## variables private to this class
	SETUP = 0
	PRELUDE = 1
	INTRO = 2
	ENGAGEMENT = 3
	STIMULI = 4
	END = 5
	BREAK_DOWN = 6

	def __init__(self, verbose_debug, ros_pub):

		#self.ALIVE = True
		#self.mTT_INTERRUPT = True
		self.VERBOSE_DEBUG = verbose_debug	## default is False
		#self.SWW_WI = ROS_sleepWhileWaiting_withInterrupt()
		#self.DC_helper = dynamixelConversions()
		self.__is_game_running = False
		self.state = None
		self.previous_state = None

		self.durationHeadTurn = 1.0
		self.durationWatchStimuli = 8.0

		#print ros_pub
		if ros_pub == None:
			self.initROS( self )
			#INSPIRE4Controller.initROS( self )
		else:
			self.ros_pub = ros_pub		## can we pass a ros publisher??? Apparently so!
		INSPIRE4Controller.initROSPub( self )
		#INSPIRE4Controller.initROSSub( self )

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
	## Set up publisher to /maki_macro
	#####################
	def initROSPub( self, latch=False ):
		rospy.logdebug( "setup rostopic publisher to /maki_macro" )
			
		self.ros_pub = rospy.Publisher( "maki_macro", String, queue_size = 26, latch = latch)	## if LATCH==True, any new subscribers will see the most recent message published
	
		return 

	def pubTo_maki_macro( self, commandOut ):
		rospy.logdebug( commandOut )
		if not rospy.is_shutdown():
			self.ros_pub.publish( commandOut )
		return

	#####################
	## Set up subscriber to /inspire_four_pilot_command
	#####################
	def initROSSub( self ):
		rospy.Subscriber( "inspire_four_pilot_command", String, INSPIRE4Controller.parse_pilot_command )
	        rospy.logdebug( "now subscribed to /inspire_four_pilot_command" )
		return


	'''
	def start( self ):
		self.ALIVE = True
		self.mTT_INTERRUPT = False
		return

	#######################
	# stop at a planned break point
	#######################
	def stop(self):
		self.mTT_INTERRUPT = True
		return

	#######################
	# stop immediately
	#######################
	def abort(self):
		self.ALIVE = False
		self.mTT_INTERRUPT = True
		return
	'''


	def setBlinkAndScan( self, blink=False, scan=False, disable_ht=False ):
		rospy.logdebug("setBlinkAndScan(): BEGIN")
		_pub_cmd = ""

		if (isinstance(blink, bool) and blink):
			INSPIRE4Controller.pubTo_maki_macro( self, "spontaneousBlink start" )
		else:
			INSPIRE4Controller.pubTo_maki_macro( self, "spontaneousBlink stop" )

		if (isinstance(scan, bool) and scan):
			INSPIRE4Controller.pubTo_maki_macro( self, "visualScan start" )
		else:
			_pub_cmd = "visualScan stop"
			if not disable_ht:	_pub_cmd += " disable_ht=False"
			INSPIRE4Controller.pubTo_maki_macro( self, _pub_cmd )

		rospy.logdebug("setBlinkAndScan(): END")
		return


	#########################
	##
	## Run this for setup and ending
	##
	#########################
	def doSetup( self ):
		rospy.logdebug("doSetup(): BEGIN")	
		INSPIRE4Controller.setBlinkAndScan( self, blink=False, scan=False )

		## has own invocation to headTiltBaseBehavior.start() and .stop( disable_ht=True )
		INSPIRE4Controller.pubTo_maki_macro( self, "asleep" )

		INSPIRE4Controller.__is_setup_done = True
		rospy.logdebug("doSetup(): END")	
		return


	def transitionToEngagement( self, msg ):
		rospy.logdebug("transitionToEngagement(): BEGIN")
		INSPIRE4Controller.setBlinkAndScan( self, blink=False, scan=False )
		INSPIRE4Controller.pubTo_maki_macro( self, msg )
		## TODO: add timer
		#self.SWW_WI.sleepWhileWaiting(1)	## takes 1 second to turn head
		rospy.sleep(1.0)

		if (self.state == None) or (not isinstance(self.state, int)):
			rospy.logerr("transitionToEngagement(): ERROR: Unknown self.state: " + str(self.state))
			rospy.logwarn("transitionToEngagement(): WARN: Expected transitions from INTRO or STIMULI")
			return

		elif self.state == INSPIRE4Controller.INTRO:
			INSPIRE4Controller.pubTo_maki_macro( self, "intro stop disable_ht=False" )
		elif self.state == INSPIRE4Controller.STIMULI:
			INSPIRE4Controller.pubTo_maki_macro( self, "interaction stop disable_ht=False" )
		else:
			rospy.logwarn("transitionToEngagement(): WARNING: Unexpect transition from self.state: " + str(self.state))
			rospy.logwarn("transitionToEngagement(): WARN: Expected transitions from INTRO or STIMULI")
			return
		self.state = INSPIRE4Controller.ENGAGEMENT
		rospy.logdebug("transitionToEngagement(): END")
		return


	def transitionToStimuli( self ):
		rospy.logdebug("transitionToStimuli(): BEGIN")
		INSPIRE4Controller.setBlinkAndScan( self, blink=False, scan=False )
		if self.__is_game_running:
			INSPIRE4Controller.pubTo_maki_macro( self, "startleGame stop disable_ht=False" )
			self.__is_game_running = False
		INSPIRE4Controller.pubTo_maki_macro( self, "interaction start" )
		self.state = INSPIRE4Controller.STIMULI
		rospy.logdebug("transitionToStimuli(): END")
		return


	## durations in seconds
	def setAutoTransitionWatchStimuli( self, durationTurnToScreen=1.0, durationWatchStimuli=8.0 ):
	#def setAutoTransitionWatchStimuli( self, durationTurnToScreen=INSPIRE4Controller.durationHeadTurn, durationWatchStimuli=INSPIRE4Controller.durationWatchStimuli):
		rospy.loginfo("setAutoTransitionFromStimuli(): BEGIN")
		self.start_watch_timer = rospy.Timer(rospy.Duration(durationTurnToScreen), self.startWatchStimuli_callback, oneshot=True)
		rospy.loginfo("CREATED self.start_watch_timer")
		self.stop_watch_timer = rospy.Timer(rospy.Duration(durationTurnToScreen + durationWatchStimuli), self.stopWatchStimuli_callback, oneshot=True)
		rospy.loginfo("CREATED self.stop_watch_timer")
		rospy.loginfo("setAutoTransitionFromStimuli(): END")
		return


	def startWatchStimuli_callback( self, event ):
		rospy.logdebug("startWatchStimuli(): BEGIN")
		_start_time = rospy.get_time()
		rospy.loginfo("startWatchStimuli_callback() called at " + str( event.current_real))
		#INSPIRE4Controller.setBlinkAndScan( self, blink=True, scan=True )
		INSPIRE4Controller.setBlinkAndScan( self, blink=True, scan=False )
		_elapsed_duration = rospy.get_time() - _start_time
		while (_elapsed_duration < 8.0):
			_elapsed_duration = rospy.get_time() - _start_time
			rospy.logdebug("startWatchStimuli_callback(): watching stimuli; ELAPSED DURATION: " + str(_elapsed_duration) + " seconds")
			rospy.sleep(1)	## sleep for 1 second
		rospy.logdebug("startWatchStimuli(): END")
		return


	def stopWatchStimuli_callback( self, event ):
		rospy.logdebug("stopWatchStimuli(): BEGIN")
		rospy.loginfo("stopWatchStimuli_callback() called at " + str( event.current_real))
		_ros_pub = rospy.Publisher( "inspire_four_pilot_command", String, queue_size = 10)
		_ros_pub.publish( "turnToInfant" )
		rospy.logdebug("stopWatchStimuli(): END")
		return

	def parse_pilot_command( self, msg ):
		rospy.logdebug("parse_pilot_command(): BEGIN")
		#rospy.logdebug("received: " + str(msg))
		rospy.loginfo("received: " + str(msg))

		self.previous_state = self.state	## for later comparison
		_unknown_flag = False
		_data = str(msg.data)
		rospy.loginfo("_data = " + _data)
	
		if _data == "setup":
			INSPIRE4Controller.doSetup( self )
			self.state = INSPIRE4Controller.SETUP

		elif _data.startswith( "sync" ):
			if _data.endswith( "Tobii calibration start" ):
				if not INSPIRE4Controller.__is_setup_done:
					INSPIRE4Controller.doSetup( self )
			elif _data.endswith( "Tobii calibration done" ):
				pass
			elif _data.endswith( "visual clap" ):
				pass
			elif _data.endswith( "Tobii verify left screen" ):
				pass
			elif _data.endswith( "Tobii verify maki" ):
				## enable head tilt and get ready for MAKI's introduction
				INSPIRE4Controller.pubTo_maki_macro( self, "intro start" )
			elif _data.endswith( "Tobii verify right screen" ):
				pass
			else:
				_unknown_flag = True

			if not _unknown_flag:
				self.state = INSPIRE4Controller.PRELUDE

		elif _data.startswith( "awake" ):
			rospy.loginfo("prefix = awake; forward the message contents to /maki_macro: " + _data)
			## forward the message contents
			INSPIRE4Controller.pubTo_maki_macro( self, _data )
			self.state = INSPIRE4Controller.INTRO

		elif _data.startswith( "intro" ):
			if _data.endswith( "greet" ):
				rospy.logdebug("\tgreet")
				pass
			elif _data.endswith( "startle" ):
				rospy.logdebug("\tstartle")
				INSPIRE4Controller.setBlinkAndScan( self, scan=True )
			elif "lookAt" in _data:
				INSPIRE4Controller.setBlinkAndScan( self, scan=False )
				## TODO: Set a timer to enable blink and scan
				if "Infant" in _data:
					#rospy.logdebug("\tInfant")
					rospy.logwarn("\tInfant")
					INSPIRE4Controller.transitionToEngagement( self, _data )
					return
				else:
					rospy.logdebug( _data )
			else:
				rospy.logdebug("prefix = intro, SUFFIX IS UNKNOWN: " + _data)
				_unknown_flag = True

			if not _unknown_flag:
				rospy.loginfo("prefix = intro; forward the message contents to /maki_macro: " + _data)
				## forward the message contents
				INSPIRE4Controller.pubTo_maki_macro( self, _data )
				self.state = INSPIRE4Controller.INTRO
			
		elif (_data.startswith( "startleGame stop" )) or (_data == "infant fixation"):
			INSPIRE4Controller.transitionToStimuli( self )

		elif ("tartle" in _data):
			if _data == "startleGame start":	self.__is_game_running = True
			rospy.loginfo("'tartle' in _data; forward the message contents to /maki_macro: " + _data)
			## forward the message contents
			INSPIRE4Controller.pubTo_maki_macro( self, _data )
			self.state = INSPIRE4Controller.ENGAGEMENT

		elif ("turnToScreen" in _data):
			rospy.loginfo("'turnToScreen' in _data; forward the message contents to /maki_macro: " + _data)
			## forward the message contents
			INSPIRE4Controller.pubTo_maki_macro( self, _data )
			self.state = INSPIRE4Controller.STIMULI
			## TODO: add timer to trigger 'watch stimuli' behavior
			#INSPIRE4Controller.setAutoTransitionWatchStimuli( self )
			## TODO: Set a timer to enable blink and scan

		elif _data == "turnToInfant":
			rospy.logwarn("====> turnToInfant")
			INSPIRE4Controller.transitionToEngagement( self, _data )

		elif (_data == "outro start") or (_data == "ending start"):
			INSPIRE4Controller.doSetup( self )
			self.state = INSPIRE4Controller.END

		## TODO: Add logic for participant ID

		## TODO: Add logic for managing start/stop log recording

		else:
			_unknown_flag = True


		if _unknown_flag:	
			rospy.logwarn( "UNKNOWN pilot command: " + str(_data) + "; REMAINS in self.state " + str(self.state) )

		if self.state != self.previous_state:
			rospy.loginfo("UPDATE self.state from previous " + str(self.previous_state) + " to " + str(self.state) + " current")

		rospy.logdebug("parse_pilot_command(): END")
		return




## ------------------------------
if __name__ == '__main__':
        print "__main__: BEGIN"
	controller = INSPIRE4Controller( True, None )

	rospy.Subscriber( "/inspire_four_pilot_command", String, controller.parse_pilot_command )
        rospy.logdebug( "now subscribed to /inspire_four_pilot_command" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


