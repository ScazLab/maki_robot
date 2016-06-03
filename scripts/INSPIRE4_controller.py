#! /usr/bin/env python

#	RUN AS:	rosrun maki_robot INSPIRE4_controller.py 

# 	NOTE:	To be run in conjunction with master-table.xls

import rospy
import re
from std_msgs.msg import String
from std_srvs.srv import Empty

import signal
import sys
import string
import random
import thread

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

	NUMBER_OF_INTERACTIONS = 6

	def __init__(self, verbose_debug, ros_pub):

		#self.ALIVE = True
		#self.mTT_INTERRUPT = True
		self.VERBOSE_DEBUG = verbose_debug	## default is False
		#self.SWW_WI = ROS_sleepWhileWaiting_withInterrupt()
		#self.DC_helper = dynamixelConversions()

		INSPIRE4Controller.resetInteractionCount( self )
		self.__is_setup_done = False
		self.__is_game_running = False

		self.data_logger_status = "unknown"
		self.start_logger_timer = None
		self.stop_logger_timer = None

		self.state = None
		self.previous_state = None

		self.durationHeadTurn = 1.0
		self.durationWatchStimuli = 8.0

		## primarily will publish to /maki_macro
		if ros_pub == None:
			self.initROS( self )
			#INSPIRE4Controller.initROS( self )
		else:
			self.ros_pub = ros_pub		## can we pass a ros publisher??? Apparently so!
		INSPIRE4Controller.initROSPub( self )
		#INSPIRE4Controller.initROSSub( self )

		## secondarily may publish to /maki_internal_monologue when fancied
		self.ros_pub_MIM = rospy.Publisher( "maki_internal_monologues", String, queue_size = 10)
		self.MIM_count = 0
		return


	def resetInteractionCount( self ):
		self.interaction_count = 0

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

	def pubTo_maki_internal_monologue( self, thought ):
		if not rospy.is_shutdown():
			self.MIM_count += 1
			self.ros_pub_MIM.publish( "Thought #" + str(self.MIM_count) + ": " + thought )
		return

	## Occasionally, will need to publish to self
	def pubTo_inspire_four_pilot_command( self, commandOut ):
		rospy.logdebug( ">>>>> SELF PUBLISHING: " + commandOut )
		if not rospy.is_shutdown():
			_ros_pub = rospy.Publisher( "inspire_four_pilot_command", String, queue_size = 10)
			_ros_pub.publish( commandOut )
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

		self.__is_setup_done = True
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

		## KATE LIVE HACKING: BEGIN
		#INSPIRE4Controller.pubTo_maki_macro( self, "intro stop disable_ht=False" )
		#INSPIRE4Controller.pubTo_maki_macro( self, "interaction stop disable_ht=False" )
		## KATE LIVE HACKING: END

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


	## ------------------------------
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
		#_ros_pub = rospy.Publisher( "inspire_four_pilot_command", String, queue_size = 10)
		#_ros_pub.publish( "turnToInfant" )
		INSPIRE4Controller.pubTo_inspire_four_pilot_command( self, "turnToInfant" )
		rospy.logdebug("stopWatchStimuli(): END")
		return


	## ------------------------------
	## based on MAKI-WOz-INSPIRE4.py
	##
	## To toggle recording, pass the opposite status
	##
	## To start recording, pass dl_status as "stopped"
	## To stop recording, pass dl_status as "started"
	def toggleDataLoggerRecording( self, dl_status ):
		rospy.logdebug("toggleDataLoggerRecording(): BEGIN")

		if ( (not dl_status.isalpha()) or (dl_status != "started") and (dl_status != "stopped") ):
			rospy.logwarn( "toggleDataLoggerRecording(): Invalid data_logger_status (" + str( dl_status ) + "); rosnode /data_logger may not be running")
			return

		_service_exception_flag = False

		if (dl_status == "started"):
			## dl_status == "started", so stop /data_logger rosbag recording
			try:
				rospy.wait_for_service('/data_logger/stop', 5)	## timeout is 5 seconds
			except rospy.exceptions.ROSException as _e0:
				rospy.logerr( "Service /data_logger/stop not found: " + str(_e0) )
				rospy.logwarn( "rosnode /data_logger may not be running" )
				self.data_logger_status = "unknown"
				return
			dl_stop = rospy.ServiceProxy('/data_logger/stop', Empty)
			try:
				rospy.logdebug( "calling rosservice /data_logger/stop..." )
				dl_stop_resp = dl_stop()
			except rospy.ServiceException as _e1:
				rospy.logwarn( "Service /data_logger/stop did not process request: " + str(_e1) )
				_service_exception_flag = True
			else:
				rospy.logdebug( "calling rosservice /data_logger/stop... SUCCESS" )

		else:
			## dl_status == "stopped", so start /data_logger rosbag recording
			try:
				rospy.wait_for_service('/data_logger/start', 5) ## timeout = 5 seconds
			except rospy.exceptions.ROSException as _e2:
				rospy.logerr( "Service /data_logger/start not found: " + str(_e2) )
				rospy.logwarn( "rosnode /data_logger may not be running" )
				self.data_logger_status = "unknown"
				return
			dl_start = rospy.ServiceProxy('/data_logger/start', Empty)
			try:
				rospy.logdebug( "calling rosservice /data_logger/start..." )
				dl_start_resp = dl_start()
			except rospy.ServiceException as _e3:
				rospy.logwarn( "Service /data_logger/start did not process request: " + str(_e3) )
				_service_exception_flag = True
			else:
				rospy.logdebug( "calling rosservice /data_logger/start... SUCCESS" )

		if _service_exception_flag:
			if self.data_logger_status == "started":
				rospy.logwarn( "Unable to toggle rosbag recording to ON... Recording remains " + str(self.data_logger_status) )
			elif self.data_logger_status == "stopped":
				rospy.logwarn( "Unable to toggle rosbag recording to OFF... Recording remains " + str(self.data_logger_status) )
			else:
				rospy.logwarn( "Unable to toggle rosbag recording... INVALID STATUS; self.data_logger_status = " + str(self.data_logger_status) )

		rospy.logdebug("toggleDataLoggerRecording(): END")
		return


	## self.data_logger_status is only updated by the value
	##	published to rostopic /data_logger/status 
	##	by calling its start/stop services
	def updateDataLoggerStatus_callback( self, msg ):
		rospy.logdebug( "updateDataLoggerStatus_callback(): BEGIN" )
		_data = str( msg.data )
		if ((not _data.isalpha) or ((_data != "started") and (_data != "stopped"))):
			rospy.logwarn( "updateDataLoggerStatus_callback(): INVALID INPUT: expected string 'started' or 'stopped'; received " + str( _data ) )
			rospy.logdebug( "updateDataLoggerStatus_callback(): self.data_logger_status remains " + str(self.data_logger_status) )
			return

		if (_data != self.data_logger_status):
			rospy.logdebug( "Updating self.data_logger_status to " + str(_data) + " (from " + str(self.data_logger_status) + ")" )
			self.data_logger_status = _data

			if self.data_logger_status == "started":
				rospy.loginfo( "Toggle rosbag recording to ON" )
			else:
				rospy.loginfo( "Toggle rosbag recording to OFF" )

		rospy.logdebug( "updateDataLoggerStatus_callback(): END" )
		return


	## durations in seconds
	def setAutoDataLogging( self, durationToAutoOff=None, durationToAutoOn=None ):
		rospy.logdebug( "setAutoDataLogging(): BEGIN" )

		if ((durationToAutoOn != None) and (isinstance(durationToAutoOn, float) or isinstance(durationToAutoOn, int))):
			self.start_logger_timer = rospy.Timer( rospy.Duration(durationToAutoOn), self.startAutoDataLogger_callback, oneshot=True)
			rospy.loginfo("CREATED self.start_logger_timer; will fire in " + str(durationToAutoOn) + " seconds...")

		if ((durationToAutoOff != None) and (isinstance(durationToAutoOff, float) or isinstance(durationToAutoOff, int))):
			self.stop_logger_timer = rospy.Timer( rospy.Duration(durationToAutoOff), self.stopAutoDataLogger_callback, oneshot=True)
			rospy.loginfo("CREATED self.stop_logger_timer; will fire in " + str(durationToAutoOff) + " seconds")

		rospy.logdebug( "setAutoDataLogging(): END" )
		return


	def startAutoDataLogger_callback( self, event ):
		rospy.logdebug("startAutoDataLogger_callback(): BEGIN")
		if self.start_logger_timer != None:
			rospy.loginfo("startAutoDataLogger_callback() called at " + str( event.current_real))
			## Synthesize a button press by publishing message
			#INSPIRE4Controller.pubTo_inspire_four_pilot_command( self, "log record start" )
			INSPIRE4Controller.toggleDataLoggerRecording( self, "stopped" )	## we want to start recording
		else:
			rospy.logdebug("INVALID ACTION: self.start_logger_timer = " + str(self.start_logger_timer))
		rospy.logdebug("startAutoDataLogger_callback(): END")
		return


	def stopAutoDataLogger_callback( self, event ):
		rospy.logdebug("stopAutoDataLogger_callback(): BEGIN")
		if self.stop_logger_timer != None:
			rospy.loginfo("stopAutoDataLogger_callback() called at " + str( event.current_real))
			## Synthesize a button press by publishing message
			#INSPIRE4Controller.pubTo_inspire_four_pilot_command( self, "log record stop" )
			INSPIRE4Controller.toggleDataLoggerRecording( self, "started" )	## we want to stop recording
		else:
			rospy.logdebug("INVALID ACTION: self.stop_logger_timer = " + str(self.stop_logger_timer))
		rospy.logdebug("stopAutoDataLogger_callback(): END")
		return


	def cancelAutoDataLoggerCallbacks( self ):
		rospy.logdebug("cancelAutoDataLoggerCallbacks(): BEGIN")

		## neutralize outstanding timers
		if self.stop_logger_timer != None:
			self.stop_logger_timer.shutdown()
			self.stop_logger_timer = None
			rospy.loginfo("CANCELLED self.stop_logger_timer")

		if self.start_logger_timer != None:
			self.start_logger_timer.shutdown()
			self.start_logger_timer = None
			rospy.loginfo("CANCELLED self.start_logger_timer")

		rospy.logdebug("cancelAutoDataLoggerCallbacks(): END")
		return


	## ------------------------------
	## stand alone timed skit
	## "Friend" actor will adapt to Maki-ro's skit
	def runFamiliarizationSkit( self ):
## KATE
		rospy.logdebug("runFamiliarizationSkit(): BEGIN")

		_verbose = True
		_thought = ""	## start off not thinking of anything, beginner's mind

		AA = asleepAwake( _verbose, self.ros_pub )
		lookIntro = lookINSPIRE4Intro( _verbose, self.ros_pub )

		## STEP -1: --- PRE-CHECK ---
		## Assumes that Maki-ro begins in the asleep position
		rospy.logdebug( "...\tPRE-CHECK...\tIs Maki-ro already sleeping? AA.asleep_p() = " + str( AA.asleep_p() ) )
		if AA.asleep_p():
			_thought = "Maki-ro is already asleep... ZZZzzzz ZZZZzzzzzz..."
			INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
			rospy.logwarn("runFamiliarizationSkit(): " + _thought + " SKIP")
			pass
		else:
			_thought = "Maki-ro is not yet asleep... Going to sleep..."
			INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
			rospy.logwarn("runFamiliarizationSkit(): " + _thought )
			## TODO: Add timers to see how long each step takes
			#_start_time = rospy.get_time()

			## blocking until Maki-ro is in the asleep position
			AA.runAsleep()

		## STEP 0: Officially declare the familiarization begun
		lookIntro.introStart()

		## STEP 1: Friend touches Maki-ro on the shoulder
		pass

		## STEP 2: Maki-ro wakes up and faces Friend
		_thought = "Maki-ro, wake up!!! Face Friend..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		AA.runAwakeExperimenter( disable_ht=False )
		rospy.logdebug("...\tMaki-ro should now be awake... AA.awake_p()=" + str( AA.awake_p() ))

		## STEP 3: Experimenter greets Maki-ro
		rospy.sleep(0.75)	## 750 ms

		## STEP 4: Maki-ro acknowledges greeting
		_thought = "Maki-ro, mind your manners and say hello..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.macroGreeting()

		## STEP 5: Friend plays peek-a-boo. Covers own eyes with hands, then uncover
		rospy.sleep(0.75)	## 750 ms

		## STEP 6: Maki-ro reacts with a startle expression and immediately relaxes
		_thought = "Maki-ro is startled!!! Peek-a-boo does that..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.startStartle( relax=True )
		### KATE HACKING: BEGIN...
		#rospy.logdebug("Maki-ro doesn't quiet fully relax, so adjust gaze to look at Friend again...")
		#rospy.sleep(0.5)	## need a brief moment before executing next motion
		### Maki-ro doesn't quiet come back to relax position
		#lookIntro.macroLookAtExperimenter()
		### More insurance that eyelids return to neutral
		#lookIntro.setEyelidNeutralPose( lookIntro.LL_NEUTRAL, cmd_prop=True )
		### KATE HACKING: END

		## STEP 7: Friend show Maki-ro the flashing ball
		rospy.sleep(0.5)	## 500 ms

		## TODO: Insert agentic behaviors where appropriate
		INSPIRE4Controller.setBlinkAndScan( self, blink=True )

		## STEP 8: Maki-ro nods
		_thought = "Oooo a blinky... Maki-ro tries to nod enthusiastically..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		## This hack works better than specifying repetitions as optional input parameter
		lookIntro.macroGreeting()
		rospy.sleep(0.5)
		lookIntro.macroGreeting()
		_thought = "Maki-ro might be a little dizzy now..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logdebug("...\t" + _thought)

		## STEP 9: Friend moves the flashing ball to the UPPER RIGHT calibration point
		##	and jiggles it to attract Maki-ro's attention
		rospy.sleep(0.5)

		## STEP 10: Maki-ro looks at the upper right calibration point
		_thought = "Maki-ro likes the fun flashy ball and here comes faux smooth pursuit..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.macroLookAtBallLocationRight( upper=True )

		## STEP 11: Friend moves the flashing ball to the LOWER RIGHT calibration point
		##	and jiggles it to attract Maki-ro's attention
		rospy.sleep(0.5)

		## STEP 12: Maki-ro looks at the lower right calibration point
		_thought = "Maki-ro thinks the flashy ball is pretty... There Maki-ro goes chasing the ball..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.macroLookAtBallLocationRight( lower=True )

		## STEP 13: Friend moves the flashing ball between the infant and Maki-ro
		##	and jiggles it to attract Maki-ro's attention
		rospy.sleep(0.5)

		## STEP 14: Maki-ro looks at the infant
		_thought = "Maki-ro... Follow, follow, follow, follow the flashy ball..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.macroLookAtInfant()

		## STEP 15: Friend retracts the flashy ball wand
		rospy.sleep(0.5)

		## STEP 16: Maki-ro follows the ball and faces Friend
		_thought = "Maki-ro... Follow and follow, follow the flashy ball... Oh hiya, Friend!!!..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.macroLookAtExperimenter()

		## STEP 17: Maki-ro watches Friend leave
		_thought = "Hey, wait!!! Where did Friend go?? Maki-ro was playing with Friend..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		rospy.sleep(0.5)

		## STEP 18: Maki-ro turns back to Infant
		_thought = "Maki-ro misses Friend... Sad... Lonely... Who will play with Maki-ro now??..."
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.macroLookAtInfant()

		## STEP 19: Maki-ro makes a new pal
		_thought = "Maki-ro spies with its eyes... a human baby!?!?"
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		rospy.sleep(0.5)

		## STEP 20: END OF FAMILIARIZATION
		_thought = "<<<< END SCENE >>>>"
		INSPIRE4Controller.pubTo_maki_internal_monologue( self, _thought )
		rospy.logwarn("runFamiliarizationSkit(): " + _thought)
		lookIntro.stop( disable_ht=True )
		## TODO: disable_ht might have to change once in the
		##	larger context of the experiment

		rospy.logdebug("runFamiliarizationSkit(): END")
		return


	## ------------------------------
	def parse_pilot_command( self, msg ):
		rospy.logdebug("parse_pilot_command(): BEGIN")
		rospy.logdebug("received: " + str(msg))

		self.previous_state = self.state	## for later comparison
		_unknown_flag = False
		_data = str(msg.data)
		rospy.loginfo("_data = " + _data)
	
		if _data == "init pilot GUI":
			rospy.loginfo( "Received initial message from clicking a button in the pilot's GUI" )			
		elif _data == "setup":
			INSPIRE4Controller.doSetup( self )
			self.state = INSPIRE4Controller.SETUP

		elif _data.startswith( "log record" ):
			## logic for managing start/stop log recording
			if _data.endswith( "start" ):
				INSPIRE4Controller.toggleDataLoggerRecording( self, "stopped" )	## we want to start the recording
				rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )	## add AFTER start recording

			elif _data.endswith( "stop" ):
				rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )	## add BEFORE stop recording
				INSPIRE4Controller.toggleDataLoggerRecording( self, "started" )	## we want to stop the rcording
				INSPIRE4Controller.cancelAutoDataLoggerCallbacks( self )

			else:
				_unknown_flag = True

		elif _data.startswith( "sync" ):
			if _data.endswith( "Tobii calibration start" ):
				## If the 'setup' button hadn't been pressed prior to this, fake it
				if (self.__is_setup_done == None) or (not self.__is_setup_done):
					INSPIRE4Controller.doSetup( self )

				if self.data_logger_status == "started":
					## There is an actively recording rosbag,
					##	so close the existing one and start a new one
					INSPIRE4Controller.toggleDataLoggerRecording( self, "started" )	## we want to stop recording
					rospy.sleep(0.5)	## 0.5 second delay to allow time to close rosbag
					INSPIRE4Controller.toggleDataLoggerRecording( self, "stopped" )	## we want to start recording
					## resend this message for synchronization purposes
					_msg = _data + " (resend to mark the new rosbag)"
					INSPIRE4Controller.pubTo_inspire_four_pilot_command( self, _msg )
				else:
					## There is no actively recording rosbag, 
					##	so start a new one
					INSPIRE4Controller.toggleDataLoggerRecording( self, "stopped" )	## we want to start recording
					rospy.sleep(0.5)	## 0.5 second delay to allow time to open rosbag
					## resend this message for synchronization purposes
					_msg = _data + " (resend to mark the new rosbag)"
					INSPIRE4Controller.pubTo_inspire_four_pilot_command( self, _msg )

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
				rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )
				self.state = INSPIRE4Controller.PRELUDE

## KATE
		elif _data == "runFamiliarizationSkit":
			try:
				thread.start_new_thread( INSPIRE4Controller.runFamiliarizationSkit, ( self,  ) )
			except Exception as _e_rFS:
				rospy.logerror("Unable to start new thread for INSPIRE4Controller.runFamiliarizationSkit()")

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
					rospy.logdebug("\tInfant")
					rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )
					## KATE LIVE HACKING: BEGIN
					if (self.state != INSPIRE4Controller.INTRO):
						INSPIRE4Controller.pubTo_maki_macro( self, "intro start" )
						self.state = INSPIRE4Controller.INTRO
					## KATE LIVE HACKING: END
					INSPIRE4Controller.transitionToEngagement( self, _data )
					return
				else:
					## KATE LIVE HACKING: BEGIN
					INSPIRE4Controller.pubTo_maki_macro( self, _data )
					## KATE LIVE HACKING: END
					rospy.logdebug( _data )
			else:
				rospy.logdebug("prefix = intro, SUFFIX IS UNKNOWN: " + _data)
				_unknown_flag = True

			if not _unknown_flag:
				rospy.loginfo("prefix = intro; forward the message contents to /maki_macro: " + _data)
				## forward the message contents
				INSPIRE4Controller.pubTo_maki_macro( self, _data )
				self.state = INSPIRE4Controller.INTRO
				rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )
			
		elif (_data.startswith( "startleGame stop" )) or (_data == "infant fixation"):
			rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )
			INSPIRE4Controller.transitionToStimuli( self )

		elif ("tartle" in _data):
			if _data == "startleGame start":	self.__is_game_running = True
			rospy.loginfo("'tartle' in _data; forward the message contents to /maki_macro: " + _data)
			## forward the message contents
			INSPIRE4Controller.pubTo_maki_macro( self, _data )
			self.state = INSPIRE4Controller.ENGAGEMENT

		elif ("turnToScreen" in _data):
			rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )

			rospy.loginfo("'turnToScreen' in _data; forward the message contents to /maki_macro: " + _data)
			## forward the message contents
			INSPIRE4Controller.pubTo_maki_macro( self, _data )
			self.state = INSPIRE4Controller.STIMULI

			if _data.endswith( "auto_return=True" ):
				## add timed trigger 'watch stimuli' behavior 
				## and followed by turning back to face the infant
				INSPIRE4Controller.setAutoTransitionWatchStimuli( self )
			## TODO: Set a timer to enable blink and scan

		elif _data == "turnToInfant":
			rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )

			rospy.logwarn("====> turnToInfant")
			INSPIRE4Controller.transitionToEngagement( self, _data )
			rospy.sleep(1.0)	## it takes 1 second to return from facing left/right screen
			self.interaction_count += 1
			rospy.loginfo( str(self.interaction_count) + " of " + str(INSPIRE4Controller.NUMBER_OF_INTERACTIONS) + " INTERACTIONS have occurred" )
			if self.interaction_count < INSPIRE4Controller.NUMBER_OF_INTERACTIONS:
				pass
				## TODO: automatically start playing startle game
			else:
				rospy.loginfo( "NOW BEGIN AUTOMATICALLY TERMINATING THE EXPERIMENT" )
				INSPIRE4Controller.pubTo_inspire_four_pilot_command( self, "ending start" )

		elif (_data == "outro start") or (_data == "ending start"):
			rospy.loginfo( "ADD SYNC MARKER: " + str(_data) )

			INSPIRE4Controller.doSetup( self )
			self.state = INSPIRE4Controller.END
			if self.data_logger_status == "started":
				INSPIRE4Controller.setAutoDataLogging( self, durationToAutoOff=float(60.0 * 5) )


		## TODO: Add logic for participant ID


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

	rospy.Subscriber( "/data_logger/status", String, controller.updateDataLoggerStatus_callback )
	rospy.logdebug( "now subscribed to /data_logger/status" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


