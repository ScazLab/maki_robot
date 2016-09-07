#! /usr/bin/env python

# 	NOTE:	To be run in conjunction with INSPIRE4_freeplay_annex_controller and master-table.xls

import rospy
import re
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_srvs.srv import Empty

import signal
import sys
import string
import random
import thread
import threading

from maki_robot_common import *
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

from blinking import *
from selective_attention import *
from INSPIRE4_look_intro import *
from idle_head_pan import *

from base_behavior import *


## ------------------------------
class freeplayAgency( object ):
	## all instances of this class will share the same value
	## variables private to this class
	__is_scanning = None
	__is_blinking = None
	__is_nodding = None

	ENABLE_VS = None
	ENABLE_BL = None
	ENABLE_IL = None

	def __init__(self, verbose_debug, ros_pub):

		## primarily will publish to /maki_macro
		if ros_pub == None:
			self.initROS( self )
		else:
			self.ros_pub = ros_pub		## can we pass a ros publisher??? Apparently so!
		freeplayAgency.initROSPub( self )

		self.ALIVE = False
		##self.__is_robot_ready = False
		##self.__is_running_stimuli=False
		##freeplayAgency.resetInteractionCount( self )
		##
		##self.state = None
		##self.previous_state = None
		##
		#self.durationHeadTurn = 1.0
		#self.durationWatchStimuli = 8.0
		#self.blocking_gui = True
		#
		### instead of passing messages, instantiate the behaviors
		##self.asleepAwake = asleepAwake( verbose_debug, self.ros_pub )
		#self.lookIntro = lookINSPIRE4Intro( verbose_debug, self.ros_pub )
		##self.startleGame = engagementStartleGame( verbose_debug, self.ros_pub )
		##self.lookStimuli = lookINSPIRE4Interaction( verbose_debug, self.ros_pub )
		##self.eyelids = blinking( verbose_debug, self.ros_pub )
		#self.blinking = blinking( verbose_debug, self.ros_pub )
		#self.scanning = selectiveAttention( verbose_debug, self.ros_pub )
		### and a generic one for dealing with resetting to neutral
		#self.htBB = headTiltBaseBehavior( False, self.ros_pub )

		self.idleHP = idleHeadPan( True, self.ros_pub )

		self.pause_blinkPrep = False
		self.blink_prep_timer = None
		self.scan_pause_timer = None
		self.scan_resume_timer = None
		self.scan_pause_resume_timer_count = 0

		freeplayAgency.__is_blinking = False
		freeplayAgency.__is_scanning = False
		freeplayAgency.__is_nodding = False

		self.duration_until_scan_pause = 0
        	self.thread_pauseResumeScan = None

		freeplayAgency.ENABLE_VS = False
		freeplayAgency.ENABLE_BL = True
		freeplayAgency.ENABLE_IL = True

		self.ALIVE = True
		return


	#####################
	## Set up publisher to /maki_macro
	#####################
	def initROSPub( self, latch=False ):
		rospy.logdebug( "setup rostopic publisher to /maki_macro" )
		self.ros_pub = rospy.Publisher( "maki_macro", String, queue_size = 26, latch = latch)	## if LATCH==True, any new subscribers will see the most recent message published
	
		## publisher showing robot's current agency behaviors
		self.ag_info = rospy.Publisher("agency_info", String, queue_size = 10)

		return 

	def pubTo_maki_macro( self, commandOut, cmd_prop=True, time_s=0.1 ):
		rospy.logdebug( commandOut )
		if not rospy.is_shutdown():
			self.ros_pub.publish( commandOut )
			if cmd_prop:	rospy.sleep(time_s)
		return


	## ------------------------------
	## durations in seconds
	def setBlinkTimer( self, msg ):
		next_blink = 0

		if ((msg.data != None) and (isinstance(msg.data, int) and
			(msg.data > 0))):
			next_blink = msg.data
			freeplayAgency.__is_blinking = True
			self.pause_blinkPrep = False
		else:
			freeplayAgency.__is_blinking = False
			self.pause_blinkPrep = False
			freeplayAgency.cleanupBlinkPrepTimer( self )
			return False

		next_blink = float(next_blink) - 1	#0.5	## end a bit early

		self.blink_prep_timer = rospy.Timer(rospy.Duration(next_blink), self.blinkPrepTimer_callback, oneshot=True)

		return

	def blinkPrepTimer_callback( self, event ):
		_pause_duration = 0.01	## seconds
		_resume_duration = 2.0

		if not freeplayAgency.__is_blinking:	return

		## before upcoming blink, make sure to pause scanning
		if freeplayAgency.__is_scanning:
			rospy.loginfo("blinkPrepTimer_callback: __is_scanning=" + str(freeplayAgency.__is_scanning))
			self.pause_blinkPrep = True
			freeplayAgency.cancelScanTimers( self )
			## fire immediately
			self.scan_pause_timer = rospy.Timer(rospy.Duration(_pause_duration), self.scanPauseTimer_callback, oneshot=True)
			self.scan_resume_timer = rospy.Timer(rospy.Duration(_pause_duration+_resume_duration), self.scanResumeTimer_callback, oneshot=True)


		## similar for idle head pan
		if self.idleHP.isIdling():
			self.idleHP.setIdlingPauseResumeTimers( pause_duration=_pause_duration, resume_duration=_resume_duration )


		## TODO: similar for head nodding

		return

	def cancelBlinkPrepTimer( self ):
		## neutralize outstanding timers
		if self.blink_prep_timer != None:
			self.blink_prep_timer.shutdown()
			self.blink_prep_timer = None
			rospy.logdebug("CANCELLED self.blink_prep_timer")
		return

	def cleanupBlinkPrepTimer( self ):
		self.pause_blinkPrep = False
		freeplayAgency.cancelBlinkPrepTimer( self )
		freeplayAgency.pubTo_maki_macro( self, "reset eyelids")

		## if visual scanning thread was also running... reset it
		if freeplayAgency.__is_scanning:
			rospy.loginfo("cleanupBlinkPrepTimer: reset visual scanning")
			freeplayAgency.setVisualScanTimer( self, Int16( 4 ) )

	## ------------------------------
	## durations in seconds
	def setVisualScanTimer( self, msg ):
		## check if visual scanning already exists
		if freeplayAgency.__is_scanning:
			freeplayAgency.cancelScanTimers( self )
			freeplayAgency.__is_scanning = False
        		#self.thread_pauseResumeScan = None

		duration_until_scan_pause = 0

		if ((msg.data != None) and (isinstance(msg.data, int) and
			(msg.data > 0))):
			self.duration_until_scan_pause = msg.data
			freeplayAgency.__is_scanning = True
		else:
			#freeplayAgency.cleanupRunRandomPauseResumeScan( self )
			freeplayAgency.__is_scanning = False
			return False

		# start new thread to randomly pause and resume visual scanning
		try:
			thread.start_new_thread( freeplayAgency.runRandomPauseResumeScan, ( self, duration_until_scan_pause ) )
		except Exception as _e_rFS:
			rospy.logerr("Unable to start new thread for freeplayAgency.runRandomPauseResumeScan()")
		#self.thread_pauseResumeScan = threading.Thread(target=freeplayAgency.runRandomPauseResumeScan, args=())
		#self.thread_pauseResumeScan = threading.Thread(target=self.runRandomPauseResumeScan, args=())
        	#self.thread_pauseResumeScan.setDaemon(True)     # make sure to set this; otherwise, stuck with this thread open
        	#self.thread_pauseResumeScan.start()


	def runRandomPauseResumeScan( self, duration_until_scan_pause ): 
	#def runRandomPauseResumeScan( self ): 
		#duration_until_scan_pause = self.duration_until_scan_pause

		_count = 0
		rospy.logwarn("runRandomPauseResumeScan: BEFORE while()")
		while (self.ALIVE and not rospy.is_shutdown() and
			freeplayAgency.__is_scanning):

			rospy.sleep(0.5)
			if self.pause_blinkPrep and freeplayAgency.__is_blinking:	continue

			_rand_pause = random.uniform(duration_until_scan_pause, duration_until_scan_pause+1)
			_rand_resume = random.uniform(0, duration_until_scan_pause)

			## ONLY SET NEW PAUSE and RESUME TIMERS if BOTH have expired
			if ( self.scan_pause_timer == None and self.scan_resume_timer == None and
				self.scan_pause_resume_timer_count <= 0 ):
				self.scan_pause_timer = rospy.Timer(rospy.Duration(_rand_pause), self.scanPauseTimer_callback, oneshot=True)
				self.scan_pause_resume_timer_count += 1
				self.scan_resume_timer = rospy.Timer(rospy.Duration(_rand_pause+_rand_resume), self.scanResumeTimer_callback, oneshot=True)
				self.scan_pause_resume_timer_count += 1

				_count += 1
				#rospy.loginfo(" runRandomPauseResumeScan: _count=" + str(_count) + "; self.scan_pause_resume_timer_count=" + str(self.scan_pause_resume_timer_count))

		rospy.logwarn("runRandomPauseResumeScan: AFTER while()")
		freeplayAgency.cleanupRunRandomPauseResumeScan( self )
		rospy.logwarn("runRandomPauseResumeScan: DONE")
		return

	def cleanupRunRandomPauseResumeScan( self ):
		_my_is_blinking = freeplayAgency.__is_blinking
		if _my_is_blinking:
			freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop auto_reset_eyelid" )

		freeplayAgency.cancelScanTimers( self )
		self.scan_pause_timer = rospy.Timer(rospy.Duration(0.1), self.scanPauseTimer_callback, oneshot=True)
		rospy.sleep(0.25)	## make sure that scanPauseTimer_callback publishes
		#freeplayAgency.cancelScanTimers( self )
		self.scan_pause_resume_timer_count = 0
		freeplayAgency.pubTo_maki_macro( self, "reset selectiveAttention" )

		if _my_is_blinking:
			freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink start" )
		rospy.logwarn("cleanupRunRandomPauseResumeScan: DONE CLEANUP")
		return

	def scanPauseTimer_callback( self, event ):
		## simulate a longer pause
		#if freeplayAgency.__is_scanning and self.scan_pause_timer != None:
		if self.scan_pause_timer != None:
			rospy.loginfo("scanPauseTimer_callback: FIRED!!")
			_pub_cmd = "visualScan stop"
			_pub_cmd += " disable_ht=False"
			freeplayAgency.pubTo_maki_macro( self, _pub_cmd )
			self.scan_pause_timer = None	## IS THIS NECESSARY??	-- Helps for restarting visualScan
			self.scan_pause_resume_timer_count -= 1

		return

	def scanResumeTimer_callback( self, event ):
		## resume from pause
		if freeplayAgency.__is_scanning and self.scan_resume_timer != None:
			_pub_cmd = "visualScan start"
			freeplayAgency.pubTo_maki_macro( self, _pub_cmd )
			self.scan_resume_timer = None	## IS THIS NECESSARY??
			self.scan_pause_resume_timer_count -= 1
		return

	def cancelScanTimers( self ):
		## neutralize outstanding timers
		if self.scan_pause_timer != None:
			self.scan_pause_timer.shutdown()
			self.scan_pause_timer = None
			rospy.logdebug("CANCELLED self.scan_pause_timer")
			## TO REMOVE
			### for good measure
			#self.scan_pause_timer = rospy.Timer(rospy.Duration(0.1), self.scanPauseTimer_callback, oneshot=True)

		if self.scan_resume_timer != None:
			self.scan_resume_timer.shutdown()
			self.scan_resume_timer = None
			rospy.logdebug("CANCELLED self.scan_resume_timer")
		return





	def setBlinkAndScan( self, blink=False, auto_reset_eyelid=True, scan=False, disable_ht=False ):
		rospy.logdebug("setBlinkAndScan(): BEGIN")
		_pub_cmd = ""

		if (isinstance(blink, bool) and blink):
			freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink start" )
		else:
			if auto_reset_eyelid:
				freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop auto_reset_eyelid" )
			else:
				freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop" )

		if (isinstance(scan, bool) and scan):
			freeplayAgency.pubTo_maki_macro( self, "visualScan start" )
		else:
			_pub_cmd = "visualScan stop"
			if not disable_ht:	_pub_cmd += " disable_ht=False"
			freeplayAgency.pubTo_maki_macro( self, _pub_cmd )

		rospy.logdebug("setBlinkAndScan(): END")
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
			# if self.VERBOSE_DEBUG:
			# 		self.ros_pub = rospy.init_node(str(nodename), anonymous=_anon_rosnode, log_level=rospy.DEBUG)
			# rospy.logdebug("log_level=rospy.DEBUG")
			# else:
			self.ros_pub = rospy.init_node(nodename, anonymous=_anon_rosnode)       ## defaults to log_level=rospy.INFO
		
			rospy.logdebug("anonymous=" + str(_anon_rosnode))
			rospy.loginfo( str(_fname) + ": END")
		return


	## ------------------------------
	def parse_cmd( self, msg ):
		rospy.logdebug("parse_cmd(): BEGIN")
		rospy.logdebug("received: " + str(msg))

		_data = str(msg.data)
		#rospy.logdebug("_data = " + _data)
		rospy.loginfo("_data = " + _data)

		if "start" in _data:
			## THERE IS AN ORDERING HERE.. put visualScan first
			if (freeplayAgency.ENABLE_VS and 
				("visualScan" in _data or "selectiveAttention" in _data)):
				#freeplayAgency.pubTo_maki_macro( self, "reset selectiveAttention" )

				## BEFORE RESETTING, NEED TO STOP FIRST
				if freeplayAgency.__is_scanning:
					freeplayAgency.setVisualScanTimer( self, Int16( 0 ) )
					freeplayAgency.pubTo_maki_macro( self, "reset selectiveAttention" )

				rospy.sleep(0.5)
				freeplayAgency.setVisualScanTimer( self, Int16( 4 ) )

			if (freeplayAgency.ENABLE_BL and "spontaneousBlink" in _data):
				## BEFORE RESETTING, NEED TO STOP FIRST
				if freeplayAgency.__is_blinking:
					#freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop" )
					#freeplayAgency.cleanupBlinkPrepTimer( self )

					freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop auto_reset_eyelid" )
					## DO something more immediate
					freeplayAgency.setBlinkTimer( self, Int16(0) )
					freeplayAgency.pubTo_maki_macro( self, "reset eyelids" )

				rospy.sleep(0.5)
				freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink start" )

			if (freeplayAgency.ENABLE_IL and "idleHeadPan" in _data):
				self.idleHP.start()
				# start new thread for idleHeadPan.doIdle()
				try:
					thread.start_new_thread( self.idleHP.doIdle, ( self, ) )
				except Exception as _e_DI:
					rospy.logerr("Unable to start new thread for self.idleHP.doIdle()")

		elif "stop" in _data:
			## THERE IS AN ORDERING HERE.. put visualScan first
			if ("visualScan" in _data or "selectiveAttention" in _data):
				freeplayAgency.setVisualScanTimer( self, Int16( 0 ) )
				#freeplayAgency.pubTo_maki_macro( self, "visualScan stop disable_ht=False" )
				rospy.sleep(0.5)

			if "spontaneousBlink" in _data:
				freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop auto_reset_eyelid" )
				## DO something more immediate
				freeplayAgency.setBlinkTimer( self, Int16(0) )
				rospy.sleep(0.5)
			
			if "idleHeadPan" in _data:
				self.idleHP.abort()
				rospy.sleep(0.5)

		elif (_data == "reset selectiveAttention"):
			## BEFORE RESETTING, NEED TO STOP FIRST
			if freeplayAgency.__is_scanning:
				freeplayAgency.setVisualScanTimer( self, Int16( 0 ) )
				#freeplayAgency.cleanupRunRandomPauseResumeScan( self )

			freeplayAgency.pubTo_maki_macro( self, _data )

		elif (_data == "reset eyelids"):
			## BEFORE RESETTING, NEED TO STOP FIRST
			if freeplayAgency.__is_blinking:
				#freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop" )
				#freeplayAgency.cleanupBlinkPrepTimer( self )

				freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop auto_reset_eyelid" )
				## DO something more immediate
				freeplayAgency.setBlinkTimer( self, Int16(0) )

			freeplayAgency.pubTo_maki_macro( self, _data )

		else:
			rospy.logwarn("[WARNING] UNKNOWN AGENCY COMMAND: " + _data)

		rospy.logdebug("parse_cmd(): END")
		return


## ------------------------------
	def controllerExit( self ):
		rospy.logdebug("controllerExit(): BEGIN")

		## nicely stop/exit the behaviors too
		if freeplayAgency.__is_blinking:
			freeplayAgency.pubTo_maki_macro( self, "spontaneousBlink stop" )
			freeplayAgency.cleanupBlinkPrepTimer( self )

		if freeplayAgency.__is_scanning:
			freeplayAgency.setVisualScanTimer( self, Int16( 0 ) )
			freeplayAgency.cleanupRunRandomPauseResumeScan( self )

		if self.idleHP.isIdling():
			self.idleHP.abort()

		rospy.sleep(1)  # give a chance for everything else to shutdown nicely
		self.ALIVE = False
		rospy.sleep(1)  # give a chance for everything else to shutdown nicely

		rospy.logdebug( "controllerExit: SHUTTING DOWN FREEPLAY AGENCY..." )
		self.ag_info.publish('----- SHUTTING DOWN FREEPLAY AGENCY -----')
		self.ag_info.publish("==========================================")
		rospy.logdebug("controllerExit(): END")
		#exit    ## meant for interactive interpreter shell; unlikely this actually exits


def signal_handler(signal, frame):
	global controller
	rospy.loginfo( "signal_handler: CTRL+C" )
	controller.controllerExit()
	rospy.loginfo( "signal_handler: CTRL+C says goodnight" )
	sys.exit()      ## use this instead of exit (which is meant for interactive shells)
## ------------------------------


if __name__ == '__main__':
	print "__main__: BEGIN"

	global controller 
	controller = freeplayAgency( True, None )
	rospy.logdebug("-------- controller.__init__() DONE -----------")
	controller.ag_info.publish("=========================================")
	controller.ag_info.publish("...\tInitializing freeplay agency... Please wait.")

	# allow closing the program using CTRL+C
	#signal.signal(signal.SIGINT, signal_handler)
	# Register shutdown hook
	rospy.on_shutdown(controller.controllerExit)

	rospy.Subscriber( "blinking/next_blink", Int16, controller.setBlinkTimer )
	rospy.logdebug( "now subscribed to /blinking/next_blink" )

	rospy.Subscriber( "freeplay_agency", String, controller.parse_cmd )
	rospy.logdebug( "now subscribed to /freeplay_agency" )

	#rospy.logdebug("-------- controller.start() DONE -----------")
	controller.ag_info.publish("...\tInitializing freeplay agency... DONE")

	rospy.spin()   ## keeps python from exiting until this node is stopped

	print "__main__: END"


