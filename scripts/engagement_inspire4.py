#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import os

import math
import string

from maki_robot_common import *
from dynamixel_conversions import dynamixelConversions
from base_behavior import *	## baseBehavior, headTiltBaseBehavior, eyelidBaseBehavior, eyelidHeadTiltBaseBehavior
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

HT_STARTLE = 525

########################
## Infant engagement behaviors for INSPIRE4 study
##
## STARTLE
## Description: from neutral, MAKI rapidly opens eyes wide (LL_OPEN_MAX)
##	and head lifts slightly (HT). 
##
## TODO:
##	* Eye tilt (ET) compensates for HT
########################
class engagementStartleGame( eyelidHeadTiltBaseBehavior ):	#headTiltBaseBehavior, eyelidBaseBehavior ):
	## variables private to this class
	## all instances of this class share the same value
	__is_startled = None


	def __init__(self, verbose_debug, ros_pub):
		## call base class' __init__
		eyelidHeadTiltBaseBehavior.__init__( self, verbose_debug, ros_pub )

		## add anything else needed by an instance of this subclass
		engagementStartleGame.__is_startled = False

		## Game variables
		self.ll_startle = LL_OPEN_MAX

		self.last_startle_time = None
		self.next_startle_time = None

		self.repetitions = 6	## do 6 rounds of infant engagement behavior max

		self.duration_between_startle_min = 2	## seconds
		self.duration_between_startle_max = 5	## seconds

		return


	## DO NOT USE -- UNDER DEVELOPMENT -- NOT TESTED	
	def runGame( self, repetitions=None, duration=None ):
		pass
		'''
		rospy.logdebug("runGame: BEGIN")

		if repetitions == None and duration == None:	
			repetitions = self.repetitions

		self.count_movements = 0
		_initial_startle = True
		_duration_to_startle = float( self.startle_action_duration ) 
		_start_time = rospy.get_time()
		self.next_startle_time = _start_time
		## this is a nested while loop
		while self.ALIVE and not rospy.is_shutdown():

			if self.mTT_INTERRUPT:	
				rospy.logdebug("mTT_INTERRUPT=" + str(self.mTT_INTERRUPT))
				##print "start sleep 5"
				#self.SWW_WI.sleepWhileWaiting(5)	# 5 seconds
				##print "end sleep 5"
				#continue	## begin loop again from the beginning skipping below
				##print "shouldn't get here"
				return

			_remaining_duration = self.next_startle_time - rospy.get_time()
			if (_remaining_duration > 0.01):	## 100 ms
				continue
			else:
				### FPPZ request published takes 100ms to propogate
				engagementStartleGame.requestFeedback( self, SC_GET_PP )

			if _initial_startle:
				## Calculate goal speed
				_distance_to_startle = abs( self.makiPP["LL"] - self.ll_startle )
				_gs_ll = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_startle, _duration_to_startle ) 
			engagementStartleGame.pubTo_maki_command( self, "LLGS" + str(_gs_ll) + str(TERM_CHAR_SEND))

			rospy.logdebug("SPONTANEOUS BLINK... eyelid close, GO!")
			_startle_start_time = rospy.get_time()
			## NOTE: cmd_prop must be set to True in eyelidClose
			## otherwise subsequent eyelidOpen won't open corectly
			## almost 50% of the time
			engagementStartleGame.eyelidClose( self, cmd_prop=True )	
			rospy.logdebug("SPONTANEOUS BLINK... eyelid close, DONE! eyelid open, GO!")
			engagementStartleGame.eyelidOpen( self, monitor=True )
			_startle_end_time = rospy.get_time()
			rospy.logdebug("SPONTANEOUS BLINK... eyelid open, DONE!")
			self.last_blink_time = rospy.get_time()
			rospy.loginfo("spontaneous blink duration: " + str( abs(_startle_end_time - _startle_start_time) ) + " seconds" )
			self.count_movements = self.count_movements +1

			_seconds_until_next_blink = random.uniform(self.spontaneous_blink_rate_min, self.spontaneous_blink_rate_max)
			self.next_blink_time = self.last_blink_time + _seconds_until_next_blink
			rospy.logdebug("Next spontaneous blink in " + str(_seconds_until_next_blink) + " seconds... at time " + str(self.next_blink_time))

			_duration = abs(rospy.get_time() - _start_time)
		#end	while self.ALIVE and not rospy.is_shutdown():

		rospy.loginfo( "NUMBER OF SPONTANEOUS BLINKS: " + str(self.count_movements) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("macroSpontaneousBlink: END")
		return
		'''

	############################
	##
	##	To run, publish to /maki_macro:
	##		hideFromStartle
	##
	############################
	## TODO: Below is a work in progress
	def hideFromStartle( self ):
		if (engagementStartleGame.__is_startled == False):	
			rospy.logerr("hideFromStartle: INVALID STATE: engagementStartleGame.__is_startled=" + str(engagementStartleGame.__is_startled))
			return

		## taken from macroStartleRelax
		## TODO: prefix with self.
		HT_STARTLE = 525	#530	#525
		HT_NEUTRAL = HT_MIDDLE
		HT_GS_DEFAULT = 15
		HT_GS_MAX = 75	#60	#50
		HT_GS_MIN = 10

		LL_STARTLE = LL_OPEN_MAX
		LL_NEUTRAL = LL_OPEN_DEFAULT
		LL_GS_MIN = 10

		## (_gs_ll, _gs_ht)
		## distance covered in 200 ms timestepd
		## ((50%, 20%), (50%, 40%), (0%, 25%), (0%, 10%), (0%, 5%))
		_gs_sequence = ((192,29), (192,57), (192,36), (192,14), (192,7))
		_duration_into_hide = 1000	## milliseconds
		_step_duration = float( _duration_into_hide / len(_gs_sequence) )

		_pub_cmd = ""

		_start_time = rospy.get_time()
		if engagementStartleGame.__is_startled and (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):

			_ll_gp = LL_CLOSE_MAX
			_ht_gp = HT_DOWN

			engagementStartleGame.requestFeedback( self, SC_GET_PP )
			_ll_start_pp = self.makiPP["LL"]
			_ht_start_pp = self.makiPP["HT"] 
			
			_loop_count = 0
			_first_pass = True
			for _gs_ll, _gs_ht in _gs_sequence:
				_start_time_step = rospy.get_time()

				_pub_cmd = ""
				_pub_cmd += "LLGS" + str(_gs_ll)
				_pub_cmd += "HTGS" + str(_gs_ht)
				_pub_cmd += TERM_CHAR_SEND
				rospy.loginfo( _pub_cmd )
				engagementStartleGame.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
				## has 100 ms delay for propogation to motors

				if _first_pass:
					_pub_cmd = ""
					_pub_cmd += "LLGP" + str(_ll_gp)
					_pub_cmd += "HTGP" + str(_ht_gp)
					_pub_cmd += TERM_CHAR_SEND
					rospy.logwarn( _pub_cmd )
					engagementStartleGame.pubTo_maki_command( self, _pub_cmd, cmd_prop=True )
					_first_pass = False
			
				if (abs(rospy.get_time() - _start_time) < _duration_into_hide):
					## has 100 ms delay for propogation to motors
					engagementStartleGame.requestFeedback( self, SC_GET_PP )

					## compensate to maintain pacing of 200 ms apart
					_adjusted_sleep = _step_duration - abs(rospy.get_time() - _start_time_step)
					if (_adjusted_sleep <= 0):
						rospy.logdebug("... no sleep _step_duration adjustment")
					else:	
						rospy.logdebug( str(_adjusted_sleep) + " milliseconds more are needed to fulfill _step_duration pacing")
						self.SWW_WI.sleepWhileWaitingMS( _adjusted_sleep, end_early=False )

					_loop_count = _loop_count +1
				else:
					rospy.logdebug("TIME IS UP")
					break	

			#end	for _gs_ll, _gs_ht in _gs_sequence:
			engagementStartleGame.__is_startled = False
		else:
			return
		#end	if engagementStartle.__is_startled and Game(self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):

		_duration = abs(rospy.get_time() - _start_time)
		rospy.loginfo( "NUMBER OF TIMESTEPS: " + str(_loop_count) )
		rospy.loginfo( "Duration: " + str(_duration) + " seconds" )

		rospy.logdebug("hideFromStartle: END")
		return

	############################
	##
	##	To run, publish to /maki_macro:
	##		unhideIntoStartle
	##
	############################
	## TODO: Below is temporary
	def unhideIntoStartle( self ):
		rospy.logdebug("unhideIntoStartle: BEGIN")

		## reset to neutral position
		## resets goal speeds too
		engagementStartleGame.pubTo_maki_command( self, "reset" )	
		## wait for reset motion to complete
		## meanwhile, print out the present speeds while moving
		for _i in range(10):
			engagementStartleGame.requestFeedback( self, SC_GET_PS )
			self.SWW_WI.sleepWhileWaitingMS( 100, end_early=False )		

		engagementStartleGame.macroStartleRelax( self, relax=False )

		rospy.logdebug("unhideIntoStartle: END")
		return

	############################
	##	To run, publish to /maki_macro:
	##		startle start
	##
	##	To stop, publish to /maki_macro:
	##		startle stop
	##
	## By default, Maki-ro will perform one startle action followed by
	## one relax action and stop
	############################
	def macroStartleRelax( self, startle=True, relax=True, repetitions=1 ):
		rospy.logdebug("macroStartleRelax(): BEGIN")

		_start_time = rospy.get_time()
		## check the inputs
		if isinstance(startle, bool) and (not startle) and isinstance(relax, bool) and (not relax):	return
		if isinstance(repetitions, int) and (repetitions > 0):
			pass
		else:
			rospy.logwarn("macroStartleRelax(): INVALID VALUE: repetitions=" + str(repetitions) + "; updated to 1")
			repetitions = 1

		## TODO: move these variables to global scope
		HT_STARTLE = 525	#530	#525
		HT_NEUTRAL = HT_MIDDLE
		HT_GS_DEFAULT = 15		## as set in Arbotix-M driver
		HT_GS_MAX = 75	#60	#50
		HT_GS_MIN = 10

		LL_STARTLE = LL_OPEN_MAX
		LL_NEUTRAL = LL_OPEN_DEFAULT
		LL_GS_DEFAULT = 100	## as set in Arbotix-M driver
		LL_GS_MIN = 10

		## generate servo control command to set goal positions
		## NOTE: on the Arbotix-M side, a sync_write function is used
		## to simultaneously broadcast the updated goal positions
		_startle_gp_cmd = ""
		_startle_gp_cmd += "LL" + SC_SET_GP + str(LL_STARTLE)
		_startle_gp_cmd += "HT" + SC_SET_GP + str(HT_STARTLE)
		_startle_gp_cmd += TERM_CHAR_SEND
		_relax_gp_cmd  = ""
		_relax_gp_cmd += "LL" + SC_SET_GP + str(LL_NEUTRAL)
		_relax_gp_cmd += "HT" + SC_SET_GP + str(HT_NEUTRAL)
		_relax_gp_cmd += TERM_CHAR_SEND
		_pub_cmd = ""

		## Store initial pose
		engagementStartleGame.requestFeedback( self, SC_GET_PP )
		_previous_ll = self.makiPP["LL"]
		_previous_ht = self.makiPP["HT"]

		if relax and (not startle):
			rospy.loginfo("relax ONLY... skip alignment")
			pass
		else:
			## Move to neutral eyelid and head tilt pose
			rospy.loginfo("BEFORE startle, adjust LL and HT to NeutralPose")
			_pub_cmd = ""
			if (abs(self.makiPP["LL"] - LL_NEUTRAL) > DELTA_PP):
				_pub_cmd += "LLGP" + str(LL_NEUTRAL)
			if (abs(self.makiPP["HT"] - HT_NEUTRAL) > DELTA_PP):
				_pub_cmd += "HTGP" + str(HT_NEUTRAL) 
			if ( len(_pub_cmd) > 0 ):
				engagementStartleGame.monitorMoveToGP( self, _pub_cmd, ll_gp=LL_NEUTRAL, ht_gp=HT_NEUTRAL )
				#self.SWW_WI.sleepWhileWaiting(1)	## 1 second	## debugging

		_duration = abs(rospy.get_time() -_start_time)
		rospy.loginfo("OVERHEAD SETUP TIME: " + str(_duration) + " seconds")

		## TODO: unify duration_startle
		#_duration_startle = 100		## millisecond
		_duration_relax = 1000		## milliseconds
		_duration_relax_wait = 250
		_loop_count = 0
		_start_time = rospy.get_time()
		while (_loop_count < repetitions) and (self.ALIVE) and (not self.mTT_INTERRUPT) and (not rospy.is_shutdown()):
			rospy.logdebug("-------------------")

			if startle:
				rospy.loginfo("====> STARTLE")
				engagementStartleGame.requestFeedback( self, SC_GET_PP )
				rospy.logdebug( str(self.makiPP) )

				## Calculate goal speed base on distance and duration (in milliseconds)
				_duration_startle = 100		## millisecond
				_distance_to_startle_ll = abs( self.makiPP["LL"] - LL_STARTLE )
				rospy.logdebug("_distance_startle_ll=" + str(_distance_to_startle_ll))
				_gs_ll = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_startle_ll, _duration_startle) )
				rospy.loginfo("_gs_ll=" + str(_gs_ll))

				_duration_startle = 150	#200	#250		## millisecond
				_distance_to_startle_ht = abs( self.makiPP["HT"] - HT_STARTLE )
				rospy.logdebug("_distance_startle_ht=" + str(_distance_to_startle_ht))
				_gs_ht = abs( self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_startle_ht, _duration_startle) )
				rospy.loginfo("_gs_ht=" + str(_gs_ht))
				_gs_ht = min(_gs_ht, HT_GS_MAX)
				rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))

				## preset the desired goal speeds BEFORE sending the goal positions
				_pub_cmd = ""
				_pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
				_pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
				## NOTE: pubTo_maki_command will automatically add TERM_CHAR_SEND postfix

				## publish and give time for the command to propogate to the servo motors
				engagementStartleGame.pubTo_maki_command( self, str(_pub_cmd), cmd_prop=True )

				## set servo control command to set goal positions
				_pub_cmd = _startle_gp_cmd

				_start_time_startle = rospy.get_time()
				try:
					## Maki-ro open eyes wide
					## and "jerks" head back
					engagementStartleGame.pubTo_maki_command( self, _pub_cmd )
					## NOTE: publish and give time for the command to propogate to the servo motors,
					## but DO NOT MONITOR (excess overhead of minimum 200ms, which is greater
					## than _duration_startle and will cause delay)
					#engagementStartleGame.monitorMoveToGP( self, _pub_cmd, ll_gp=LL_STARTLE, ht_gp=HT_STARTLE)
				except rospy.exceptions.ROSException as e1:
					rospy.logerr( str(e1) )
				_duration = abs(_start_time_startle - rospy.get_time())
				rospy.logwarn( "Startle duration: " + str(_duration) + " seconds" )

				engagementStartleGame.__is_startled = True
				rospy.loginfo("Done: STARTLE ====")
			#end	if startle:

			if relax:
				rospy.loginfo("====> RELAX")
				rospy.loginfo( str(self.makiPP) )
				_first_pass = True
				_start_time_relax = rospy.get_time()

				## own version of monitorMoveToGP
				## adjusts speed based on difference
				## between current position and goal
				## position to stay within _duration_relax
				while relax and (not rospy.is_shutdown()):
					engagementStartleGame.requestFeedback( self, SC_GET_PP )
					rospy.loginfo( str(self.makiPP) )

					## computer difference between current and goal positions
					## TODO: do this calculation using map
					_distance_to_relax_ll = abs( self.makiPP["LL"] - LL_NEUTRAL )
					rospy.loginfo("_distance_to_relax_ll=" + str(_distance_to_relax_ll))
					_distance_to_relax_ht = abs( self.makiPP["HT"] - HT_NEUTRAL )
					rospy.loginfo("_distance_to_relax_ht=" + str(_distance_to_relax_ht))

					## adjust duration to stay within _duration_relax
					if _first_pass:
						rospy.logdebug("duration_relax = " + str(_duration_relax))
						_first_pass=False
						pass
					elif (_distance_to_relax_ll > DELTA_PP) or (_distance_to_relax_ht > DELTA_PP):
						_duration_relax = _duration_relax - _duration_relax_wait
						rospy.logdebug("duration_relax = " + str(_duration_relax))
					else:
						rospy.logdebug("close enough...done relax while loop")
						break
					if (_duration_relax <= 0):
						rospy.logdebug("negative time...done relax while loop")
						break

					## calculate new goal speeds
					_gs_ll = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_relax_ll, _duration_relax)
					rospy.loginfo("_gs_ll=" + str(_gs_ll))
					_gs_ll = max(_gs_ll, LL_GS_MIN)
					rospy.loginfo("adjusted _gs_ll=" + str(_gs_ll))
					_gs_ht = self.DC_helper.getGoalSpeed_ticks_durationMS( _distance_to_relax_ht, _duration_relax)
					rospy.loginfo("_gs_ht=" + str(_gs_ht))
					_gs_ht = max(_gs_ht, HT_GS_MIN)
					rospy.loginfo("adjusted _gs_ht=" + str(_gs_ht))
					#_duration_relax_wait = self.DC_helper.getTurnDurationMS_ticks_goalSpeed( _distance_to_relax, _gs_ll )
					#rospy.loginfo("waitMS = " + str(_duration_relax_wait))
			
					## generate servo control command to set new goal speeds
					_pub_cmd = ""
					_pub_cmd += "LL" + SC_SET_GS + str(_gs_ll)
					_pub_cmd += "HT" + SC_SET_GS + str(_gs_ht)
					engagementStartleGame.pubTo_maki_command( self, str(_pub_cmd) )

					## servo control command to set goal positions
					_pub_cmd = _relax_gp_cmd
		
					#_start_time_relax = rospy.get_time()
					try:
						## Maki-ro relaxes wide open eyes
						## and head comes back forward to neutral
						engagementStartleGame.pubTo_maki_command( self, _pub_cmd )
						self.SWW_WI.sleepWhileWaitingMS( _duration_relax_wait, end_early=False)
					except rospy.exceptions.ROSException as e2:
						rospy.logerr( str(e2) )
				#end	while relax and (not rospy.is_shutdown()):

				_duration = abs(_start_time_relax - rospy.get_time())
				rospy.logwarn( "Relax duration: " + str(_duration) + " seconds" )
				rospy.loginfo( str(self.makiPP) )

				engagementStartleGame.__is_startled = False
				rospy.loginfo("Done: RELAX ====")
			#end	if relax:

			_loop_count = _loop_count +1

			## debugging
			rospy.loginfo(".............P A U S E ...")
			self.SWW_WI.sleepWhileWaiting( 1 )	## 1 second
		# end	while not rospy.is_shutdown():

		_duration = abs(rospy.get_time() - _start_time)
		rospy.logdebug( "NUMBER OF STARTLE/RELAX MOVMENTS: " + str(_loop_count) )
		rospy.logdebug( "Duration: " + str(_duration) + " seconds" )
		return


	def startStartle( self, relax=False ):
		rospy.logdebug("startStartle(): BEGIN")
		## call base class' start function
		eyelidHeadTiltBaseBehavior.start(self)
		rospy.logdebug("startStartle(): After eyelidHeadTiltBaseBehavior.start()")
		engagementStartleGame.macroStartleRelax( self, startle=True, relax=relax )
		rospy.logdebug("startStartle(): END")
		return

	def stopStartle( self ):
		## shift into eyelid and headtilt neutral
		engagementStartleGame.macroStartleRelax( self, startle=False, relax=True )

		## call base class' stop function
		eyelidHeadTiltBaseBehavior.stop(self)
		return

	## DO NOT USE -- UNDER DEVELOPMENT -- NOT TESTED	
	def startStartleGame( self ):
		pass
		'''
		## call base class' start function
		eyelidHeadTiltBaseBehavior.start(self)

		engagementStartleGame.setEyelidNeutralPose( self, LL_OPEN_DEFAULT, monitor=True )

		engagementStartleGame.requestFeedback( self, SC_GET_PP )
		## set to startle game values for neutral eyelid
		engagementStartleGame.setEyelidRange( self, self.makiPP["LL"], ll_startle=LL_OPEN_MAX, ll_close=LL_CLOSE_MAX )

		self.next_startle_time = rospy.get_time()
		rospy.logdebug("startStartle(): next_startle_time=" + str(self.next_startle_time))

		engagementStartleGame.runGame( self )
		return
		'''

	## DO NOT USE -- UNDER DEVELOPMENT -- NOT TESTED	
	def stopStartleGame( self ):
		pass
		'''
		## call base class' stop function
		eyelidHeadTiltBaseBehavior.stop(self)

		## reset to single startle values from neutral eyelid
		engagementStartleGame.setEyelidRange( self, LL_OPEN_DEFAULT, ll_close=LL_CLOSE_MAX )
		self.next_startle_time = None
		return
		'''

	def setEyelidRange( self, ll, ll_delta=None, ll_close=None, ll_startle=None ):
		## call to base class' function; eyelidBaseBehavior in this case
		eyelidBaseBehavior.setEyelidRange( self, ll, ll_delta, ll_close )

		if ll_startle == None:	return

		## override base class's function
		self.ll_startle = ll_startle	## passed position

		## check range values
		if self.ll_startle > LL_OPEN_MAX:	self.ll_startle = LL_OPEN_MAX
		rospy.logdebug("Startle -- Eyelid range: (" + str(self.ll_close) + ", " + str(self.ll_open) + ", " + str(self.ll_startle) + ")")
		return

	def parse_maki_macro( self, msg ):
		rospy.loginfo( msg.data )

		if msg.data == "startle start":
			engagementStartleGame.startStartle( self )
		elif (msg.data == "startle stop") or (msg.data == "startle relax"):
			engagementStartleGame.stopStartle( self )
		elif (msg.data == "startleGame start"):
			engagementStartleGame.startStartleGame( self )
		elif (msg.data == "startleGame stop"):
			engagementStartleGame.stopStartleGame( self )
		elif (msg.data == "hideFromStartle"):
			engagementStartleGame.hideFromStartle( self )
		elif (msg.data == "unhideIntoStartle"):
			engagementStartleGame.unhideIntoStartle( self )
		else:
			pass

		return

if __name__ == '__main__':
        print "__main__: BEGIN"
	E = engagementStartleGame( True, None )

	rospy.Subscriber( "/maki_macro", String, E.parse_maki_macro )
        rospy.logdebug( "now subscribed to /maki_macro" )

	rospy.spin()   ## keeps python from exiting until this node is stopped

        print "__main__: END"


