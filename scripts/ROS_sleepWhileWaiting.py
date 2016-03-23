#! /usr/bin/env python

import rospy

import math
from time import sleep
from timeit import default_timer as timer	## wall clock. Unix 1/100 second granularity

from maki_robot_common import *


class ROS_sleepWhileWaiting_withInterupt:
	def __init__(self, verbose_debug=False ):
		self.PIC_INTERUPT = True
		self.VERBOSE_DEBUG = verbose_debug


	def abort( self ):
		self.PIC_INTERUPT = True


	def sleepWhileWaitingMS( self, ms_sleep_time, increment=0.25, end_early=True):
		## convert from milliseconds to sleep in seconds

		_new_ms_sleep_time = float(ms_sleep_time)/1000.0 
		if end_early:
			_new_ms_sleep_time = _new_ms_sleep_time - float(increment)
	
		#print "ms_sleep_time = " + str( ms_sleep_time )
		#print "increment = " + str( increment )
		#print "new_ns_sleep_time = " + str( _new_ms_sleep_time )
		ROS_sleepWhileWaiting_withInterupt.sleepWhileWaiting( self, _new_ms_sleep_time, increment )


	def sleepWhileWaiting( self, sleep_time, increment=1 ):
		self.PIC_INTERUPT = False	## reset

		if self.VERBOSE_DEBUG: rospy.logdebug( "BEGIN: sleepWhileWaiting for " + str(sleep_time) + " seconds" )
		increment = max(0.01, increment)	## previous max values: 1.0, 0.25	## in seconds
		_start_sleep_time = timer()
		sleep(increment)	# emulate a do-while
		while (not rospy.is_shutdown() and
			not self.PIC_INTERUPT and
			((timer() - _start_sleep_time) < sleep_time) ):
			sleep(increment)
			#if self.VERBOSE_DEBUG:	rospy.logdebug(".")

		if self.VERBOSE_DEBUG: rospy.logdebug( "DONE: sleepWhileWaiting for " + str(sleep_time) + " seconds" )

