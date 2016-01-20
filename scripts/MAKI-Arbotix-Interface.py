#! /usr/bin/env python

import rospy
import re
from std_msgs.msg import String
import serial
from serial.tools import list_ports

from time import sleep
import signal

# --------------------------------------------------------------------	
maki_port = "/dev/ttyUSB0" # default port for the MAKI Arbotix Board
maki_serial = serial.Serial(maki_port, 9600, timeout=None) # no timeout  timeout=None
print maki_serial

resetPositions = ""
resetSpeeds = ""
# --------------------------------------------------------------------
def main():
	global ALIVE

	# ENSURE SERIAL COMMUNICATION WITH THE ROBOT
	if maki_serial.isOpen():
		maki_serial.flushInput();	# clear the input buffer
		maki_serial.flushOutput();	# clear the output buffer
	else:
		print "ERROR: Unable to connect to MAKI on " + maki_port
		exit	# Goodbye

	i = 0
	n = maki_serial.inWaiting()
	print str(i) + ") maki_serial.inWaiting() = " + str(n)
	while n <= 0:
		if not ALIVE:
			print "THE END"
			return
		sleep(1)	# 1s
		i += 1
		try:
			if maki_serial.isOpen():
				n = maki_serial.inWaiting()
		except ValueError:
			print "VALUE ERROR"
			return
		print str(i) + ") maki_serial.inWaiting() = " + str(n)
	maki_serial.flushInput();	# clear the input buffer; we don't actually care about the contents

	# Reset MAKI to default position and speed
	defReset()
	print "resetPositions: " + resetPositions
	print "resetSpeeds: " + resetSpeeds

	# Initialize ROS node
	rospy.init_node('maki_listener')
	# Subscribe to the maki_command stream
	rospy.Subscriber("maki_command", String, sendToMAKI)
	rospy.spin()	

def sendToMAKI (message): 
	global maki_serial
	maki_serial.flushOutput()
	print "message received"
	
	#handle feedback commands
	feedback_strings = {'FMXZ', 'FMNZ', 'FPPZ', 'FPSZ', 'FPTZ', 'FPLZ', 'FERZ', 'FDPZ', 'FDSZ'}
	if message.data in feedback_strings:
		feedback(message.data);
		return

	#reset positions (need to query them on start)
	if (message.data == "reset"):
		print "resetting speeds: " + resetSpeeds
		maki_serial.write(resetSpeeds)
		print "speeds reset"

		print "resetting positions: " + resetPositions
		maki_serial.write(resetPositions)
		print "positions reset"
		
	#check for valid command formatting (regex)
	else:
		regex2 = re.compile('(((((HP)|(HT)|(LL)|(EP)|(ET))((GP)|(GS)))|(IPT))\d{3,5})+Z')
		match = regex2.match(message.data)
		if (match):
			print "sending command: " + message.data
			maki_serial.write(message.data)
			print "command sent"
		else:
			print "invalid format" 
			return
	#if position command, ask for present position
	#if 'GP' in message.data:
		#feedback("FPPZ")

def feedback(feedbackString):
	print "about to give feedback"
	startLetter = ''
	feedbackTopic = ''
	if (feedbackString == "FMXZ"):
		startLetter = 'M'
		feedbackTopic = 'maki_max_feedback'
	if (feedbackString == "FMNZ"):
		startLetter = 'M'
		feedbackTopic = 'maki_min_feedback'
	if (feedbackString == "FPPZ"):
		startLetter = 'P'
		feedbackTopic = 'maki_position_feedback'
	if (feedbackString == "FPSZ"):
		startLetter = 'P'
		feedbackTopic = 'maki_speed_feedback'
	if (feedbackString == "FPTZ"):
		startLetter = 'M'
		feedbackTopic = 'maki_temp_feedback'
	if (feedbackString == "FPLZ"):
		startLetter = 'P'
		feedbackTopic = 'maki_load_feedback'
	if (feedbackString == "FERZ"):
		startLetter = 'E'
		feedbackTopic = 'maki_error_feedback'
	if (feedbackString == "FDPZ"):
		startLetter = 'D'
		feedbackTopic = 'maki_default_position'
	if (feedbackString == "FDSZ"):
		startLetter = 'D'
		feedbackTopic = 'maki_default_speed'

	maki_serial.write(feedbackString)
	line = ""
	c = maki_serial.read()
	while (c != startLetter):
		c = maki_serial.read()
	while (c != ';'):
		line += c
		c = maki_serial.read()
	print "feedback: " + line
	pub = rospy.Publisher(feedbackTopic, String, queue_size = 26)
	pub.publish(line)	
	return

def defReset():
	print "defining reset strings"
	maki_serial.write("FDPZ")
	resetCommand = ""
	c = maki_serial.read()
	while (c != 'D'): # waits for feedback
		c = maki_serial.read()
	while (c != ':'): #eliminates DP and first value
		c = maki_serial.read()
	resetCommand += token("LLGP")
	resetCommand += token("EPGP")
	resetCommand += token("ETGP")
	resetCommand += token("HTGP")
	resetCommand += token("HPGP")
	resetCommand += 'Z'
	
	global resetPositions
	resetPositions = resetCommand
	
	maki_serial.write("FDSZ")
	resetCommand = ""
	c = maki_serial.read()
	while (c != 'D'): # waits for feedback
		c = maki_serial.read()
	while (c != ':'): #eliminates DS and first value
		c = maki_serial.read()
	resetCommand += token("LLGS")
	resetCommand += token("EPGS")
	resetCommand += token("ETGS")
	resetCommand += token("HTGS")
	resetCommand += token("HPGS")
	resetCommand += 'Z'
	
	global resetSpeeds
	resetSpeeds = resetCommand
		

def token(header):
	subCommand = header
	c = maki_serial.read()
	while (c != ':' and c != ';'):
		subCommand += c
		c = maki_serial.read()
	return subCommand		
		
			
# --------------------------------------------------------------------
def signal_handler(signal, frame):
	global ALIVE

	if maki_serial.isOpen():
		print 'Closing the Arduino port...'
		maki_serial.close()
	ALIVE = False
	sleep(1)	# give a chance for everything else to shutdown nicely
	exit

if __name__ == '__main__':
	global ALIVE

	# allow closing the program using CTRL+C
	signal.signal(signal.SIGINT, signal_handler)

	ALIVE = True

	main()
