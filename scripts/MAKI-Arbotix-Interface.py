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

resetPositions = ""	## init as empty string; dynamically populated
resetSpeeds = ""	## init as empty string; dynamically populated

## servo control infix for type of feedback
FEEDBACK_SC = [ "MX", "MN", "PP", "PS", "PT", "PL", "ER", "DP", "DS" ]
FEEDBACK_TOPIC = [ "maki_feedback_max_pos",
			"maki_feedback_min_pos",
			"maki_feedback_pres_pos",
			"maki_feedback_pres_speed",
			"maki_feedback_pres_temp",
			"maki_feedback_pres_load",
			"maki_feedback_error",
			"maki_feedback_default_pos",
			"maki_feedback_default_speed" ]
FEEDBACK_PUB_DICT = { }	## init as empty dictionary; dynamically populated

## NOTE: These globals are #define at the top of the Arduino servo driver
## MAKIv1_4_servo_controller_LITE.ino
INVALID_INT = 9999
SERVOCOUNT = 6
DELIMITER_RECV = ':'	## syntax for FEEDBACK delimiter
TERM_CHAR_RECV = ';'	## syntax for end of FEEDBACK
TERM_CHAR_SEND = 'Z'	## syntax for end of servo command

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
	# Publisher setup
	initPubFeedback()
	# Setup regular expression templates for parsing feedback messages
	initFeedbackFormat()

	# And now... go!
	rospy.spin()	

## ------------------------------
def recvFromArduino():
	global maki_serial
	global TERM_CHAR_RECV
	print "recvFromArduino: BEGIN"

	_recv_msg = ''
	_RECEIVING = True
	while _RECEIVING:
		if maki_serial.inWaiting() > 0:
			_m_char = maki_serial.read(1)	# read 1 byte from Arduino
			_recv_msg += _m_char
			if _m_char==TERM_CHAR_RECV:
				_RECEIVING = False

	if _recv_msg != '' and maki_serial.isOpen():
		maki_serial.flushInput();	# clear the input buffer

	print "recvFromArduino: END"
	return _recv_msg

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

## ------------------------------
##	ros publishers are stored as the contents of a dictionary
##	keys come from the FEEDBACK_SC list; servo control infix for type of feedback
## ------------------------------
def initPubFeedback():
	global FEEDBACK_PUB_DICT, FEEDBACK_SC, FEEDBACK_TOPIC
	print "setup rostopic publishers to give feedback"
			
	_tmp_dict = dict( zip(FEEDBACK_SC, FEEDBACK_TOPIC) )
	print _tmp_dict
	FEEDBACK_PUB_DICT = { }		# init as empty dictionary
	for _sc_dict_key, _feedbackTopic in _tmp_dict.iteritems():
		_pub = rospy.Publisher(_feedbackTopic, String, queue_size = 26)
		FEEDBACK_PUB_DICT[_sc_dict_key] = _pub
	return

def initFeedbackFormat():
	global TERM_CHAR_SEND, TERM_CHAR_RECV, DELIMITER_RECV
	global FEEDBACK_REQ_TEMPLATE, FEEDBACK_RESP_TEMPLATE

	_feedback_request_format = "\AF"	## F is the FEEDBACK request prefix
	_feedback_request_format += "([A-Z]{2})"	
	_feedback_request_format += TERM_CHAR_SEND + "\Z"	## ends in Z (capital zed)
	FEEDBACK_REQ_TEMPLATE = re.compile(_feedback_request_format)

	_feedback_response_format = "\A([A-Z]{2})"	## 2 alphabetic char prefix
	_feedback_response_format += "(([0-9]+" + DELIMITER_RECV + "){" + str(SERVOCOUNT-1) + "}[0-9]+)"
	_feedback_response_format += TERM_CHAR_RECV + "\Z"	## ends in ;
	FEEDBACK_RESP_TEMPLATE = re.compile(_feedback_response_format)
	return

def feedback(feedbackString):
	_feedback_type = requestFeedback(feedbackString)
	if _feedback_type != '':
		publishFeedback(_feedback_type)
	return

def requestFeedback(feedbackString):
	global FEEDBACK_REQ_TEMPLATE

	print "about to request feedback"
	_tmp = FEEDBACK_REQ_TEMPLATE.search(feedbackString)
	## Yes, feedbackString has the expected format
	if _tmp != None:
		_feedback_type = _tmp.group(1)
		maki_serial.write(feedbackString)
		return _feedback_type
	else:
		print "INVALID SYNTAX for feedback request: " + feedbackString
		return ''
	return

def publishFeedback(feedbackType):
	## TODO: currently feedbackType is ignored

	global FEEDBACK_RESP_TEMPLATE
	global FEEDBACK_PUB_DICT

	_recv_msg = recvFromArduino()
	_tmp = FEEDBACK_RESP_TEMPLATE.search(_recv_msg)
	if _tmp != None:
		_prefix = _tmp.group(1)
		_feedback_values = _tmp.group(2)
		print "Validated: prefix='" + _prefix + "' and feedback_values='" + _feedback_values + "'"
		if FEEDBACK_PUB_DICT.has_key(_prefix):
			FEEDBACK_PUB_DICT[_prefix].publish(_recv_msg)
			print "published std_msgs/String '" + _recv_msg + "' on rostopic " + "FOO" 

	print "feedback: " + _recv_msg
	return

## ------------------------------
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
		
			
## ------------------------------
def signal_handler(signal, frame):
	global ALIVE

	if maki_serial.isOpen():
		print 'Closing the Arduino port...'
		maki_serial.close()
	ALIVE = False
	sleep(1)	# give a chance for everything else to shutdown nicely
	exit

## ------------------------------
if __name__ == '__main__':
	global ALIVE

	# allow closing the program using CTRL+C
	signal.signal(signal.SIGINT, signal_handler)

	ALIVE = True

	main()
