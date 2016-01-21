#! /usr/bin/env python

#RUN AS:	python MAKI-Arbotix-Interface.py <PORT, default=USB0>

import rospy
import re
from std_msgs.msg import String
import serial
from serial.tools import list_ports

from time import sleep
import signal

# --------------------------------------------------------------------	
## ---- USER DEFINED GLOBALS ----

VERBOSE_DEBUG = False
TTY_PORT = "USB0"	## default port for the MAKI Arbotix-M board

## ---- CONSTANTS ----

## NOTE: These globals are #define at the top of the Arduino servo driver
## MAKIv1_4_servo_controller_LITE.ino
INVALID_INT = 9999
DELIMITER_RECV = ':'	## syntax for FEEDBACK delimiter
TERM_CHAR_RECV = ';'	## syntax for end of FEEDBACK
TERM_CHAR_SEND = 'Z'	## syntax for end of servo command
SC_SET_GP = "GP"        ## servo command syntax for setting a specified servo's GOAL POSITION
SC_SET_GS = "GS"        ## servo command syntax for setting a specified servo's GOAL SPEED
SC_SET_IPT = "IPT"      ## servo command syntax for setting the INTERPOLATION POSE TIME
SC_FEEDBACK = "F"       ## servo command syntax for FEEDBACK or status of all servos
SC_GET_MX = "MX"        ## servo command syntax for FEEDBACK of all servo MAXIMUM POSITION
SC_GET_MN = "MN"        ## servo command syntax for FEEDBACK of all servo MINIMUM POSITION
SC_GET_PP = "PP"        ## servo command syntax for feedback with PRESENT POSITION
SC_GET_PS = "PS"        ## servo command syntax for feedback with PRESENT SPEED
SC_GET_GS = "GS"        ## servo command syntax for feedback with GOAL SPEED
SC_GET_PT = "PT"        ## servo command syntax for feedback with PRESENT TEMPERATURE (in Celsius)
SC_GET_PL = "PL"        ## servo command syntax for feedback with PRESENT LOAD
SC_GET_ER = "ER"        ## servo command syntax for feedback with error returned from AX_ALARM_LED
SC_GET_DP = "DP"	## servo command syntax for default positions
SC_GET_DS = "DS"	## servo command syntax for default speed
BAUD_RATE = 9600

## from MAKIv14.h
SERVOCOUNT = 6  ## MAKIv1.4 has 6 servos
LR = 1  ## EYELID_RIGHT
LL = 2  ## EYELID_LEFT
EP = 3  ## EYE_PAN
ET = 4  ## EYE_TILT
HT = 5  ## HEAD_TILT
HP = 6  ## HEAD_PAN

## servo control infix for type of feedback
FEEDBACK_SC = [ SC_GET_MX, SC_GET_MN, SC_GET_PP, SC_GET_PS, SC_GET_PT, SC_GET_PL, SC_GET_ER, SC_GET_DP, SC_GET_DS ]
FEEDBACK_TOPIC = [ "maki_feedback_max_pos",
			"maki_feedback_min_pos",
			"maki_feedback_pres_pos",
			"maki_feedback_pres_speed",
			"maki_feedback_pres_temp",
			"maki_feedback_pres_load",
			"maki_feedback_error",
			"maki_feedback_default_pos",
			"maki_feedback_default_speed" ]

## ---- DYNAMIC GLOBALS ---- modified programatically ----
ALIVE = False
maki_serial = "" 	## init as empty string
feedback_req_template = ""	## init as empty string; dynamically populated as compiled regular expression
feedback_resp_template = ""	## init as empty string; dynamically populated as compiled regular expression
feedback_pub_dict = { }	## init as empty dictionary; dynamically populated
resetPositions = ""	## init as empty string; dynamically populated
resetSpeeds = ""	## init as empty string; dynamically populated

# --------------------------------------------------------------------
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
	global feedback_pub_dict, FEEDBACK_SC, FEEDBACK_TOPIC
	print "setup rostopic publishers to give feedback"
			
	_tmp_dict = dict( zip(FEEDBACK_SC, FEEDBACK_TOPIC) )
	print _tmp_dict
	feedback_pub_dict = { }		# init as empty dictionary
	for _sc_dict_key, _feedbackTopic in _tmp_dict.iteritems():
		_pub = rospy.Publisher(_feedbackTopic, String, queue_size = 26)
		feedback_pub_dict[_sc_dict_key] = _pub
	return

def initFeedbackFormat():
	global TERM_CHAR_SEND, TERM_CHAR_RECV, DELIMITER_RECV
	global feedback_req_template, feedback_resp_template

	_feedback_request_format = "\A" + str(SC_FEEDBACK)	## F is the FEEDBACK request prefix
	_feedback_request_format += "([A-Z]{2})"	
	_feedback_request_format += str(TERM_CHAR_SEND) + "\Z"	## ends in Z (capital zed)
	feedback_req_template = re.compile(_feedback_request_format)

	_feedback_response_format = "\A([A-Z]{2})"	## 2 alphabetic char prefix
	_feedback_response_format += "(([0-9]+" + str(DELIMITER_RECV) + "){" + str(SERVOCOUNT-1) + "}[0-9]+)"
	_feedback_response_format += str(TERM_CHAR_RECV) + "\Z"	## ends in ;
	feedback_resp_template = re.compile(_feedback_response_format)
	return

def feedback(feedbackString):
	_feedback_type = requestFeedback(feedbackString)
	if _feedback_type != '':
		publishFeedback(_feedback_type)
	return

def requestFeedback(feedbackString):
	global feedback_req_template

	print "about to request feedback"
	_tmp = feedback_req_template.search(feedbackString)
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

	global feedback_resp_template
	global feedback_pub_dict

	_recv_msg = recvFromArduino()
	_tmp = feedback_resp_template.search(_recv_msg)
	if _tmp != None:
		_prefix = _tmp.group(1)
		_feedback_values = _tmp.group(2)
		print "Validated: prefix='" + _prefix + "' and feedback_values='" + _feedback_values + "'"
		if feedback_pub_dict.has_key(_prefix):
			feedback_pub_dict[_prefix].publish(_recv_msg)
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
	global maki_serial

	if maki_serial.isOpen():
		print 'Closing the Arduino port...'
		maki_serial.close()
	ALIVE = False
	sleep(1)	# give a chance for everything else to shutdown nicely
	exit

## ------------------------------
if __name__ == '__main__':

	global VERBOSE_DEBUG
	global ALIVE
	global TTY_PORT, BAUD_RATE, maki_serial

	## ------------------------------
	## BEGIN INITIALIZATION
	## ------------------------------
	## STEP 0: INIT GLOBAL VARIABLES
	ALIVE = True

	## STEP 1: SIGNAL HANDLER
	# allow closing the program using CTRL+C
	signal.signal(signal.SIGINT, signal_handler)

	## STEP 2: ROS SETUP
	# Initialize ROS node
	rospy.init_node('maki_listener')
	# Subscribe to the maki_command stream
	rospy.Subscriber("maki_command", String, sendToMAKI)
	# Publisher setup
	initPubFeedback()
	# Setup regular expression templates for parsing feedback messages
	initFeedbackFormat()

	## STEP 3: ESTABLISH SERIAL COMMUNICATION WITH THE ROBOT
	## STEP 3A: INSTANTIATE THE CONNECTION
	_maki_port = "/dev/tty" + str(TTY_PORT) # default port for the MAKI Arbotix Board
	maki_serial = serial.Serial(_maki_port, int(BAUD_RATE), timeout=None) # no timeout  timeout=None
	print maki_serial

	## STEP 3B: ENSURE SERIAL COMMUNICATION WITH THE ROBOT
	if maki_serial.isOpen():
		maki_serial.flushInput();	# clear the input buffer
		maki_serial.flushOutput();	# clear the output buffer
	else:
		print "ERROR: Unable to connect to MAKI on " + _maki_port
		print "Exiting..."
		exit	# Goodbye

	## wait until Arbotix-M board transmits before continuing 
	## THIS IS BLOCKING
	i = 0
	n = maki_serial.inWaiting()
	print str(i) + ") maki_serial.inWaiting() = " + str(n)
	while n <= 0:
		if not ALIVE:
			print "THE END"
			exit
		sleep(1)	# 1s
		i += 1
		try:
			if maki_serial.isOpen():
				n = maki_serial.inWaiting()
		except ValueError:
			print "VALUE ERROR"
			exit
		print str(i) + ") maki_serial.inWaiting() = " + str(n)
	# clear the input buffer; we don't actually care about the contents
	maki_serial.flushInput();

	## STEP 4: INIT ROBOT STATE
	# Reset MAKI to default position and speed
	defReset()
	print "resetPositions: " + resetPositions
	print "resetSpeeds: " + resetSpeeds
	## ------------------------------
	## END OF INITIALIZATION
	## ------------------------------
	


	# And now... go!
	rospy.spin()	
	print "I am here now"
	print "Bye bye"

