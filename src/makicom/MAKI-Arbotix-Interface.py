#! /usr/bin/env python
#RUN AS:	rosrun maki_robot MAKI-Arbotix-Interface.py <PORT, default=USB0>

import rospy
import re
import serial
import sys
import random
from std_msgs.msg import String

from makicom.msg import MakiCommand, MakiFeedback
from makicom import INVALID_INT, EYELID_RIGHT, EYELID_LEFT, EYE_PAN, EYE_TILT, HEAD_TILT, HEAD_PAN
from makicom import MAX_POS, MIN_POS, PRESENT_POS, GOAL_POS, PRESENT_SPEED, GOAL_SPEED, PRESENT_TEMP, PRESENT_LOAD, TORQUE_MAX, TORQUE_LIM, TORQUE_ENABLE, ERROR, DEFAULT_POS, DEFAULT_SPEED
from makicom import feedbackCommand, setCommand, newMakiFeedback, getFeedback, updateFeeback

SERVO_COUNT = 6
BAUD_RATE = 115200
makiSerial = None
experimentInfo = None

simulating = False # simulate the arbotix board with fake data -- no need for USB connection

feedbackDataRegex = re.compile('([A-Z]{2})(([0-9]+:){%d}[0-9]+);' % (SERVO_COUNT - 1))
receiveBuffer = ''
makiFeedbackValues = newMakiFeedback()

def receiveFromMaki():
    while makiSerial.inWaiting() > 0:
        newChar = makiSerial.read(1)
        if newChar in ' \r\n':
            receiveBuffer = ''
        elif newChar == ';':
            receiveBuffer += newChar
            match = feedbackDataRegex.search(receiveBuffer)
            if match:
                feedbackType = match.group(1)
                values = match.group(2).split(':')
                updateFeedback(feedbackType, values, makiFeedbackValues)
                feedbackPublisher.publish(makiFeedbackValues)
        else
            if len(receiveBuffer) > 100:
                rospy.logwarn('Buffer overflow from Arduino, threw out: ' + receiveBuffer)
                receiveBuffer = ''
            receiveBuffer += newChar

def addSimulatedFeedback(feedbackType):
    valueMax = 1023
    if feedbackType == PRESENT_TEMP:
        valueMax = 65
    elif feedbackType == TORQUE_ENABLE:
        valueMax = 1
    values = [random.randint(0,valueMax) for i in xrange(SERVO_COUNT)]
    rospy.logwarn('Generated simulated values for ' + feedbackType + ' of ' + str(values))
    updateFeedback(feedbackType, values, makiFeedbackValues)
            
def sendToMaki(makiCmd):
    commandStr = ''
    if makiCmd.type == 'reset':
        commandStr = 'RESETZ\n'
    elif makiCmd.type == 'feedback':
        commandStr = 'F' + makiCmd.feedbackType + 'Z\n'
    elif makiCmd.type == 'set':
        commandParts = [s.servo + s.setType + s.value for s in makiCmd.settings]
        commandStr = ''.join(commandParts) + ('IPT' + makiCmd.movementTimeMs if movementTimeMs else '') + 'Z\n'
    else:
        except ValueError('Invalid maki command type: ' + makiCmd.type)
        
    if simulating:
        rospy.logdebug('Simulating sending command to Maki: ' + commandStr)
        if makiCmd.type == 'feedback':
            addSimulatedFeedback(makiCmd.feedbackType)
    else:
        rospy.logdebug('Sending command to Maki: ' + commandStr)
        try:
            makiSerial.write(commandStr)
        except serial.SerialException as e:
            rospy.logerr('Trouble with connection to Maki: ' + str(e))
            
def setupSerial(makiUsb):
    makiDevicePath = '/dev/tty' + makiUsb
    try:
        makiSerial = serial.Serial()
        makiSerial.baudrate = BAUD_RATE
        makiSerial.timeout = 1000 # 1 second
        makiSerial.port = makiDevicePath
        makiSerial.open()
        
        # confirm communication
        sendToMaki(feedbackCommand(PRESENT_POS))
        while makiSerial.read() != ';':
            pass
        rospy.loginfo('SUCCESS (1/2): Opened serial connection to MAKI on ' + makiDevicePath + ' and established communication')
    except serial.SerialException as err:
        rospy.logerr('Unable to connect to Maki Arbotix: ' + str(err))
        sys.exit();

def makiArbotix(makiUsb):
    rospy.init_node('maki_arbotix_interface')
    feedbackPublisher = rospy.Publisher('maki_feedback', MakiFeedback, queue_size = 26, latch = LATCH) # if LATCH==True, any new subscribers will see the most recent message 
    experimentInfo = rospy.Publisher("experiment_info", String, queue_size = 10)
    rospy.Subscriber('maki_command', MakiCommand, sendToMaki)

    if not simulating:
        setupSerial(makiUsb)
    
    sendToMaki(feedbackCommand(PRESENT_POS))
	rospy.loginfo('SUCCESS (2/2): Robot successfully connected.')
	experimentInfo.publish('Robot is ready.')
    
    # effectively disable the underpowered head tilt motor (though TORQUE_ENABLE would make more sense)
    sendToMaki(setCommand(HEAD_TILT, TORQUE_LIM, 0))
    sendToMaki(feedbackCommand(TORQUE_LIM)) # and confirm... necessary?

    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        if not simulating:
            receiveFromMaki()
        rate.sleep()

if __name__ == '__main__':
    args = rospy.myargv()
    if len(args) > 2:
        rospy.loginfo('Usage: rosrun maki_robot MAKI-Arbotix-Interface.py <PORT, default=USB0>')
        sys.exit()
        
    makiUsb = 'USB0'
    if len(args) == 2:
        makiUsb = args[1]

    try:
        makiArbotix(makiUsb)
    except rospy.ROSInterruptException:
        pass