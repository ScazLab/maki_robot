#! /usr/bin/env python
#RUN AS:	rosrun maki_robot MAKI-Arbotix-Interface.py <PORT, default=USB0>

import rospy
import re
from std_msgs.msg import String
import serial

import signal
import sys

import random

from maki_robot_common import *
from ROS_sleepWhileWaiting import ROS_sleepWhileWaiting_withInterrupt

BAUD_RATE = 115200
makiSerial = None
experimentInfo = None

def setupSerial(makiUsb):
    makiDevicePath = '/dev/tty' + makiUsb
    try:
        makiSerial = serial.Serial()
        makiSerial.baudrate = BAUD_RATE
        makiSerial.timeout = 1000 # 1 second
        makiSerial.port = makiDevicePath
        makiSerial.open()
        
        # confirm communication
        makiSerial.write('FPPZ')
        while makiSerial.read() != ';':
            pass
    except serial.SerialException as err:
        rospy.logerr('Unable to connect to Maki Arbotix: ' + err)
        sys.exit();

def makiArbotix(makiUsb):
    rospy.init_node('maki_arbotix_interface')
    experimentInfo = rospy.Publisher("experiment_info", String, queue_size = 10)
    rospy.Subscriber('maki_command', String, sendToMaki)

    setupSerial(makiUsb)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
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