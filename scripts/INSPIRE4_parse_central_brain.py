#! /usr/bin/env python

import rospy
import re
from std_msgs.msg import String



#--------------------
class messageCoordinater():
    def __init__(self, vhmsg_topic="freeplay_annex_command", ros_topic="INSERT_TOPIC_HERE"):
        self.vhmsg_sub = rospy.Subscriber(vhmsg_topic,String,self.parseMessage)
        self.ros_sub = rospy.Subscriber(ros_topic,String,self.parseMessage)

    def parseMessage(self, msg):
        rospy.logdebug("parseMessage(): BEGIN")
        rospy.logdebug("received: " + str(msg))
        _data = str(msg.data)
