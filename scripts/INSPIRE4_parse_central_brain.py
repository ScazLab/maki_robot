#! /usr/bin/env python

import rospy
import re
import csv
from std_msgs.msg import String



#--------------------
class messageCoordinater():
    def __init__(self, behavior_file, ros_topic="INSERT_TOPIC_HERE"):

        rospy.init_node('messageCoordinater',anonymous=False)

        #self.ros_pub = rospy.Publisher("test_topic",String,queue_size=10)
        self.behaviorDict = self.createBehaviorDict(behavior_file)


    def parseVHMessage(self, msg):
        rospy.logdebug("parseMessage(): BEGIN")
        rospy.logdebug("received: " + str(msg))
        # Turns message into array for easy manipulation
        _data = str(msg.data).strip().split()
        _data = [d.strip() for d in _data]
        rospy.loginfo(_data)

        ## These will help us check that the message is well formed
        # Right now there is only one command, but might add more later
        _command_list= ["performance"]
        _agent_list = ["Maki-ro", "Maki"]

        #[Jake] For October, all valid messages will adhere to this length
        try:
            _command = _data[0]
            _msg_id = _data[1]
            _agent = _data[2]
            _behavior ,_pub_topic = self.behaviorDict[_data[3]]
        except IndexError:
            rospy.logerr("parseMessage(): ERROR: Message is not well formed!")
            return
        except KeyError:
            rospy.logerr("parseMessage(): ERROR:{} is not a valid behavior!".format(_data[3]))
            return

        if _command in _command_list and _agent in _agent_list and len(_data)==4: 
            rospy.loginfo("{} is well formed!".format(_data))
            ros_pub = rospy.Publisher(_pub_topic,String, queue_size=10)
            rospy.loginfo("Publishing: {} to Maki on topic: {}".format(_behavior,_pub_topic))
            ros_pub.publish(_behavior+" "+_msg_id)
            rospy.Rate(1).sleep()
        else:
            rospy.loginfo("ERROR: {} is not well formed".format(msg.data))
            rospy.logerr("parseMessage(): ERROR: Message is not well formed!")

    # returns column col of the behavior csv as an generator generating an array
    def getCol(self,filename, col):
        for row in csv.reader(open(filename), delimiter=','):
            yield row[col]

    # creates a dict with woz.id's as keys and Answers as associated vals.
    def createBehaviorDict(self, behavior_file):
        _answer_col = self.getCol(behavior_file, 0)
        _woz_id_col = self.getCol(behavior_file, 1)
        _ros_topic_col = self.getCol(behavior_file, 2)
        return dict(zip(_woz_id_col, zip(_answer_col, _ros_topic_col)))

if __name__ == '__main__':
    try:
        vhmsg_topic="from_central_brain"

        infile = '/home/jakebrawer/Desktop/maki_stuff/catkin_ws/src/ros2vhmsg/WoZ/freeplay-annex-behaviors.csv' 
        m = messageCoordinater(infile)
        while not rospy.is_shutdown():
            #rospy.Subscriber(ros_topic,String,m.parseMessage)
            rospy.Subscriber(vhmsg_topic,String,m.parseVHMessage)
            rospy.Rate(1).sleep()
    except rospy.ROSInterruptException:
        pass

        

