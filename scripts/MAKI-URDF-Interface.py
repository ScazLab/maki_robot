#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math

defVals = [512, 512, 512, 512, 500]
curVals = [512, 512, 512, 512, 500]
headers = ["HT", "HP", "EP", "ET", "LL"]

def listener():
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher("maki_command", String, queue_size = 26)
    pub.publish("reset")
    rospy.Subscriber("joint_states", JointState, sendCommand)
    rospy.spin()

def sendCommand(jointState):
    global curVals
    global defVals
    global headers
    global INIT
    global map_js_to_headers

    #rospy.logdebug( jointState )
    if INIT:
        ## Match headers with JointState
        map_js_to_headers = list()	## initialize to empty list
        for i in range(0, len(headers)):
            index = None
            for j in range(0, len(jointState.name)):
                if (headers[i] == "HT" and jointState.name[j] == "head_tilt"):
		    index = j
		    break
		elif (headers[i] == "HP" and jointState.name[j] == "head_pan"):
		    index = j
		    break
		elif (headers[i] == "EP" and jointState.name[j] == "eye_right_pan"):
		    index = j
		    break
		elif (headers[i] == "ET" and jointState.name[j] == "eye_right_tilt"):
		    index = j
		    break
		elif (headers[i] == "LL" and jointState.name[j] == "eyelid_left_joint"):
		    index = j
		    break
		else:
		    pass

	    if index != None:
		#rospy.logdebug( index )
		map_js_to_headers.append(index)
        INIT = False
    #rospy.logdebug( map_js_to_headers )

    command = ""
    comFlag = 0
    i = 0
    for index in map_js_to_headers:
        servoVal = int(defVals[i] - jointState.position[index]/0.00511326171)
        if (curVals[i] != servoVal):
            comFlag = 1
            command += headers[i] + "GP" + str(servoVal)
            curVals[i] = servoVal
        i += 1
    if (comFlag):
        pub = rospy.Publisher("maki_command", String, queue_size = 26)
        pub.publish(command + "Z")
        comFlag = 0
        rospy.logdebug( command )
        

if __name__ == '__main__':
    global INIT
    global map_js_to_headers
    INIT = True
    map_js_to_headers = list()	## initialize to empty list
    listener()
