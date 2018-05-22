#!/usr/bin/env python

from bruin2_msgs.msg import CompassDataMsg
import rospy
import re
import os
import serial

rospy.init_node('compass')
rospy.loginfo("Bruin-2 Compass Driver V2.0 Starting")

fakeData = "$C212.4P2.5R-14.0T28.4Mx107977.90My-79422.00Mz173.27Ax0.045Ay0.245Az0.977*3A"
pub = rospy.Publisher("CompassData", CompassDataMsg, queue_size=10)

msg_re = re.compile(r'\$C(-?[0-9]*\.[0-9]*)P(-?[0-9]*\.[0-9]*)R(-?[0-9]*\.[0-9]*)T(-?[0-9]*\.[0-9]*)M')

def sendMsg(msg):
    match = msg_re.match(msg)
    if not match:
        rospy.logerror("Invalid compass string: " + msg)
        return
    
    heading     = float(match.group(1))
    pitch       = float(match.group(2))
    roll        = float(match.group(3))
    temperature = float(match.group(4))
    pub.publish(heading, pitch, roll, temperature)

if "ROS_DEBUG" not in os.environ:
    compass = serial.Serial('/dev/compass', 19200, timeout=1)
    while True:
        for msg in compass:
            sendMsg(msg)
else:
    def sendFakeMsg(event):
        sendMsg(fakeData)
    
    # publish a message at 40Hz
    rospy.Timer(rospy.Duration(0.025), sendFakeMsg)
    rospy.spin()
