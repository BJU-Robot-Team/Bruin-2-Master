#!/usr/bin/env python

# Bruin-2 Compass Node v2.0
# Created by Nathan Collins
# Reads and parses the NMEA 0183 data coming from the digital compass
# The compass (model OS5000-US by DigitalOcean) is assigned to /dev/compass and returns data at up to 40Hz in NMEA 0183 format
# This node publishes the compass data on the CompassData topic using the CompassDataMsg message

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
        rospy.logerr("Invalid compass string: " + msg)
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
