#!/usr/bin/env python

# Bruin-2 Relay Board Node v2.0
# Created by Nathan Collins
# Recieves control messages on the RelayControl topic using the RelayCommandMsg message
# For certain messages, such as GPIO result, returns the status on the RelayData topic using the RelayDataMsg message

import rospy
from bruin2_msgs.msg import RelayCommandMsg, RelayDataMsg
import os
from relayController import RelayController, FakeRelayController

pub = rospy.Publisher("RelayData", RelayDataMsg, queue_size=10)

relayController = FakeRelayController()

def handleMsg(msg):
    rospy.logdebug("Receieved relay msg: {} {} {}".format(msg.device_type, msg.device_number, msg.command))

    state = ""
    if msg.device_type == "relay":
        if msg.command == "on":
            state = relayController.turnOnRelay(msg.device_number)
        if msg.command == "off":
            state = relayController.turnOffRelay(msg.device_number)
        if msg.command == "read":
            state = relayController.readRelay(msg.device_number)
    else:
        # GPIO pin
        state = relayController.readGPIO(msg.device_number)
    
    pub.publish(msg.device_type, msg.device_number, state)

rospy.init_node('relay_board')
rospy.loginfo("Bruin-2 Relay Driver V2.0 Starting")

isDebug = "ROS_DEBUG" in os.environ

if not isDebug:
    relayController = RelayController()

rospy.Subscriber("RelayControl", RelayCommandMsg, handleMsg)
# keep listening till we're told to shut down
rospy.spin()
