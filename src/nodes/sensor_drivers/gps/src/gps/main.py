#!/usr/bin/env python

from bruin2_msgs.msg import GPSDataMsg
import rospy
import pynmea2 # gps parser
import os
import serial

rospy.init_node('gps')
rospy.loginfo("Bruin-2 GPS Driver V2.0 Starting")

fakeData = "$GPGGA,193956.000,3452.4626,N,08221.7059,W,0,00,,329.9,M,-32.1,M,,0000*48"
pub = rospy.Publisher("GpsData", GPSDataMsg, queue_size=10)

def sendMsg(msg, valid = True):
    hdop      = msg.hdop if "hdop" in dir(msg) else 0.0
    pdop      = msg.pdop if "pdop" in dir(msg) else 0.0
    vdop      = msg.vdop if "vdop" in dir(msg) else 0.0
    velocity  = msg.spd_over_grnd if "spd_over_grnd" in dir(msg) else 0.0
    longitude = msg.longitude
    latitude  = msg.latitude
    altitude  = float(msg.altitude)
    valid     = True
    pub.publish(hdop, pdop, vdop, velocity, longitude, latitude, altitude, valid)

if "ROS_DEBUG" not in os.environ:
    gps = pynmea2.NMEAStreamReader(serial.Serial('/dev/gps', 4800, timeout=1))
    while True:
        try:
            for msg in gps:
                sendMsg(msg)
        except:
            sendMsg(pynmea2.NMEASentence.parse(fakeData), False)
else:
    def sendFakeMsg(event):
        msg = pynmea2.NMEASentence.parse(fakeData)
        sendMsg(msg)
    
    # publish a message every second
    rospy.Timer(rospy.Duration(1), sendFakeMsg)
    rospy.spin()
