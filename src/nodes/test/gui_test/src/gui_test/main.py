#!/usr/bin/env python

import rospy
import rospkg
import sys
import os
import time
from bruin2_msgs.msg import RelayDataMsg, RelayCommandMsg, GPSDataMsg, CompassDataMsg

from PyQt5 import QtCore, QtWidgets, QtGui

# create absolute paths to resources and code folders
resource_path = os.path.join(rospkg.RosPack().get_path('gui_test'), 'resource')
src_path = os.path.join(rospkg.RosPack().get_path('gui_test'), 'src', "gui_test", "gui")

# auto generate the files we are trying to import from
os.system("pyuic5 -o " + os.path.join(src_path, "gui_test.py") + " " + os.path.join(resource_path, "gui_test.ui") )

# import the information we need from those files
from gui.gui_test import *

# class that intializes the GUI and handles events for the Main Window
class MainForm(QtWidgets.QMainWindow):
    def relayCallBack(self, msg):
        """ Based on the message from the Relay controller, set the checkboxes appropriately """
        rospy.loginfo(msg)
        checkboxNum = msg.device_number
        state       = msg.state == "on"
        self.relayCheckboxes[checkboxNum].setChecked(state)

    def compassCallBack(self, msg):
        self.ui.compass_heading.setText(str(msg.heading))
        self.ui.compass_pitch.setText(str(msg.pitch))
        self.ui.compass_roll.setText(str(msg.roll))
        self.ui.compass_temp.setText(str(msg.temperature))
    
    def gpsCallBack(self, msg):
        self.ui.gps_hdop.setText(str(msg.hdop))
        self.ui.gps_pdop.setText(str(msg.pdop))
        self.ui.gps_vdop.setText(str(msg.vdop))
        self.ui.gps_vel.setText(str(msg.velocity))
        self.ui.gps_long.setText(str(msg.longitude))
        self.ui.gps_lat.setText(str(msg.latitude))
        self.ui.gps_alt.setText(str(msg.altitude))

    def relayButtonClicked(self, checkBoxNum, isChecked):
        self.pub.publish("relay", checkBoxNum, ["off", "on"][isChecked], 0)

    pub = rospy.Publisher("RelayControl", RelayCommandMsg)

    def __init__(self, parent = None):
        """Constructor for Mainwindow (possibly split into separate method?)"""
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # initialize the listeners to the various things we need to pull data from
        rospy.Subscriber("RelayData", RelayDataMsg, self.relayCallBack)
        rospy.Subscriber("CompassData", CompassDataMsg, self.compassCallBack)
        rospy.Subscriber("GpsData", GPSDataMsg, self.gpsCallBack)

        # Give the subscriber time to initialize
        time.sleep(0.1)

        # connect the signals of the button presses to their respective events
        self.relayCheckboxes = [
            self.ui.relay00,
            self.ui.relay01,
            self.ui.relay02,
            self.ui.relay03,
            self.ui.relay04,
            self.ui.relay05,
            self.ui.relay06,
            self.ui.relay07,
            self.ui.relay08,
            self.ui.relay09,
            self.ui.relay10,
            self.ui.relay11,
            self.ui.relay12,
            self.ui.relay13,
            self.ui.relay14,
            self.ui.relay15
        ]

        for i, checkbox in enumerate(self.relayCheckboxes):
            checkbox.clicked.connect((lambda x: lambda event: self.relayButtonClicked(x, event))(i))
        
        # Send a message to the relay controller to get current status of each checkbox
        for i in range(16):
            self.pub.publish("relay", i, "read", 0)
    
if __name__ == '__main__':
    """create a new window and display it until the user closes the window"""
    global mainwin

    rospy.init_node('gui_test')
    rospy.loginfo("Bruin-2 Graphical IO tester V1.0 Starting")

    app = QtWidgets.QApplication(sys.argv)
    mainwin = MainForm()
    mainwin.show()
    
    sys.exit(app.exec_())
