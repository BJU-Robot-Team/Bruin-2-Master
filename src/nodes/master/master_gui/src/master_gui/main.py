#!/usr/bin/env python

#ATTENTION: DO NOT MODIFY THIS CODE WITHOUT FIRST CONSULTING CARTER SHEAN
#-----------------------------------------------------------------
#Python file for managing the GUI for the Bruin 2 Robot
#using PyQT5 modules and QT designer paired with the Pyuic command
#-----------------------------------------------------------------

import sys
import os
import rospy
import rospkg
import std_msgs.msg
from compass.msg import CompassDataMsg
from state_machine.msg import MsgsForGUI
from master_gui.msg import GUImsg
from roboteq_msgs.msg import Command
from PyQt5 import QtCore, QtWidgets, QtGui


#create absolute paths to resources and code folders
resource_path = os.path.join(rospkg.RosPack().get_path('master_gui'), 'resource')
src_path = os.path.join(rospkg.RosPack().get_path('master_gui'), 'src', "master_gui")

#auto generate the files we are trying to import from
os.system("pyuic5 -o " + os.path.join(src_path, "bruin2.py") + " " + os.path.join(resource_path, "bruin2.ui") )
os.system("pyuic5 -o " + os.path.join(src_path, "bruin2state.py") + " " + os.path.join(resource_path, "bruin2state.ui"))
os.system("pyuic5 -o " + os.path.join(src_path, "gostopscreen.py") + " " + os.path.join(resource_path, "gostopscreen.ui"))
os.system("pyuicmsg.state = currentState5 -o " + os.path.join(src_path, "gostopscreen.py") + " " + os.path.join(resource_path, "gostopscreen.ui"))

#import the information we need from those files
from bruin2 import *
from bruin2state import *
from gostopscreen import *

debuggingWindowText = ""
currentStation = "none"
x = 0
y = 0
turnDirection = ""
direction = ""
coordinates = ""
currentRobotState = "None"



#class that intializes the GUI and handles events for the Main Window
class MainForm(QtWidgets.QMainWindow):

       #simple escape event when the user presses the escape key
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.close()

    #creates a new debugging window when the debug button is pressed
    def debugButtonPressEvent(self, event):
        if self.debuggingwindow is None:
            self.debuggingwindow = DebuggingInfo(self)
        self.debuggingwindow.show()

    #creates a new stop/go window when a waypoint is clicked
    def waypointButton1PressEvent(self, event):
        global currentStation
        currentStation = "waypoint1"
        if self.stopgowindow is None:
            self.stopgowindow = StopGo(self)
        self.stopgowindow.show()


    #creates a new stop/go window when a waypoint is clicked
    def waypointButton2PressEvent(self, event):
        global currentStation
        currentStation = "waypoint2"
        if self.stopgowindow is None:
            self.stopgowindow = StopGo(self)
        self.stopgowindow.show()

    #writes what state we are in, ticks every .050 seconds
    def outputState(self):
        self.msg.state = currentStation
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    #gets a call from the compass, get the heading 
    def CompassCallBack(self, data):
        global direction 
        direction = str(data.heading)
        rospy.loginfo("%s" % (data.heading))

        #this is probably not necessary here
        global debuggingWindowText 
        debuggingWindowText += str(data.heading) + "\n"
    def SteerCallBack(self,data):
        global turnDirection
        rospy.loginfo("%s" % (data))

    #handles a call from the state machine's output 
    def StateMachineCallBack(self, data):
        global currentRobotState
        discard, currentRobotState = str(data).split(':')
        currentRobotState.strip()
        self.ui.lbl_StateType.setText(currentRobotState)

        rospy.loginfo("%s" % (data))
        #this is probably not necessary here
        global debuggingWindowText 
        debuggingWindowText += currentRobotState + "\n"
    def InitializeButtons(self):
         #set up the objects on the map (waypoints, bruin2)
        self.bruin2 = QtWidgets.QLabel(self)
        self.bruin2.setGeometry(QtCore.QRect(350, 350, 30, 30))
        self.bruin2.setPixmap(QtGui.QPixmap(os.path.join(resource_path, "c_loc.png")))
        self.bruin2.setScaledContents(True)
        self.bruin2.setObjectName("lbl_bruin2")
        self.bruin2.show()

        self.waypoint1 = QtWidgets.QPushButton(self)
        self.waypoint1.setGeometry(QtCore.QRect(230, 300, 25, 25))
        self.waypoint1.setObjectName("waypoint1")
        self.waypoint1.setText("1")
        self.waypoint1.setStyleSheet("background-color: rgb(255, 0, 0);")
      
        self.waypoint2 = QtWidgets.QPushButton(self)
        self.waypoint2.setGeometry(QtCore.QRect(200, 250, 25, 25))
        self.waypoint2.setObjectName("waypoint2")
        self.waypoint2.setText("2")
        self.waypoint2.setStyleSheet("background-color: rgb(255, 0, 0);")
        
    #initializes the Main Window, connecting events and initializes the timer
    def __init__(self, parent = None):
        """Constructor for Mainwindow (could use resizing"""
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.InitializeButtons()

        #set up the publisher of the state
        rospy.init_node('GUIIO', anonymous=True)
        self.pub = rospy.Publisher('GUIData', GUImsg)
        self.msg = GUImsg()
        self.msg.state = currentStation

        #set a timer to update which station we're going to every 100 milliseconds
        self.stateTimer = QtCore.QTimer()
        self.stateTimer.timeout.connect(self.outputState)
        self.stateTimer.start(50)

        #initialize the listener to the various things we need
        rospy.Subscriber("CompassData", CompassDataMsg,self.CompassCallBack)
        rospy.Subscriber("CurrentState", MsgsForGUI, self.StateMachineCallBack)
        rospy.Subscriber("steer/cmd", Command, self.SteerCallBack)
   
        #initialize the pop up windows to None
        self.debuggingwindow = None
        self.stopgowindow = None
        
        #connect the signals of the button presses to their respective events
        self.ui.btn_debug.clicked.connect(self.debugButtonPressEvent)
        self.waypoint1.clicked.connect(self.waypointButton1PressEvent)
        self.waypoint2.clicked.connect(self.waypointButton2PressEvent)

        #setup map's picture using the pixmap feature
        self.ui.lbl_map.setPixmap(QtGui.QPixmap(os.path.join(resource_path, "map.png")))



#class that intializes and contains logic for the debugging window
class DebuggingInfo(QtWidgets.QMainWindow):
    
     #get the latest information from the text file
     #function that intializes the form
    def __init__(self, parent = None):
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_DebuggingInfo()
        self.ui.setupUi(self)
        self.show()
        self.logFileTimer = QtCore.QTimer()
        self.logFileTimer.timeout.connect(self.timerHit)
        self.ui.btn_log.clicked.connect(self.timerStart)
 
        #self.totalText = ''
    def timerHit(self):
        self.getLogFileInfo()
    #gets the log file info and displays it to the screen
    def timerStart(self, event):
        self.logFileTimer.start(100)
   #gets the log file info and displays it to the screen
    def getLogFileInfo(self):
        self.ui.label.setText(debuggingWindowText)

#class that contains the logic for stop/go on the robot itself
class StopGo(QtWidgets.QMainWindow):
     #function that intializes the form, may need to modify code if the form changes names
     #function that processes the timer hit and updates the current info of the waypoint
    
    def timerHit(self):

        #TODO: replace these with the listener information
        global direction
        self.ui.lbl_latittudedata.setText("100")
        self.ui.lbl_longitudedata.setText("200")
        self.ui.lbl_directiondata.setText(direction)

    def goButtonPress(self):
        self.ui.btn_go.setEnabled(False)
        #TODO: set the global variable that contains the boolean
        #for whether or not we're going to a certain waypoint to false
    def __init__(self, parent = None):
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timerHit)
        self.timer.start(1000)
        self.ui.btn_go.clicked.connect(self.goButtonPress)
  
#main function
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    mainwin = MainForm()
    mainwin.show()
    
    sys.exit(app.exec_())
