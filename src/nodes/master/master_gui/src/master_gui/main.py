#!/usr/bin/env python

#ATTENTION: DO NOT MODIFY THIS CODE WITHOUT FIRST CONSULTING CARTER SHEAN 
#(at least until Bruin 2 is finished)
#-----------------------------------------------------------------
#Python file for managing the GUI for the Bruin 2 Robot
#using PyQT5 modules and QT designer paired with the Pyuic command
#-----------------------------------------------------------------

#import the necessary modules
import sys
import os
import rospy
import rospkg
import std_msgs.msg
import subprocess

#TODO: I think we need to use the screeninfo module to move the robot (since QT's positioning is based on pixels and pixels are relative to the 
#monitor, I thought moving the robot locally would need to be based on the pixels of each individual screen)
#from screeninfo import get_monitors
from compass.msg import CompassDataMsg
from state_machine.msg import MsgsForGUI
from master_gui.msg import GUImsg
from roboteq_msgs.msg import Command
from sensor_msgs.msg import NavSatFix
from PyQt5 import QtCore, QtWidgets, QtGui


#create absolute paths to resources and code folders
resource_path = os.path.join(rospkg.RosPack().get_path('master_gui'), 'resource')
src_path = os.path.join(rospkg.RosPack().get_path('master_gui'), 'src', "master_gui")

#auto generate the files we are trying to import from
os.system("pyuic5 -o " + os.path.join(src_path, "bruin2.py") + " " + os.path.join(resource_path, "bruin2.ui") )
os.system("pyuic5 -o " + os.path.join(src_path, "bruin2state.py") + " " + os.path.join(resource_path, "bruin2state.ui"))
os.system("pyuic5 -o " + os.path.join(src_path, "gostopscreen.py") + " " + os.path.join(resource_path, "gostopscreen.ui"))

#import the information we need from those files
from bruin2 import *
from bruin2state import *
from gostopscreen import *

#global variables 
debuggingWindowText = ""
currentStation = "None"
x = 0
y = 0
turnDirection = ""
direction = ""
coordinates = ""
currentRobotState = "None"
goToStation = False

#Constants (PYTHON DOES NOT ACTUALLY HAVE CONSTANTS 
#SO BE CAREFUL ABOUT CHANGING THESE)
CONST_CLOCK_SPEED = 100


#class that intializes the GUI and handles events for the Main Window
class MainForm(QtWidgets.QMainWindow):
           
    def startGUImoving(self):
        global CONST_CLOCK_SPEED
        self.moveRobotTimer.start(CONST_CLOCK_SPEED)
    
    def debugButtonPressEvent(self, event):
        """creates a new debugging window when 
        the debug button is pressed"""
        if self.debuggingwindow is None:
            self.debuggingwindow = DebuggingInfo(self)
        self.debuggingwindow.show()

    
    def targetButton1PressEvent(self, event):
        """creates a new stop/go window when the first target button
        is clicked (or a similar signal was recieved from the combo box"""
        global currentStation
        #if the window is not already open, open a new window
        currentStation = "station1"
        if self.stopgowindow is None:
            self.stopgowindow = StopGo(self)
        self.stopgowindow.show()


    
    def targetButton2PressEvent(self, event):
        """creates a new stop/go window when the second target button
        is clicked (or a similar signal was recieved from the combo box"""
        global currentStation
        currentStation = "station2"
        # if the window is not already open, open a new window
        if self.stopgowindow is None:
            self.stopgowindow = StopGo(self)
        self.stopgowindow.show()

    
    def comboBoxPressEvent(self, event):
        """handle the text changing in the combo box, opens the corresponding
        corresponding target button stop/go window """
        if self.ui.cB_StationNum.currentText() is "None":
            currentStation = "None"
        elif self.ui.cB_StationNum.currentText() is "station1":
            self.targetButton1PressEvent(self)
        else:
            self.targetButton2PressEvent(self)

    def outputState(self):
        """publishes and logs to ros what state we are in,
        ticks every CONST_CLOCK_SPEED milliseconds"""
        self.msg.state = currentStation
        self.msg.goToNextState = str(goToStation)
        #rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    def moveGUI(self):
        """updates the GUI Robot object every time the timer 
        hits CONST_CLOCK_SPEED milliseconds per second using the 
        global X and Y variables"""
        #TODO: set the geometry equal to the values the GPS is giving out
        #adapted for the pixels on screen (will this value need to change based
        #on what the screen resolution is? 
        self.bruin2.setGeometry(QtCore.QRect(x, y, 30, 30))
        cmd = ['xrandr'] 
        cmd2 = ['grep', '*']
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        p2 = subprocess.Popen(cmd2, stdin=p.stdout, stdout=subprocess.PIPE)
        p.stdout.close()   
        resolution_string, junk = p2.communicate() 
        resolution = resolution_string.split()[0]
        width, height = resolution.split('x')
	
        print("unimplemented")
    def CompassCallBack(self, data):
        """Handles the call back from the compass, 
        extracting the heading and adding this information to
        the text of the debugging window"""
        global direction 
        direction = str(data.heading)
        #rospy.loginfo("%s" % (data.heading))

        #this is probably not necessary here
        global debuggingWindowText 
        debuggingWindowText += str(data.heading) + "\n"

    def SteerCallBack(self,data):
        """unimplemented method that gets the current turn direction
        of the robot and logs it"""
        global turnDirection
        #rospy.loginfo("%s" % (data))

    def StateMachineCallBack(self, data):
        """handles a call from the state machine's output
        as to which state we're in and sets that state name to 
        the corresponding state"""

        global currentRobotState
        #we get the message in the form 'current data: data_type
        #that needs split, done below
        discard, currentRobotState = str(data).split(':')
        currentRobotState.strip()
        self.ui.lbl_StateType.setText(currentRobotState)

        #rospy.loginfo("%s" % (data))
       
        global debuggingWindowText 
        debuggingWindowText += currentRobotState + "\n"
        

    def GPSCallBack(self, data):
        """handles the GPS's data output message, 
        TODO: may need commented out whenever 
        the GPS is not connected  """
        #rospy.loginfo("%s" % (data))
        global x
        global y
        x = str(data.longitude) 
        y = str(data.latitude)
        self.ui.lbl_lattitudeNum.setText(y)
        self.ui.lbl_longitudeNum.setText(x)
         
        global debuggingWindowText 
        debuggingWindowText += "coordinates: " + x + "," + y + "\n"
   
    def InitializeButtons(self):
        """set up the objects on the map (targets, bruin2)
        with scaled contents, desired text/pixmap and shape
        IMPORTANT NOTE: these will need changed every time the robot or targets change 
        locations"""
        self.bruin2 = QtWidgets.QLabel(self)
        self.bruin2.setGeometry(QtCore.QRect(350, 350, 30, 30))
        self.bruin2.setPixmap(QtGui.QPixmap(os.path.join(resource_path, "c_loc.png")))
        self.bruin2.setScaledContents(True)
        self.bruin2.setObjectName("lbl_bruin2")
        self.bruin2.show()

        self.target1 = QtWidgets.QPushButton(self)
        self.target1.setGeometry(QtCore.QRect(230, 300, 25, 25))
        self.target1.setObjectName("target1")
        self.target1.setText("1")
        self.target1.setStyleSheet("background-color: rgb(255, 0, 0);")
      
        self.target2 = QtWidgets.QPushButton(self)
        self.target2.setGeometry(QtCore.QRect(200, 250, 25, 25))
        self.target2.setObjectName("target2")
        self.target2.setText("2")
        self.target2.setStyleSheet("background-color: rgb(255, 0, 0);")
        
   
    def __init__(self, parent = None):
        """Constructor for Mainwindow (possibly split into
        separate method?)"""
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        #call the initialize buttons method which creates the robot button
        #and target buttons
        self.InitializeButtons()

        #set up the publisher of the state to be sent to the state machine
        rospy.init_node('GUIIO', anonymous=True)
        self.pub = rospy.Publisher('GUIData', GUImsg, queue_size=10)
        self.msg = GUImsg()
        self.msg.state = currentStation

        #set a timer to update which station we're going to every CONST_CLOCK_SPEED
        self.stateTimer = QtCore.QTimer()
        self.stateTimer.timeout.connect(self.outputState)
        self.moveRobotTimer = QtCore.QTimer()
        self.moveRobotTimer.timeout.connect(self.moveGUI)
        global CONST_CLOCK_SPEED
        self.stateTimer.start(CONST_CLOCK_SPEED)
        
 
        #initialize the listeners to the various things we need to pull data from
        rospy.Subscriber("CompassData", CompassDataMsg, self.CompassCallBack)
        rospy.Subscriber("CurrentState", MsgsForGUI, self.StateMachineCallBack)
        rospy.Subscriber("steer/cmd", Command, self.SteerCallBack)
        rospy.Subscriber("GPSData", NavSatFix, self.GPSCallBack)
   
        #initialize the pop up windows to None (will be initialized when a button is clicked)
        self.debuggingwindow = None
        self.stopgowindow = None
        
        #connect the signals of the button presses to their respective events
        self.ui.btn_debug.clicked.connect(self.debugButtonPressEvent)
        self.target1.clicked.connect(self.targetButton1PressEvent)
        self.target2.clicked.connect(self.targetButton2PressEvent)
        self.ui.cB_StationNum.currentIndexChanged.connect(self.comboBoxPressEvent)
        
        #setup map's picture using the pixmap feature
        self.ui.lbl_map.setPixmap(QtGui.QPixmap(os.path.join(resource_path, "map.png")))



#class that intializes and contains logic for the debugging window
class DebuggingInfo(QtWidgets.QMainWindow):
    
    def __init__(self, parent = None):
        """constructor for the debugging window, sets up
        window and creates a timer for updating the
         debugging information"""
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_DebuggingInfo()
        self.ui.setupUi(self)
        self.show()
        self.logFileTimer = QtCore.QTimer()
        self.logFileTimer.timeout.connect(self.timerHit)
        self.ui.btn_log.clicked.connect(self.timerStart)
 
    def timerHit(self):
        """sets the text of the debugging window
        equal to the global string"""
        global debuggingWindowText
        self.ui.label.setText(debuggingWindowText)

    def timerStart(self, event):
        """when the get debugging info button is pressed, 
        starts the timer getting information from the various 
        ros messages"""
        global CONST_CLOCK_SPEED
        self.logFileTimer.start(CONST_CLOCK_SPEED)
        

#class that contains the logic for stop/go on the robot itself
class StopGo(QtWidgets.QMainWindow):

    def timerHit(self):
        """Handles a timer hit by displaying the current
         information supplied by ROS messages"""

        #TODO: stil need the turn info and speed
        global direction, x, y
        self.ui.lbl_latittudedata.setText(str(y))
        self.ui.lbl_longitudedata.setText(str(x))
        self.ui.lbl_directiondata.setText(direction)

    def goButtonPress(self):
        """when the user presses the "yes I want to go here"
        button, disable the button and tell the state machine 
        we are actively going to the desired target"""
        #TODO: Renable this after the user closes the window
        self.ui.btn_go.setEnabled(False)
        global goToStation
        goToStation = True

    def __init__(self, parent = None):
        """"constructor for the window, creates timer for 
        updating information on the window"""
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.timer = QtCore.QTimer()
        self.ui.btn_go.setEnabled(True)
        self.timer.timeout.connect(self.timerHit)
	self.ui.lbl_latittudedata.setText(str(y))
        self.ui.lbl_longitudedata.setText(str(x))
        global CONST_CLOCK_SPEED
        self.timer.start(CONST_CLOCK_SPEED)
        self.ui.btn_go.clicked.connect(self.goButtonPress)
  

if __name__ == '__main__':
    """create a new window and display it until the user
    closes the window"""
    global mainwin
    app = QtWidgets.QApplication(sys.argv)
    mainwin = MainForm()
    mainwin.show()
    
    sys.exit(app.exec_())
