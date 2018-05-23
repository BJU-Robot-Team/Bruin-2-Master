import time
import rospy
from bruin2_msgs.msg import RelayCommandMsg, RelayDataMsg

# TODO: set correct relay numbers
RELAY_FRONT_HAZARDS = 0
RELAY_BACK_HAZARDS  = 1
RELAY_START_SWITCH  = 7
RELAY_SPEED_0       = 8
RELAY_SPEED_1_5     = 9
RELAY_SPEED_5_5     = 11
RELAY_WARNING_LIGHT = 15

RELAY_BRAKE = 13 # TBD

relay_pub = rospy.Publisher("RelayCotrol", RelayCommandMsg, queue_size=10)

relay_msg = None

def setRelayMsg(msg):
    global relay_msg:
    relay_msg = msg

rospy.subscribe("RelayData", RelayDataMsg, setRelayMsg)

def getRelayState(relayNum):
    relay_pub.publish("relay", relayNum, "read", 0)
    
    while relay_msg == None:
        time.sleep(0.001)

def sendRelayMessage(relayNum, on):
    relay_pub.publish("relay", relayNum, ["off", "on"][on], 0)
    
    return getRelayState(relayNum) == on

class Brake:
    def set():
        return sendRelayMessage(RELAY_BRAKE, True)
    
    def release():
        return sendRelayMessage(RELAY_BRAKE, False)

class Motor:
    def speed0():
        return sendRelayMessage(RELAY_SPEED_0, True) and 
               sendRelayMessage(RELAY_SPEED_1_5, False) and 
               sendRelayMessage(RELAY_SPEED_5_5, False)
    
    def speed1_5():
        return sendRelayMessage(RELAY_SPEED_0, False) and 
               sendRelayMessage(RELAY_SPEED_1_5, True) and 
               sendRelayMessage(RELAY_SPEED_5_5, False)
    
    def speed5_5():
        return sendRelayMessage(RELAY_SPEED_0, False) and 
               sendRelayMessage(RELAY_SPEED_1_5, False) and 
               sendRelayMessage(RELAY_SPEED_5_5, True)

class Hazards:
    def setFront(on):
        return sendRelayMessage(RELAY_FRONT_HAZARDS, on)
    
    def setBack(on):
        return sendRelayMessage(RELAY_BACK_HAZARDS, on)

class StartSwitch:
    def set(on):
        return sendRelayMessage(RELAY_START_SWITCH, on)

class WarningLight:
    status = False
    def set(on):
        WarningLight.status = on
        return sendRelayMessage(RELAY_WARNING_LIGHT, on)
    
    def toggle(event):
        WarningLight.set(not WarningLight.status)

class Steering:
    def steer(radians):
        pass # TODO: implement this
    
    def getAngle():
        pass # TODO: implement this
