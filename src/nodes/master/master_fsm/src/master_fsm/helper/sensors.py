import rospy
from bruin2_msgs.msg import CompassDataMsg, GPSDataMsg
from coordinate import Coordinate

class GPS:
    curPosition = None

    def getPosition():
        return GPS.curPosition
    
    def setPosition(msg):
        GPS.curPosition = Coordinate(msg.latitude, msg.longitude)

class Compass:
    direction = None

    def getDirection():
        return Compass.direction
    
    def setDirection(msg):
        Compass.direction = msg.heading

rospy.Subscribe("GpsData", GPSDataMsg, GPS.setPosition)
rospy.Subscribe("CompassData", CompassDataMsg, Compass.setDirection)
