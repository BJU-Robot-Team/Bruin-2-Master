import serial
import rospy

class FakeRelayController:
    def __init__(self):
        # info to keep track of current status of relays
        self.data = 0

    def turnOnRelay(self, num):
        self.data |= (1 << num)
        return "on"
    
    def turnOffRelay(self, num):
        self.data &= ~(1 << num)
        return "off"
    
    def readRelay(self, num):
        if self.data & (1 << num):
            return "on"
        return "off"
    
    def readGPIO(self, num):
        return "fake"

class RelayController:
    """ Real Controller that actually does stuff """
    def __init__(self):
        # set up the serial port
        try:
            self.serial = serial.Serial('/dev/relay_board', 19200, timeout=1)
            self.transaction("\r")
        except:
            rospy.logerr("Serial port /dev/relay_board could not be opened. Shutting down.")
            rospy.signal_shutdown("Serial port /dev/relay_board could not be opened")
        
    def transaction(self, msg):
        msg += "\n\r"
        self.serial.write(msg)
        
        result = serial.read(30)
        result = result.split("\n")[3] # Apparently the actual result is the 4th line of response

        return result

    def turnOnRelay(self, num):
        self.transaction("relay on {}".format(num))
        return "on"
    
    def turnOffRelay(self, num):
        self.transaction("relay off {}".format(num))
        return "off"
    
    def readRelay(self, num):
        return self.transaction("relay read {}".format(num))
    
    def readGPIO(self, num):
        return self.transaction("gpio read {}".format(num))
