// Relay board driver node
// V1.1 Bill Lovegrove 12/27/2016 added recovery from missing serial port
// V1.2 Nathan Collisn 4/27/2018 Changed to simpler callback functions

#include <fstream>
#include <ctype.h>
#include "relay_board/relay_board_commands.h"
#include "relay_board/ros_interface.h"
#include "ros/ros.h"

using namespace std;

serial::Serial * my_serial;

ros::Publisher state_machine_pub;
bool publisherSetUp = false;

class FakeRelayController {
private:
    unsigned short data = 0;
public:
    string turnOnRelay(int num) {
        data |= (1 << num);
        return "on";
    }
    string turnOffRelay(int num) {
        data &= ~(1 << num);
        return "off";
    }
    string readRelay(int num) {
        bool on = data & (1 << num);
        return on ? "on" : "off";
    }
    string readGPIO(int num) {
        return "fake";
    }
};

FakeRelayController fakeRelayController;

void publishMessage(string type, int number, string state) {
    if (!publisherSetUp) {
        ros::NodeHandle InterfaceHandle;
        state_machine_pub = InterfaceHandle.advertise<relay_board::RelayDataMsg>("RelayData", 1, true);
    }

    relay_board::RelayDataMsg message;
    message.device_type = type;
    message.device_number = number;
    message.state = state;
    
    //Publish message
    state_machine_pub.publish(message);
}

void fake_callback(const relay_board::RelayCommandMsg& msg) {
    ROS_DEBUG_STREAM("Received fake Relay message: " << msg.device_type  << " " << (int) msg.device_number << " " << msg.command);

    string state;
    if (msg.device_type == "relay") {
        if (msg.command == "on")   { state = fakeRelayController.turnOnRelay(msg.device_number); }
        if (msg.command == "off")  { state = fakeRelayController.turnOffRelay(msg.device_number); }
        if (msg.command == "read") { state = fakeRelayController.readRelay(msg.device_number); }
    } else {
        // GPIO
        state = fakeRelayController.readGPIO(msg.device_number);
    }
    publishMessage(msg.device_type, msg.device_number, state);
}

void real_callback(const relay_board::RelayCommandMsg& msg) {
    ROS_DEBUG_STREAM("Received real Relay message: " << msg.device_type  << " " << (int) msg.device_number << " " << msg.command);
    // TODO: implement this. See below for old implementation and above for nicer format
    ROS_ERROR_STREAM("Error: not implemented");
    // crash the program. Only directly crashes this node (relay_board), but relay_board is marked as required in launch.xml
    // so the whole thing will crash
    ros::shutdown();
}

int main(int argc, char **argv) {
    ROS_INFO_STREAM("Bruin-2 Relay Driver V1.2 Starting");
    
    //function defined in ros_interface header
    startROS(argc, argv);

    ros::NodeHandle InterfaceHandle;
    
    bool isDebug = getenv("ROS_DEBUG") != nullptr;

    if (isDebug) {
        ROS_DEBUG_STREAM("Subscribing with fake callback");

        // need to assign this to a variable, and when the variable goes out of scope, we unsubscribe
        auto subscriber = InterfaceHandle.subscribe("RelayControl", 1000, fake_callback);
        ros::spin();
        return 0;
    }

    string port = "/dev/relay_board";
    unsigned long baud = 19200;

    try {
        // my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
        my_serial = new serial::Serial();
        serial::Timeout tout(serial::Timeout::simpleTimeout(500));
        my_serial->setTimeout(tout);
        my_serial->setPort(port);
        my_serial->setBaudrate(baud);
        my_serial->open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Relay serial port open failed." << e.what());
        // crash the program. Only directly crashes this node (relay_board), but relay_board is marked as required in launch.xml
        // so the whole thing will crash
        ros::shutdown();
    }

    auto subscriber = InterfaceHandle.subscribe("RelayControl", 1000, real_callback);
    ros::spin();
}

/*
int main(int argc, char **argv) {
    cout << "Bruin-2 Relay Driver V1.1 Starting" << endl;
    
    //function defined in ros_interface header
    startROS(argc, argv);
    
    ROSInterface ros_interface;
    
    string port = "/dev/relay_board";
    unsigned long baud = 19200;
    bool fake_relay = false;
    
    // catch an invalid port error
    try {
        //my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
        my_serial = new serial::Serial();
        serial::Timeout tout(serial::Timeout::simpleTimeout(500));
        my_serial->setTimeout(tout);
        my_serial->setPort(port);
        my_serial->setBaudrate(baud);
        my_serial->open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Relay serial port open failed." << e.what());
        fake_relay = true;
    }
    
    if (fake_relay) {
        cout << "Bruin-2 Relay Driver running in fake mode." << endl;
        string device_type = "";
        int device_num = -1;
        string command = "";
        string state = "";
        unsigned int mask;
        
        while (ros_interface.isNodeRunning()) {
            
            //look for a message to react to. pass values by reference
            if (ros_interface.pollMessages(device_type, device_num, command, mask)) {
                
                //device is a relay
                if (device_type == "relay") {
                    
                    //TODO: we should probebly read the state to make sure it actually changed
                    //   since a normal command to the relay board returns nothing
                    
                    if (command == "on") {
                        cout << "Relay " << device_num << " on";
                        
                    } else if (command == "off") {
                        cout << "Relay " << device_num << " off";
                    } else if (command == "read") {
                        cout << "Relay " << device_num << " read";
                        state = "fake";
                    }
                    //device is a gpio command
                } else {
                    
                    if (command == "read") {
                        cout << "Relay " << device_num << " gpio read";
                        state = "fake";
                    }
                    
                }
                
                //send message to anyone listening so they know the sate has changed
                ros_interface.publishMessages(device_type, device_num, state);
                
                //reset values
                device_type = "";
                device_num = -1;
                command = "";
                state = "";
            }
        }
        
    } else {
        RelayBoardCommands relay_command_interface(my_serial);
        
        //setup main loop
        string device_type = "";
        int device_num = -1;
        string command = "";
        string state = "";
        unsigned int mask;
        
        while (ros_interface.isNodeRunning()) {
            
            //look for a message to react to. pass values by reference
            if (ros_interface.pollMessages(device_type, device_num, command, mask)) {
                ROS_DEBUG_STREAM("Relay: got command: [" << command << "] type " << device_type);
                //device is a relay
                if (device_type == "relay") {
                    
                    //TODO: we should probably read the state to make sure it actually changed
                    //   since a normal command to the relay board returns nothing
                    ROS_DEBUG_STREAM( "Relay: got relay command: [" << command << "]");
                    if (command == "on") {
                        relay_command_interface.relayOn(my_serial, device_num);
                        state = "on";
                        
                    } else if (command == "off") {
                        relay_command_interface.relayOff(my_serial, device_num);
                        state = "off";
                    } else if (command == "read") {
                        state = relay_command_interface.relayRead(my_serial, device_num);
                        state = state.substr(state.find(" ") + 1);
                    } else if (command == "writeall") {
                        relay_command_interface.writeAll(my_serial, mask);
                    }
                    
                    //device is a gpio command
                } else {
                    
                    if (command == "read") {
                        state = relay_command_interface.gpioRead(my_serial, device_num);
                        state = state.substr(state.find(" ") + 1);
                        
                    }
                    
                }
                
                //send message to anyone listening so they know the sate has changed
                ros_interface.publishMessages(device_type, device_num, state);
                
                //reset values
                device_type = "";
                device_num = -1;
                command = "";
                state = "";
                ros::spinOnce();
                
            }
            
        }
        
    }
}
*/