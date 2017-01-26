// Relay board driver node
// V1.1 Bill Lovegrove 12/27/2016 added recovery from missing serial port


#include <fstream>
#include <ctype.h>
#include "relay_board/relay_board_commands.h"
#include "relay_board/ros_interface.h"

using namespace std;

serial::Serial * my_serial;

int main(int argc, char **argv) {

    cout << "Bruin-2 Relay Driver V1.1 Starting" << endl;

    //function defined in ros_interface header
    startROS(argc, argv);

    ROSInterface ros_interface;

    std::string port = "/dev/relay_board";
    unsigned long baud = 19200;
    int fake_relay = 0;

// catch an invalid port error
    try {
	//my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
        my_serial = new serial::Serial();
        serial::Timeout tout(serial::Timeout::simpleTimeout(500));
        my_serial->setTimeout(tout);
        my_serial->setPort(port);
        my_serial->setBaudrate(baud);
	my_serial->open();
    }
    catch (exception &e) {
	ROS_ERROR_STREAM("Digipot serial port open failed." << e.what());
        fake_relay = true;
    }
    RelayBoardCommands relay_command_interface(my_serial);
    
    if ( fake_relay) {
        cout << "Bruin-2 Relay Driver running in fake mode." << endl;
	std::string device_type = "";
	int device_num = -1;
	std::string command = "";
	std::string state = "";
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
                            state="fake";
		        }
		    //device is a gpio command
		    } else {

		         if (command == "read") {
		            cout << "Relay " << device_num << " gpio read";
                            state="fake";
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

    }
    else {

	    //setup main loop
	    std::string device_type = "";
	    int device_num = -1;
	    std::string command = "";
	    std::string state = "";
            unsigned int mask;

	    while (ros_interface.isNodeRunning()) {

		//look for a message to react to. pass values by reference
		if (ros_interface.pollMessages(device_type, device_num, command, mask)) {
                        ROS_DEBUG_STREAM( "Relay: got command: [" << command << "] type " << device_type);
		    //device is a relay
		    if (device_type == "relay") {

		        //TODO: we should probebly read the state to make sure it actually changed
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
