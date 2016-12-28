// Relay board driver node
// V1.1 Bill Lovegrove 12/27/2016 added recovery from missing serial port


#include <fstream>
#include <ctype.h>
#include "relay_board/relay_board_commands.h"
#include "relay_board/ros_interface.h"

using namespace std;

int main(int argc, char **argv) {

    cout << "Bruin-2 Relay Driver V1.0 Starting" << endl;

    //function defined in ros_interface header
    startROS(argc, argv);

    ROSInterface ros_interface;

    std::string port = "/dev/ttyAMC0";
    unsigned long baud = 9600;
    int fake_relay = 0;

    // OLD: serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    // catch an invalid port error
    serial::Serial * my_serial;
    try {
	my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
    }
    catch (exception &e) {
        //cout<< "Relay Driver Serial open failed: " << e.what() << endl;
	ROS_ERROR_STREAM("Relay Driver serial port open failed. " << e.what());
        fake_relay = 1;
    }


    if ( fake_relay) {
        cout << "Bruin-2 Relay Driver running in fake mode." << endl;
	std::string device_type = "";
	int device_num = -1;
	std::string command = "";
	std::string state = "";

	while (ros_interface.isNodeRunning()) {

	    //look for a message to react to. pass values by reference
	    if (ros_interface.pollMessages(device_type, device_num, command)) {

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
	    RelayBoardCommands relay_command_interface(my_serial);

	    //setup main loop
	    std::string device_type = "";
	    int device_num = -1;
	    std::string command = "";
	    std::string state = "";

	    cout << "Bruin-2 Relay Driver V1.0 Setup finished" << endl;

	    while (ros_interface.isNodeRunning()) {

		//look for a message to react to. pass values by reference
		if (ros_interface.pollMessages(device_type, device_num, command)) {

		    //device is a relay
		    if (device_type == "relay") {

		        //TODO: we should probebly read the state to make sure it actually changed
		        //   since a normal command to the relay board returns nothing

		        if (command == "on") {
		            relay_command_interface.relayOn(my_serial, device_num);
		            state = "on";

		        } else if (command == "off") {
		            relay_command_interface.relayOff(my_serial, device_num);
		            state = "off"; 
		        } else if (command == "read") {
		            state = relay_command_interface.relayRead(my_serial, device_num);
		            state = state.substr(state.find(" ") + 1); 
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

		}

	    }

    }
}
