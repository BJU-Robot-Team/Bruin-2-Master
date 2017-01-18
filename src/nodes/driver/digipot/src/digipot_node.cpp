//digipot_node.cpp version 1.0 
// V1. by Bill Lovegrove
//
//	Desc: This program is designed to control the
// "digipot" device which controls the speed on the Bruin 2.0 vehicle.

#define MAX_SPEED 11.2 // meters per second = 25 mph at full throttle
#define POT_NUM 0      // The pot number in use on the digipot board

#include <iostream>
#include <string>
#include <fstream>
#include <ctype.h>
#include <string.h>
#include <chrono>
#include <thread>

#include <serial/serial.h>

#include "digipot/ros_interface.h"
#include "digipot/digipot_data.h"

const std::string eol("\r");
const size_t max_line_length(128);

serial::Serial * my_serial;
bool fake_digipot = false;

//Function that handles a serial read
string readSerial(serial::Serial *serial_port) {

	    //read response
	    string msg = serial_port->readline(max_line_length, eol);
	    
	    if (!msg.empty()) {
	    } else {
	        ROS_WARN_STREAM("Digipot: Serial::readline() returned no data.");
	    }

	    ROS_DEBUG_STREAM("Digipot: got message " << msg << endl);
	    return msg;

}

void writeSerial(serial::Serial *serial_port, char *msg) {
    if (!fake_digipot) {
        serial_port->write(msg);
    } else {
       ROS_INFO_STREAM( "Digipot messsage not delivered." );
    }

}

// Called upon receipt of a message to the subscribed channel
void digipot_callback(const digipot::DigipotDataMsg& command) {
    char msg[5];
    // Send a command to the digipot board
    msg[0] = 0xFE;
    msg[1] = 0xAA;
    msg[2] = POT_NUM;
    msg[3] = 255*MAX_SPEED/command.speed;
    msg[4] = 0;
    writeSerial(my_serial, msg);
    // And a ROS debug message
    ROS_DEBUG_STREAM( "Digipot: got command " << command.speed );

}

int main(int argc, char **argv)
{

    //function defined in ros_interface header
    startROS(argc, argv);

    ROSInterface ros_interface(&digipot_callback);

    //setup serial connection
    string port = "/dev/digipot";
    unsigned long baud = 115200;

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
        fake_digipot = true;
    }

    //initialize a blank digipot data object
    Digipot_Status digipotData(0);

    // Send a default setting to the digipot board
    char msg[5];

    msg[0] = 0xFE;
    msg[1] = 0xAC;
    msg[2] = POT_NUM;
    msg[3] = 0;
    msg[4] = 0;
    if (!fake_digipot) writeSerial( my_serial, msg);

	
    while (ros_interface.isNodeRunning()) {

	// Publish status message	
        ros_interface.publishMessages(digipotData);
        // Sleep for 1/2 second
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }    

}// end main method
