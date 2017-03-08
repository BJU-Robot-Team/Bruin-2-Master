//gps_node.cpp version 1.0
//BJU Robot Team Spring 2017. Coders: David Zuehlke 
// V1.0 Modified by Bill Lovegrove, Nathan Whoer
//
//	Desc: This program is designed to take input data from the gps (add more information) on the Bruin 2.0 vehicle. 
//	The data is then manipulated into a form usable by the main program of the vehicle to determine its location.
//

#include <iostream>
#include <chrono>
#include <thread>
#include <string>

#include <serial/serial.h>

#include "gps/ros_interface.h"
#include "gps/gps.h"

const std::string eol("\r");
const size_t max_line_length(128);

//Function that handles a serial read
string readSerial(serial::Serial *my_serial) {

	    //read response
	    string msg = my_serial->readline(max_line_length, eol);
	    
	    if (!msg.empty()) {
	    } else {
	        ROS_WARN_STREAM("Gps: Serial::readline() returned no data.");
	    }

	    ROS_DEBUG_STREAM("Gps: got message " << msg << endl);
	    return msg;

}

int main(int argc, char **argv)
{

    
    cout << "gps: gps_node built" << endl;
    //function defined in ros_interface header
    startROS(argc, argv);

    ROSInterface ros_interface;

    //setup serial connection
    string port = "/dev/gps";
    unsigned long baud = 4800;

    bool fake_gps = false; //fake gps for testing purposes

// catch an invalid port error
    serial::Serial * my_serial;
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
        //cout<< "Gps Serial open failed: " << e.what() << endl;
	ROS_ERROR_STREAM("Gps serial port open failed." << e.what());
        fake_gps = true;
    }

   
    //initialize a the object for gps data
    GPS gpsData();

    //hold the raw, unparsed GPS string
    string rawData;

    //hold GPS fields parsed from raw GPS serial string 
    vector<string> parsed_gps_string;
	

	
    //load fake data if no gps
    if ( fake_gps) {
        ROS_ERROR_STREAM("Gps continuing with fake data (21.1 degrees)..." << endl); 
	rawData = "$GPGGA,193956.000,3452.4626,N,08221.7059,W,0,00,,329.9,M,-32.1,M,,0000*48";
    }

    //while the ROS node object exists keep running
    while (ros_interface.isNodeRunning()) {

	//get serial data if we are not using test data
        if(!fake_gps){
            rawData = readSerial(my_serial);	    
	}	


        //parse raw gps serial string
	parsed_gps_string = parseLine(rawData);
	
        //get message format and handle it appropreatly 
        int index = indexOf(rawData,',');
	
        string interp = rawData.substr(0,index);

        if(interp == "$GPRMC" ) {
            //cout<<interp<<endl;
            //storeRMC(parsed_gps_string, gpsData);
        } else if(interp == "$GPGGA") {
            //cout<<interp<<endl;
            //storeGGA(parsed_gps_string, gpsData);
        } else if(interp == "$GPGSA"){
            //cout<<interp<<endl;
            //storeGSA(parsed_gps_string, gpsData);
        } else {
            cout<<"gps_node: NOT USABLE DATA"<<endl;
        }


       // ros_interface.publishMessages(gpsData);
        //cout << "Gps: Gps is publishing data" << endl;
        // Sleep for 1/2 second on fake data
        if (fake_gps) std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }    

}// end main methodf
