//gps_node.cpp version 1.0
//BJU Robot Team Spring 2017. Coders: David Zuehlke 
// V1.0 Modified by Bill Lovegrove
//
//	Desc: This program is designed to take input data from the gps (add more information) on the Bruin 2.0 vehicle. 
//	The data is then manipulated into a form usable by the main program of the vehicle to determine its location.
//

#include <iostream>
#include <string>
#include <fstream>
#include <ctype.h>
#include <string.h>
#include <chrono>
#include <thread>

#include <serial/serial.h>

#include "gps/ros_interface.h"
#include "gps/gps_data.h"

const std::string eol("\r");
const size_t max_line_length(128);

//Class to hold methods that will do work on the imported data
class ManipulateData
{
public:
	Gps_Data ParseData(string); 

	string LoadData();

	double PullData(string, int);

	void SendData(Gps_Data);

};//end ManipulateData class

Gps_Data ManipulateData::ParseData(string rawData)
{
	Gps_Data newData(0);	//initialize a blank Gps_Data object	
	int length = rawData.length();
	bool finished = false;

        //$GPGGA,193956.000,3452.4626,N,08221.7059,W,0,00,,329.9,M,-32.1,M,,0000*48
	//example data above.

	//Loop through the raw data string to find the information we want.
	for(int i =0; i <= length && !finished; i++)
	{
		char option = rawData[i];	//current character of the rawData string
			
		//TODO rewrite the Case statement to look for the characters in the GPS message. Also, correct the case start characters to be the ones used in the actual gps messages.	
		switch (option)	//switch through the wanted data options.
		{
		case '1'://start of gps message
			;//figue out form compass version what to put here
			break;

		case 'C'://we found the hdop. Find out what hdop is!
			newData.hdop = PullData(rawData, i);
			break;
			
		case 'P': //we found the pdop. Find out what pdop is!
			newData.pdop = PullData(rawData, i);			
			break;

		case 'R'://we found the vdop. Find out what vdop is!
			newData.vdop = PullData(rawData, i);			
			break;

		case 'T'://we found the velocity
			newData.velocity = PullData(rawData, i);			
			finished = true;
			break;

		case '*'://we found the longitude
			newData.longitude = PullData(rawData, i);			
			
		
		case 'a'://we found the latitude
			newData.latitude = PullData(rawData, i);
			break;

		case 'b'://we found the altitude
			newData.altitude = PullData(rawData, i);
			finished = true;			
			break;
//Take a look at the Compass Case statement to make sure the default statment is correct.
			
		default:
			;//do nothing
			break;
		}//end switch statement

	}//end for loop
	
	return newData;	//return the parsed gps data.

}//end ParseData method


//Method to pull data out of the passed in string. Iterate over from the start index until the next alphabetic character.
double ManipulateData::PullData(string rawData, int startIndex)
{
	bool stop = false;
	int i = startIndex +1;
	string tempStr = "";

	while(stop != true && i <= int(rawData.length()))
	{
		int ialpha = isalpha(rawData[i]); //check whether the next character is an alphabetic character.
		
	    if(ialpha == 1 || rawData[i] == '*')	
		{
			stop = true;
		}
		else //otherwise append the next character to the data string
		{
			tempStr += rawData[i];
			i++;
		}//end else
		
	}//end while loop

	return stod(tempStr);	//return the data pulled from the string as a double.
}//end PullData method

//method to send Gps data out to the main program.
void ManipulateData::SendData(Gps_Data newData)
{
	//cout << "The heading is: " << newData.heading << endl;
	//cout << "The pitch is: " << newData.pitch << endl;
	//cout << "The roll is: " << newData.roll << endl;
	//cout << "The temperature is: " << newData.temperature << endl;
	//cout << "The check sum is: " << newData.checkSum << endl;

	//output data to a test file. (Comment out if no longer needaed)
	//ofstream outputfile("output_test.txt");
	//outputfile << "The heading is: " << newData.heading << endl;
	//outputfile << "The pitch is: " << newData.pitch << endl;
	//outputfile << "The roll is: " << newData.roll << endl;
	//outputfile << "The temperature is: " << newData.temperature << endl;
	//outputfile << "The check sum is: " << newData.checkSum << endl;
	
	//outputfile.close();
	
}//end SendData method.

//method to load data from data source. (A file for now, but serial soon)
string ManipulateData::LoadData()
{
	ifstream inputFile("sample_data.txt");	//create a new input file object
	string testData = "";
	getline(inputFile, testData);	//read a line from the input file.
	inputFile.close();
	
	return testData;

	
}//end LoadData method


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
    bool fake_gps = true;	//fake gps is turned on for testing purposes
    string rawData;

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

   
    //initialize a blank gps data object
    Gps_Data gpsData(0);	

    //tester object to call data manipulation methods from.
	ManipulateData tester;	
	
    //load fake data if no gps
    if ( fake_gps) {
        ROS_ERROR_STREAM("Gps continuing with fake data (21.1 degrees)..." << endl); 
	rawData = "$GPGGA,193956.000,3452.4626,N,08221.7059,W,0,00,,329.9,M,-32.1,M,,0000*48";
    }

    while (ros_interface.isNodeRunning()) {

	//get and parse data.
        
        if(!fake_gps){
            rawData = readSerial(my_serial);	    
	}	

	gpsData = tester.ParseData(rawData);
	
		
	//send data out to main program for debugging
	//tester.SendData(gpsData);
        ros_interface.publishMessages(gpsData);
        cout << "Gps: Gps is publishing data" << endl;
        // Sleep for 1/2 second on fake data
        if (fake_gps) std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }    

}// end main methodf
