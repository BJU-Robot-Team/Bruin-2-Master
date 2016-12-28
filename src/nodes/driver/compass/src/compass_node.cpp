//compass_prog.cpp version 1.6
//Mechatronics ENG406 Fall 2016. Coders: David Zuehlke 
// V1.6 Modified by Bill Lovegrove
//
//	Desc: This program is designed to take input data from the OSU5000 digital compass on the Bruin 2.0 vehicle. The data is then manipulated into 
//	a usable form for the vehicle's main program.
//

#include <iostream>
#include <string>
#include <fstream>
#include <ctype.h>
#include <string.h>
#include <chrono>
#include <thread>

#include <serial/serial.h>

#include "compass/ros_interface.h"
#include "compass/compass_data.h"

//Class to hold methods that will do work on the imported data
class ManipulateData
{
public:
	Compass_Data ParseData(string); 

	string LoadData();

	double PullData(string, int);

	void SendData(Compass_Data);

};//end ManipulateData class

Compass_Data ManipulateData::ParseData(string rawData)
{
	Compass_Data newData(0);	//initialize a blank Compass_Data object	
	int length = rawData.length();
	bool finished = false;

	//Loop through the raw data string to find the information we want.
	for(int i =0; i <= length && !finished; i++)
	{
		char option = rawData[i];	//current character of the rawData string
		
		switch (option)	//switch through the wanted data options.
		{
		case 'C'://we found the heading
			newData.heading = PullData(rawData, i);
			break;
			
		case 'P': //we found the pitch
			newData.pitch = PullData(rawData, i);			
			break;

		case 'R'://we found the roll
			newData.roll = PullData(rawData, i);			
			break;

		case 'T'://we found the temperature
			newData.temperature = PullData(rawData, i);			
			finished = true;
			break;

		case '*'://check sum. Need to figure out how to use this to validate data. Checksum is a hex number that is the sum of all the characters from the '$' to the '*'
			newData.checkSum = PullData(rawData, i);			
			break;


		default:
			;//do nothing

		}//end switch statement

	}//end for loop
	
	return newData;	//return the parsed compass data.

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

//method to send compass data out to the main program.
void ManipulateData::SendData(Compass_Data newData)
{
	cout << "The heading is: " << newData.heading << endl;
	cout << "The pitch is: " << newData.pitch << endl;
	cout << "The roll is: " << newData.roll << endl;
	cout << "The temperature is: " << newData.temperature << endl;
	cout << "The check sum is: " << newData.checkSum << endl;

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
    string result = my_serial->read(30);
    size_t found = result.find('\n');
    result = result.substr(found+1);
    found = result.find('\n');
    result = result.substr(found+1);
    found = result.find('\n');
    result = result.substr(1,found);

    return result;
}

int main(int argc, char **argv)
{

    cout << "Bruin-2 Compass Driver V1.6 Starting" << endl;

    //function defined in ros_interface header
    startROS(argc, argv);

    ROSInterface ros_interface;

    //setup serial connection
    string port = "/dev/ttyAMC1";
    unsigned long baud = 19200;
    int fake_compass = 0;
    string rawData;

// catch an invalid port error
    serial::Serial * my_serial;
    try {
	my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
    }
    catch (exception &e) {
        //cout<< "Compass Serial open failed: " << e.what() << endl;
	ROS_ERROR_STREAM("Compass serial port open failed." << e.what());
        fake_compass = 1;
    }

   
    //initialize a blank compass data object
    Compass_Data compData(0);	

    //tester object to call data manipulation methods from.
	ManipulateData tester;	
	
    //load real or fake data 
    if ( fake_compass) {
        cout << "Compass continuing with fake data (21.1 degrees)..." << endl; 
	rawData = "$C21.1P-45.6R-163.4T20.5*27";
	ROS_ERROR_STREAM("Compass continuing with fake data (21.1 degrees)..." << endl);
    }
    else {
        rawData = readSerial(my_serial);
    }	

    while (ros_interface.isNodeRunning()) {

	//parse data.
	compData = tester.ParseData(rawData);
		
	//send data out to main program for debugging
	//tester.SendData(compData);
        ros_interface.publishMessages(compData);
        // Sleep for 1/2 second on fake data
        if (fake_compass) std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }    

}// end main method
