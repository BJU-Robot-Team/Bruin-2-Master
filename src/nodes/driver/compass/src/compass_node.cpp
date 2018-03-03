//compass_prog.cpp version 1.6
//Mechatronics ENG406 Fall 2016. Coders: David Zuehlke 
// V1.6 Modified by Bill Lovegrove
//
//	Desc: This program is designed to take input data from the OSU5000 digital compass on the Bruin 2.0 vehicle. The data is then manipulated into 
//	a usable form for the vehicle's main program.
//

#include <iostream>
#include <fstream>
#include <ctype.h>
#include <chrono>
#include <thread>

#include <serial/serial.h>

#include "compass/ros_interface.h"
#include "compass/compass_data.h"
#include "compass/manipulate_data.h"

const std::string eol("\r");
const size_t max_line_length(128);

Compass_Data ManipulateData::ParseData(std::string rawData) {
    Compass_Data newData(0);	//initialize a blank Compass_Data object	
    int length = rawData.length();
    bool finished = false;
    
    //Loop through the raw data string to find the information we want.
    for (int i = 0; i <= length && !finished; i++) {
        char option = rawData[i];	//current character of the rawData string
        
        switch (option)	//switch through the wanted data options.
        {
            case 'C':	//we found the heading
                // correct for east relativeness (90) and magnetic declination (6 degrees 37 arcminutes)
                //TODO: subscribe to GPS and calculate Magnetic declination from current GPS latitude
                //rospy.Subscriber("GPSData", NavSatFix, self.GPSCallBack)
                //not sure what this rawData string is, but possibly need the GPS Data object 
                
                /*
                 may need some sort of method like this
                 def GPSCallBack(self, data):
                 x = str(data.longitude) 
                 y = str(data.latitude)
                 #calculate the magnetic declination? Or is that done below already... will need to debug
                 */

                //add correction 			
                //formula to correct for east relative with CCW = 90 - theta
                newData.heading = PullData(rawData, i);
                newData.heading = 90 - newData.heading - (6 + 37 / 60);	//correct to east relative			
                if (newData.heading < 0) {
                    newData.heading = newData.heading + 360;
                }
                
                break;
                
            case 'P': //we found the pitch
                newData.pitch = PullData(rawData, i);
                break;
                
            case 'R': //we found the roll
                newData.roll = PullData(rawData, i);
                break;
                
            case 'T': //we found the temperature
                newData.temperature = PullData(rawData, i);
                finished = true;
                break;
                
            case '*': //check sum. Need to figure out how to use this to validate data. Checksum is a hex number that is the sum of all the characters from the '$' to the '*'
                newData.checkSum = PullData(rawData, i);
                break;
                
            default:
                ; //do nothing
                
        } //end switch statement
        
    } //end for loop
    
    return newData;	//return the parsed compass data.
    
}	//end ParseData method

//Method to pull data out of the passed in string. Iterate over from the start index until the next alphabetic character.
double ManipulateData::PullData(std::string rawData, int startIndex) {
    bool stop = false;
    int i = startIndex + 1;
    std::string tempStr = "";
    
    while (stop != true && i <= int(rawData.length())) {
        int ialpha = isalpha(rawData[i]); //check whether the next character is an alphabetic character.
                
        if (ialpha == 1 || rawData[i] == '*') {
            stop = true;
        } else //otherwise append the next character to the data string
        {
            tempStr += rawData[i];
            i++;
        } //end else
        
    } //end while loop
    
    return std::stod(tempStr); //return the data pulled from the string as a double.
}	//end PullData method

//method to send compass data out to the main program.
//TODO: Do we need to get rid of this method? Seems unnecessary at this point
void ManipulateData::SendData(Compass_Data newData) {
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
    
}	//end SendData method.

//method to load data from data source. (A file for now, but serial soon)
//TODO: change this to serial, if necessary at all
std::string ManipulateData::LoadData() {
    //This should be coming from the compass itself I believe, check to see if this sample data file exists on
    //linux computer
    std::ifstream inputFile("sample_data.txt");	//create a new input file object
    std::string testData = "";
    getline(inputFile, testData);	//read a line from the input file.
    inputFile.close();
    
    return testData;
    
}	//end LoadData method

//Function that handles a serial read
std::string readSerial(serial::Serial *my_serial) {
    
    //read response
    std::string msg = my_serial->readline(max_line_length, eol);
    
    if (msg.empty()) {
        ROS_WARN_STREAM("Compass: Serial::readline() returned no data.");
    }
    
    ROS_DEBUG_STREAM("Compass: got message " << msg << std::endl);
    return msg;
    
}

int main(int argc, char **argv) {
    
    //function defined in ros_interface header
    startROS(argc, argv);
    
    ROSInterface ros_interface;
    
    //setup serial connection
    std::string port = "/dev/compass";
    unsigned long baud = 19200;
    bool fake_compass = false;
    std::string rawData;
    
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
    } catch (std::exception &e) {
        //cout<< "Compass Serial open failed: " << e.what() << endl;
        ROS_ERROR_STREAM("Compass serial port open failed." << e.what());
        fake_compass = true;
    }
    
    //initialize a blank compass data object
    Compass_Data compData(0);
    
    //tester object to call data manipulation methods from.
    ManipulateData tester;
    
    //load fake data if no compass
    if (fake_compass) {
        ROS_ERROR_STREAM(
                "Compass continuing with fake data (21.1 degrees)..."
                        << std::endl);
        rawData = "$C21.1P-45.6R-163.4T20.5*27";
    }
    
    while (ros_interface.isNodeRunning()) {
        
        //get and parse data.
        
        if (!fake_compass) {
            rawData = readSerial(my_serial);
        }
        
        compData = tester.ParseData(rawData);
        
        //send data out to main program for debugging
        //tester.SendData(compData);
        ros_interface.publishMessages(compData);
        // Sleep for 1/2 second on fake data
        if (fake_compass)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
    }
    
}	// end main method
