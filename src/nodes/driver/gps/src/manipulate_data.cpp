

//####################
//Currently Depricated
//####################




//manipulate_data.cpp version 1.0
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
	
	//insert code written by Watson to parse the imported newData
	while(!finished){
	
                       index = indexOf(rawData,',');
                        if(index >= 0)
                        {
                            string interp = rawData.substr(0,index);
                            line = rawData.substr(index+1);
                            vector<string> data = parseLine(line);
                            if(interp == "$GPRMC" )
                            {
                                //cout<<interp<<endl;
                                storeRMC(data,&gps[i]);
                            }else if(interp == "$GPGGA"){
                                //cout<<interp<<endl;
                                storeGGA(data,&gps[i]);
                            }else if(interp == "$GPGSA"){
                                //cout<<interp<<endl;
                                storeGSA(data,&gps[i]);
                            }else{
                                //cout<<"NOT USABLE DATA"<<endl;
                            j--;
                            }
                        }
                   
           /* GPS out = averageData(gps);
            if(out.IsValid())
            {
                cout<<"Longitude: "<<out.longitude<<"W"<<endl;
                cout<<"Latitude: "<<out.latitude<<"N"<<endl;
            } else {
                cout<<"Invalid read"<<endl;
            } */

		finished = true;

        } //while (!finished)
    //need to debug


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
