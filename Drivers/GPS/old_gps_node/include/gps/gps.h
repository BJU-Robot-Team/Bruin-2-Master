#ifndef GPS_H 
#define GPS_H

//Jonathan watson
//For the Bruin-2 Project
//GPS Device:
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
struct GPS;

int indexOf(string str, char delim);
vector<string> parseLine(string str);
void storeGGA(vector<string> str, GPS &gps);
void storeGSA(vector<string> str, GPS &gps);
void storeRMC(vector<string> str, GPS &gps);
GPS averageData(vector<GPS> vec);


// Struct for the GPS variable
struct GPS
{
    float hdop = -1,
          pdop = -1,
          vdop = -1,
          velocity = -1,
          longitude = 500, //not possible, definitely wrong
          latitude = 500, //see above
          altitude = -1;
    bool  valid = 0;
    //bool IsValid()
    //{
    //        return valid;
    //}

};

//Need to parse lat and long to get correct info.

#endif
