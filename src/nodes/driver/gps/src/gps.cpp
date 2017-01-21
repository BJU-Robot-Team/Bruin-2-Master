#include "gps/gps.h"
using namespace std;

//returns location of delim in str
int indexOf(string str, char delim)
{
    for(unsigned int i = 0; i < str.length(); i++)
    {
        if(str[i] == delim)
        {
            return i;
        }
    }
    return -1;
}

vector<string> parseLine(string str)
{
    vector<string> out;
    str = str.substr(0,indexOf(str, '*'));
    for(unsigned int i = 0; i < str.length(); i++)
    {
        int index = indexOf(str, ',');
        while(index >= 0)
        {
            out.push_back(str.substr(0,index));
            str = str.substr(index+1);
            index = indexOf(str, ',');
        }
        out.push_back(str);
    }
    return out;
}

//stores data for GPGGA
void storeGGA(vector<string> str, GPS *gps)
{
   //need to parse lat and long before shove
   if(str.at(1) != "")
   {

       string str1 = str.at(1).substr(0,2);
       string str2 = str.at(1).substr(2);

       gps->latitude = stof(str1) + stof(str2)/60;
   }
   if(str.at(3) != "")
   {
       string str1 = str.at(3).substr(0,3);
       string str2 = str.at(3).substr(3);

       gps->longitude = stof(str1) + stof(str2)/60;
   }
   if(str.at(5) == "1"){gps->valid = 1;}
   if(str.at(7) != "") {gps->hdop = stof(str.at(7));}
   if(str.at(8) != "") {gps->altitude = stof(str.at(8));}
}

//stores data for GPGSA
void storeGSA(vector<string> str, GPS *gps)
{
   if(str.at(1)  != "1"){gps->valid = 1;}
   if(str.at(14) != "") {gps->pdop = stof(str.at(14));}
   if(str.at(15) != "") {gps->hdop = stof(str.at(16));}
   if(str.at(16) != "") {gps->vdop = stof(str.at(16));}
}

//stores data for GPRMC
void storeRMC(vector<string> str, GPS *gps)
{
    //need to parse lat and long before shove
   //cout<<1;
   if(str.at(1) == "A"){gps->valid = 1;}
   //cout<<2;
   if(str.at(2) != "")
   {
       string str1 = str.at(2).substr(0,2);
       string str2 = str.at(2).substr(2);

       gps->latitude = stof(str1) + stof(str2)/60;
   }
   //cout<<4;
   if(str.at(4) != "")
   {
       string str1 = str.at(4).substr(0,3);
       string str2 = str.at(4).substr(3);

       gps->longitude = stof(str1) + stof(str2)/60;
   }
   //cout<<6;
   if(str.at(6) != "") {gps->velocity = stof(str.at(6))*1.150779;}
}

//Averages all GPS structs together in GPS vector vec
GPS averageData(vector<GPS> vec)
{
    GPS out;
    int invalid = 0;
    for(unsigned int i=0; i < vec.size(); i++)
    {
        if(vec.at(i).valid)
        {
            out.hdop      += vec.at(i).hdop;
            out.pdop      += vec.at(i).pdop;
            out.vdop      += vec.at(i).vdop;
            out.latitude += vec.at(i).latitude;
            out.longitude += vec.at(i).longitude;
            out.altitude  += vec.at(i).altitude;
            out.velocity  += vec.at(i).velocity;
            out.valid     = true;
        } else {invalid++;}
    }
    if(out.valid)
    {
        out.hdop      = out.hdop/(vec.size() - invalid);
        out.pdop      = out.pdop/(vec.size() - invalid);
        out.vdop      = out.vdop/(vec.size() - invalid);
        out.latitude = (out.latitude-500)/(vec.size() - invalid);
        out.longitude = (out.longitude-500)/(vec.size() - invalid);
        out.altitude  = out.altitude/(vec.size() - invalid);
        out.velocity  = out.velocity/(vec.size() - invalid);

    }
    return out;
}
