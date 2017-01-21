#ifndef GPS_DATA_H 
#define GPS_DATA_H

using namespace std;


struct Gps_Data
{
public:
	double hdop, pdop, vdop, velocity, longitude, latitude, altitude;
	bool valid;

	Gps_Data();

	//declare 1 argument constructor. Used as a blank object constructor when a 0 is inputed because no arguments constructor is not working.
	Gps_Data(double);

};//end Gps_Data

Gps_Data::Gps_Data(double head)
	: heading(head)
{
	hdop = 0;
	pdop = 0;
	vdop = 0;
	velocity = 0;
        longitude = 0;
	latitude = 0;
	altitude = 0;
	valid = false;
}//end 1 argument constructor

Gps_Data::Gps_Data()
{
	hdop = 0;
	pdop = 0;
	vdop = 0;
	velocity = 0;
	longitude = 0;
	latitude = 0;
	altitude = 0;
	valid = false;
}//end no arguments constructor

#endif
