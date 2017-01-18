#ifndef GPS_DATA_H 
#define GPS_DATA_H

using namespace std;


struct Gps_Data
{
public:
	double heading, pitch, roll, temperature, checkSum;

	Gps_Data();

	//declare 1 argument constructor. Used as a blank object constructor when a 0 is inputed because no arguments constructor is not working.
	Gps_Data(double);

};//end Gps_Data

Gps_Data::Gps_Data(double head)
	: heading(head)
{
	pitch = 0;
	roll = 0;
	temperature = 0;
	checkSum = 0;
}//end 1 argument constructor

Gps_Data::Gps_Data()
{
	heading = 0;
	pitch = 0;
	roll = 0;
	temperature = 0;
	checkSum = 0;
}//end no arguments constructor

#endif
