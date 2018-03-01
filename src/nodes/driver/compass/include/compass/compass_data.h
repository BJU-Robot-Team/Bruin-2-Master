#ifndef COMPASS_DATA_H 
#define COMPASS_DATA_H


struct Compass_Data
{
public:
	double heading, pitch, roll, temperature, checkSum;

	Compass_Data();

	//declare 1 argument constructor. Used as a blank object constructor when a 0 is inputed because no arguments constructor is not working.
	Compass_Data(double);

};//end Compass_Data

Compass_Data::Compass_Data(double head)
	: heading(head)
{
	pitch = 0;
	roll = 0;
	temperature = 0;
	checkSum = 0;
}//end 1 argument constructor

Compass_Data::Compass_Data()
{
	heading = 0;
	pitch = 0;
	roll = 0;
	temperature = 0;
	checkSum = 0;
}//end no arguments constructor

#endif
