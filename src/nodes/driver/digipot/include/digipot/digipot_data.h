#ifndef DIGIPOT_DATA_H 
#define DIGIPOT_DATA_H

using namespace std;


struct Digipot_Status
{
public:
	double speed;

	Digipot_Status();

	//declare 1 argument constructor. Used as a blank object constructor when a 0 is inputed because no arguments constructor is not working.
	Digipot_Status(double);

};//end Compass_Data

Digipot_Status::Digipot_Status(double s)
	: speed(s)
{
	speed = s;
}//end 1 argument constructor

Digipot_Status::Digipot_Status()
{
	speed = 0;
}//end no arguments constructor

#endif
