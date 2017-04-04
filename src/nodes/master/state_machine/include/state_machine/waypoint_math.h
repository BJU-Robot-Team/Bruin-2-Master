#ifndef WAYPOINTMATH_H
#define WAYPOINTMATH_H

#include <cmath>
#include <iostream>
#include "vehicle_data.h"

//pi needs to be called pi_num or else we get compile errors. I think it conflicts with a macro or something
#define pi_num 3.14159265358 // the value of pi to 11 decimal places
//#define lat_o 34.8686574  //origin latitude
//#define long_o -82.36457  //origin longitude 
#define lat_const 2*pi_num*(6371000/360) // latitude constant of operating area
#define long_const lat_const*cos(lower_left_lat/(360*2*pi_num)) // longitude constant of operating area
#define max_steer pi_num/6 // pi/6 Update to actual maximum steering amount ~ 30 degrees


class WaypointMath {

    public:

	WaypointMath() {};
     
	// Computes the y-cooridinate equivalent of the latitude of a given point.
	// p_lat is the latitude of the given point
	// lat_o is the origin latitude of the localized map
	// lat_const is the latitude constant for the current operational area
	// returns y-cooridinate relative to the local map
	double computeLatitude(double p_lat) { return ((p_lat-vehicle_data->origin_lat)*lat_const); }//end computeLatitude

	// Computes the x-cooridinate equivalent of the longitude of a given point.
	// p_long is the longitude of the given point
	// long_o is the origin longitude of the localized map
	// long_const is the longitude constant for the current operational area
	// returns x-cooridinate relative to the local map
	double computeLongitude(double p_long) { return ((p_long-vehicle_data->origin_long)*lat_const); } //end computeLongitude

	// Sets the Local Map Origin point
	// new_lat_o is the latitude desired to be set as the orgin latitude
	// new_long_o is the longitude desired to be set as the origin longitude
	// does not return anything, but sets the global variables of the latitude and
	// longitude constant for the local map
	//void setLocalOrigin(double new_lat_o, double new_long_o) { lat_o = new_lat_o,long_o = new_long_o; } //end setLocalOrigin

	// Calculates the angle to the waypoint relative to east.
	// temptheta_w is the quadrant relative angle to the waypoint
	// x1, y1, are the local position of the car// todo Correct to local coordinate system.
	// x2, y2, are the local position of the waypoint.
	//  return theta_w is the angle to the waypoint. Measured CCW from East.
	double calculateThetaW(double x1, double y1, double x2, double y2);

	// Determines which direction the vehicle should turn in order to reach its destination
	// theta_c is the car's heading relative to E, measured CCW.
	// theta_w is the waypoint heading relative to E, measured CCW.
	// theta_d is the angle from the car heading to the waypoint, measured CCW. We are pointed at the waypoint when theta_d = 0.
	// k is the proportional gain for steering commands
	// returns a steering angle (in radians). A positive steering command is CW and turns the vehicle right.
	double calcSteerAngle(double &theta_c, double &theta_w, double k);

};

#endif
