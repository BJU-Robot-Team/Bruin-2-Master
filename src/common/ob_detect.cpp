#include "ob_detec.h"

// Desc: Checks to see if there is anything in the 270 degree front range of the vehicle
// Parameters: None
// Returns: Bool
void checkFront() {

}

// Desc: Determines if an obstacle is a threat to the vehicle or not by
//       comparing the distance of the obstacle and angle to the minimum
//       distance and angle the vehicle allows
// Parameters: Obtains the distance (as a double) of an obstacle and
//             angle from obstacle to car (car as reference point)
//             from the promixity sensor
// Returns: Bool if it is a threat or not.
bool getProx(double d, double theta) {
  return false;
}

double decodeTheta(std::string en_type) {
  double theta;
  return theta;
}
// Desc: Uses the Law of Cosines to determine the distance between the two obstacles
// Parameters: length (as a double) from car to obstacle 1 (a), length (as a double)
//             from car to obstacle 2 (b) angle between obstacle 1 and 2 (theta_C)
// Returns: distance between obstacle 1 and 2 as a double (c)
double lawofCos(double a, double b, double theta_C) {
  double c;
  return (sqrt((a*a)+(b*b)-(2*a*b*cos(theta_C))));
}
