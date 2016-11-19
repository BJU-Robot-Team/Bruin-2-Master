#ifndef OB_DETEC_H
#define OB_DETEC_H

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

class ObstacleDetect {
public:
  // Minimum distance for the vehicle to be from an obstacle, brake, and still
  // not collide with the obstacle
  double min_d = 15.0;
  // Threshold angle
  double theta1;
  // Width of Bruin2.0
  double car_width = 0;
  // Checks to see if there is anything in the front range of the vehicle
  void checkFront();
  // Checks to see if there is anything on the left side of the vehicle
  bool getProx(double d, double theta);
  // Calculates Distance between two obstacles
  double lawofCos(double a, double b, double theta_C);
};

#endif
