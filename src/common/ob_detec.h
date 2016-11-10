#ifndef OB_DETEC_H
#define OB_DETEC_H

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

class ObstacleDetect {
public:
  // Checks to see if there is anything in the front range of the vehicle
  bool checkFront();
  // Checks to see if there is anything on the left side of the vehicle
  bool checkLeft();
  // Checks to see if there is anything on the right side of the vehicle
  bool checkRight();
  // Obtains information from the promixity sensor
  void getProx();
};
#endif
