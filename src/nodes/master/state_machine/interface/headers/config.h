#ifndef CONFIG_H
#define CONFIG_H

#include "coord.h"
#include <vector>

namespace Config {
    extern double refLat, refLong;
    extern std::vector<Coordinate> waypoints;

    void init(); // initialize from ini file
};

#endif
