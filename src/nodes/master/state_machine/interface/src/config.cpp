#include "../headers/config.h"
#include "../headers/coord.h"

double Config::refLat, Config::refLong;
std::vector<Coordinate> Config::waypoints;

void Config::init() {
    // TODO
    refLat = 42.67833241;
    refLong = -83.19551079;
}