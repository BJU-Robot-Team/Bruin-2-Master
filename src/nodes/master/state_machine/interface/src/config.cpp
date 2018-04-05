#include "../headers/config.h"
#include "../headers/coord.h"

double Config::refLat, Config::refLong;
std::vector<Coordinate> Config::waypoints;

void Config::init() {
    // TODO
    refLat = 50;
    refLong = 30;
}