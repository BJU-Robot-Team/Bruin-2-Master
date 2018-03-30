#ifndef CONFIG_H
#define CONFIG_H

struct Config {
    static double refLat, refLong;
    static std::vector<Coordinate> waypoints;

    static void init(); // initialize from ini file
};

#endif
