#ifndef SENSORS_H
#define SENSORS_H

class GPS {
    static Coordinate readPos();
};

class Compass {
    static double getDirection();
};

class Proximity {
    // returns true on failure, which should stop robot from going anywhere
    static bool sense();
};

class Radar {
    // returns true on failure, which should stop robot from going anywhere
    static bool sense();
};

#endif
