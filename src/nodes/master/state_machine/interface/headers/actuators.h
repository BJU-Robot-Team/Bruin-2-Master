#ifndef ACTUATORS_H
#define ACTUATORS_H

class Brake {
public:
    // false means it didn't work
    static bool set();
    static bool release();
};

enum Speed {
    FULL, FIFTH, NONE
};
class Motor {
    // false means it didn't work
    static bool noSpeed();
    static bool fifthSpeed();
    static bool fullSpeed();
    //static bool speed(double sp); // speed in m/s
    static Speed getSpeed();
};

/*
class Steering {
    // false means it didn't work
    static bool steer(double angle);
    static double getAngle();
};

class Hazards {
    // return false means it failed
    static bool setFront(bool active);
    static bool setBack(bool active);
};

class StartSwitch {
    // return false means it failed
    static bool set(bool active);
};

class WarningLight {
    // return false means it failed
    static bool set(bool active);
};
*/

#endif
