#ifndef COORD_H
#define COORD_H

class Vector {
private:
    double x, y;

public:
    Vector(double _x, double _y): x(_x), y(_y) {}
    double abs();   // get the length value of this vector
    double phase(); // get the phase angle of this vector in radians
    double phaseDeg(); // get the phase angle of this vector in degrees
    Vector unit();  // get a unit vector in this direction
    Vector operator-(Vector);
    Vector operator+(Vector);
    Vector operator/(double);
    Vector operator*(double);
};

class Coordinate {
private:
    double latitude, longitude;

public:
    Coordinate(double _lat, double _long): latitude(_lat), longitude(_long) {}
    static Coordinate fromLocal(double _x, double _y);
    Vector operator-(Coordinate other);
    double localX();
    double localY();
    double getLatitude();
    double getLongitude();
};

#endif