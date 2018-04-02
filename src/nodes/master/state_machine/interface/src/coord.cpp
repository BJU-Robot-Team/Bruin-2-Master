#include "../headers/coord.h"
#include <cmath>

double pi = 3.14159265358979323846264338327950288419716;

/*
    Returns the length of the vector
 */
double Vector::abs() {
    return sqrt(x * x + y * y);
}

/*
    Returns the phase angle in radians
 */
double Vector::phase() {
    return atan2(y, x);
}

/*
    Returns the phase angle in degrees
 */
double Vector::phaseDeg() {
    return phase() * 180.0 / pi;
}

/*
    Returns a vector of magnitude 1 in the same direction as the source vector
 */
Vector Vector::unit() {
    return (*this) / abs();
}

Vector Vector::operator+(Vector other) {
    return Vector(x + other.x, y + other.y);
}

Vector Vector::operator-(Vector other) {
    return Vector(x - other.x, y - other.y);
}

Vector Vector::operator*(double scale) {
    return Vector(x * scale, y * scale);
}

Vector Vector::operator/(double scale) {
    return Vector(x / scale, y / scale);
}

/*
    Construct a Coordinate give local X and Y
 */
Coordinate Coordinate::fromLocal(double _x, double _y) {
    double _lat, _long;
    //TODO
    return Coordinate(_lat, _long);
}

Vector Coordinate::operator-(Coordinate other) {
    return Vector(localX() - other.localX(), localY() - other.localY());
}

/* 
    Get local X coordinate from latitude / longitude
 */
double Coordinate::localX() {
    // TODO
}

/* 
    Get local Y coordinate from latitude / longitude
 */
double Coordinate::localY() {
    // TODO
}

double Coordinate::getLatitude() {
    return latitude;
}

double Coordinate::getLongitude() {
    return longitude;
}
