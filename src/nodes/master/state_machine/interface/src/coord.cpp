#include "../headers/coord.h"
#include "../headers/config.h"
#include <cmath>

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
    double lat_const = 2 * pi * (6371000 / 360); // latitude constant of operating area
    double long_const = lat_const * cos((Config::refLat / 360) * 2 * pi); // longitude constant of operating area
    _lat  = Config::refLat  + lat_const  * _y;
    _long = Config::refLong + long_const * _x;
    return Coordinate(_lat, _long);
}

Vector Coordinate::operator-(Coordinate other) {
    return Vector(localX() - other.localX(), localY() - other.localY());
}

/* 
    Get local X coordinate from latitude / longitude
 */
double Coordinate::localX() {
    double lat_const = 2 * pi * (6371000 / 360); // latitude constant of operating area
    double long_const = lat_const * cos((Config::refLat / 360) * 2 * pi); // longitude constant of operating area
    return (longitude - Config::refLong) * long_const;
}

/* 
    Get local Y coordinate from latitude / longitude
 */
double Coordinate::localY() {
    double lat_const = 2 * pi * (6371000 / 360); // latitude constant of operating area
    return (latitude - Config::refLat) * lat_const;
}

double Coordinate::getLatitude() {
    return latitude;
}

double Coordinate::getLongitude() {
    return longitude;
}
