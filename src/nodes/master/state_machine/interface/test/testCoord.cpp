#include <iostream>
#include <cmath>
#include <cassert>
#include <random>
#include <execinfo.h>
#include <unistd.h>
#include "../headers/coord.h"
#include "../headers/config.h"

using namespace std;

double random(double from, double to) {
    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(from, to);
    return dist(e2);
}

void error() {
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

void assertEqual(double x, double y) {
    if (abs(x - y) > 1e-6) {
        error();
    }
}

Vector randomVec() {
    double a = random(-1e5, 1e5);
    double b = random(-1e5, 1e5);
    return Vector(a, b);
}

void testVectorUnit() {
    Vector vec = randomVec();
    assertEqual(vec.unit().abs(), 1);
}

void testTriangularIneq() {
    // assert that |a + b| <= |a| + |b|
    Vector a = randomVec();
    Vector b = randomVec();
    assert((a + b).abs() <= a.abs() + b.abs());
}

void testMinus() {
    Vector a = randomVec();
    Vector b = randomVec();
    assertEqual(((a + b) - a).abs(), b.abs());
    assertEqual(((a + b) - a).phase(), b.phase());
}

void testTimes() {
    Vector a = randomVec();
    double x = random(1, 1e3);
    assertEqual(a.abs(), ((a * x) / x).abs());
    assertEqual(a.phase(), ((a * x) / x).phase());
    assertEqual(a.abs() * x, (a * x).abs());
    assertEqual(a.abs() / x, (a / x).abs());
}

void testVector() {
    Vector vec(3,4);
    assertEqual(vec.abs(), 5);
    
    vec = Vector(5, -12);
    assertEqual(vec.abs(), 13);

    vec = Vector(1, 1);
    assertEqual(vec.phase(), pi / 4);
    assertEqual(vec.phaseDeg(), 45);

    for (int i = 0; i < 100; i++) {
        testVectorUnit();
        testTriangularIneq();
        testMinus();
        testTimes();
    }
}

// calculate great circle distance
// From Competitive Programming 3
double gcDistance(Coordinate c1, Coordinate c2, double radius) {
    double pLat = c1.getLatitude();
    double pLong = c1.getLongitude();
    double qLat = c2.getLatitude();
    double qLong = c2.getLongitude();
    pLat *= pi / 180; pLong *= pi / 180; // convert degree to radian
    qLat *= pi / 180; qLong *= pi / 180;
    return radius * acos(cos(pLat)*cos(pLong)*cos(qLat)*cos(qLong) +
        cos(pLat)*sin(pLong)*cos(qLat)*sin(qLong) +
        sin(pLat)*sin(qLat));
}

void testCoordinate() {
    Config::refLat  = 34.874264;
    Config::refLong = -82.363416; // Rodeheaver
    Coordinate c1 = Coordinate::fromLocal(0,0);
    Coordinate c2(34.875820, -82.361891); // M&G

    double earthR = 6371000; // in meters

    assert(abs((c2 - c1).abs() - gcDistance(c1, c2, earthR)) < 0.01);
}

int main() {
    testVector();
    testCoordinate();
}
