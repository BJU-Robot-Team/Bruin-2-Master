#include <iostream>
#include <cmath>
#include <cassert>
#include <random>
#include <execinfo.h>
#include <unistd.h>
#include "../headers/coord.h"

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

int main() {
    testVector();
}
