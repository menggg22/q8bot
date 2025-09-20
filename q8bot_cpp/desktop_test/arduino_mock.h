#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

// Mock Arduino functions for desktop testing
#include <iostream>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>

// Use standard math functions - no need to redefine them
using std::abs;
using std::max;
using std::min;
using std::round;
using std::pow;
using std::sqrt;
using std::sin;
using std::cos;
using std::acos;

// Mock Arduino functions
class MockSerial {
public:
    void begin(int baud) { (void)baud; } // Suppress unused parameter warning
    void print(const char* str) { std::cout << str; }
    void print(int val) { std::cout << val; }
    void print(float val) { std::cout << val; }
    void print(double val) { std::cout << val; }
    void println() { std::cout << std::endl; }
    void println(const char* str) { std::cout << str << std::endl; }
    void println(int val) { std::cout << val << std::endl; }
    void println(float val) { std::cout << val << std::endl; }
    void println(double val) { std::cout << val << std::endl; }
};

extern MockSerial Serial;

// Arduino utility functions that don't conflict
inline long millis() { return 0; }
inline void delay(int ms) { (void)ms; /* no-op for testing */ }

#endif