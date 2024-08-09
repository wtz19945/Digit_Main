#pragma once

#include <cmath>

inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T>
int is_positive(T val) {
    return T(0) < val;
}



