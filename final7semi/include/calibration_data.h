#pragma once

#include <stdint.h>

struct CalibrationPoint {
    int32_t rawADC;
    float weight;
};

enum CalibrationMethod : uint8_t {
    CAL_LINEAR = 0,
    CAL_PIECEWISE_LINEAR = 1
};

struct CalibrationData {
    CalibrationPoint points[4];
    float k;
    float b;
    float r2;
    uint32_t timestamp;
    CalibrationMethod method;
};
