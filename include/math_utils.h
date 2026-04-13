#pragma once
#include "calibration_data.h"

// Вычисляет коэффициенты k, b методом МНК по n точкам.
void linearRegression(const CalibrationPoint* points, int n,
                      float& k, float& b);

// Вычисляет R² для набора точек и модели y = k*x + b.
float computeR2(const CalibrationPoint* points, int n,
                float k, float b);
