#pragma once

#include "7Semi_NAU7802.h"
#include "calibration_data.h"

void requestTare();
void clearTare();
float getTareOffset();
void resetWeighingFilter();

float piecewiseWeightFromRaw(const CalibrationData& cal, int32_t rawADC);
void weighingTick(NAU7802_7Semi& scale, const CalibrationData& cal);
