#pragma once

#include "calibration_data.h"

void saveCalibration(const CalibrationData& cal);
bool loadCalibration(CalibrationData& cal);
