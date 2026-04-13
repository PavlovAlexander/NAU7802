#pragma once
#include "calibration_data.h"

// Сохраняет CalibrationData в NVS (namespace "scale_cal").
void saveCalibration(const CalibrationData& cal);

// Загружает CalibrationData из NVS.
// Возвращает false если k == 0.0 (калибровка отсутствует).
bool loadCalibration(CalibrationData& cal);
