#pragma once
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include "calibration_data.h"

// Запускает интерактивный 4-шаговый мастер калибровки.
// Заполняет cal.points[], вычисляет коэффициенты в зависимости от метода.
// method: метод калибровки (по умолчанию CAL_LINEAR для обратной совместимости)
// Возвращает true если R² >= 0.99.
bool runCalibrationWizard(NAU7802& scale, CalibrationData& cal, CalibrationMethod method = CAL_LINEAR);

// Собирает numSamples отсчётов с интервалом 100 мс, возвращает среднее.
int32_t collectSamples(NAU7802& scale, int numSamples);
