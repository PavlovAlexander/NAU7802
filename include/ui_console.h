#pragma once
#include <stdint.h>
#include "calibration_data.h"

// Выводит сообщение с произвольным тегом.
void printTagged(const char* tag, const char* msg);

// Форматирует и выводит строку взвешивания.
// Формат: [WEIGH] Raw: <rawADC> | Calibrated: <X.XXX>g
void printWeighLine(int32_t rawADC, float calibrated);

// Выводит диагностический статус с тегом [STAT].
void printStatus(const CalibrationData& cal);

// Выводит сообщение об ошибке с тегом [ERROR].
void printError(const char* msg);
