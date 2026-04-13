#pragma once
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include "calibration_data.h"

// Вычисляет вес по линейной модели: weight = k * rawADC + b
float computeWeight(float k, float b, int32_t rawADC);

// Один цикл взвешивания (вызывается из loop()).
// Читает rawADC, вычисляет calibrated, выводит в консоль.
void weighingTick(NAU7802& scale, const CalibrationData& cal);
