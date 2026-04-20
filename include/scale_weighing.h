#pragma once
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include "calibration_data.h"

// --- Управление режимом взвешивания (весовой пользовательский уровень) ---
// Tare: следующий тик взвешивания зафиксирует текущий сглаженный вес как ноль.
void requestTare();
// Сброс tare в 0 (на случай перекалибровки / перезахода в режим).
void clearTare();
// Текущее tare-смещение в граммах (для отладки).
float getTareOffset();

// Forward declarations
class CalibrationModule;
class TemperatureCompensator;
class OutlierFilter;
class AdaptiveAverager;
class AccuracyConfigManager;

// Инициализация компонентов точности (вызывается из setup())
void initAccuracyComponents();

// Функции доступа к компонентам (для консольных команд)
CalibrationModule& getCalibrationModule();
TemperatureCompensator& getTemperatureCompensator();
OutlierFilter& getOutlierFilter();
AdaptiveAverager& getAdaptiveAverager();
AccuracyConfigManager& getAccuracyConfig();

// Вычисляет вес по линейной модели: weight = k * rawADC + b
float computeWeight(float k, float b, int32_t rawADC);

// Один цикл взвешивания (вызывается из loop()).
// Читает rawADC, вычисляет calibrated, выводит в консоль.
void weighingTick(NAU7802& scale, const CalibrationData& cal);

// Статистика
void printAccuracyStatistics();
void resetAccuracyStatistics();
