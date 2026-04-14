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

// ============================================================================
// Команды управления компонентами точности
// ============================================================================

// Вывод статуса всех компонентов
void cmd_accuracy_status();

// Включение/выключение компонентов
void cmd_accuracy_enable(const char* component);   // component: "outlier", "temp", "adaptive"
void cmd_accuracy_disable(const char* component);

// Настройка OutlierFilter
void cmd_accuracy_set_outlier_method(const char* method);  // method: "median", "sigma", "none"
void cmd_accuracy_set_outlier_window(uint8_t size);        // size: 3, 5, 7

// Настройка TemperatureCompensator
void cmd_accuracy_set_temp_alpha(float value);
void cmd_accuracy_set_temp_beta(float value);

// Настройка AdaptiveAverager
void cmd_accuracy_set_adaptive_threshold(float value);
void cmd_accuracy_set_adaptive_samples(uint8_t minSamples, uint8_t maxSamples);

// Экспорт/импорт/сброс конфигурации
void cmd_accuracy_export();
void cmd_accuracy_import(const char* configText);
void cmd_accuracy_reset();
void cmd_accuracy_save();

// ============================================================================
// Диагностические команды
// ============================================================================

void cmd_accuracy_diag_on();
void cmd_accuracy_diag_off();
bool isDiagnosticMode();
void cmd_accuracy_stats();

// Примечание: cmd_accuracy_log_export() и cmd_accuracy_log_clear()
// будут добавлены в Task 12.1

// Forward declaration
class MeasurementLogger;

MeasurementLogger& getMeasurementLogger();
void cmd_accuracy_log_export();
void cmd_accuracy_log_clear();

// ============================================================================
// Тестовые команды
// ============================================================================

// Forward declaration
class NAU7802;

void cmd_accuracy_test_stability(NAU7802& scale, const CalibrationData& cal);
void cmd_accuracy_compare_calibration(NAU7802& scale, const CalibrationData& cal);
