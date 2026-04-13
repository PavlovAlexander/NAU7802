#pragma once
#include <stdint.h>

// Одна калибровочная точка: среднее из 50 отсчётов ADC + эталонный вес.
struct CalibrationPoint {
    int32_t rawADC;  // Среднее из 50 отсчётов
    float   weight;  // Эталонный вес, г
};

// Полный набор калибровочных данных.
struct CalibrationData {
    CalibrationPoint points[4]; // 0, 10, 20, 30 г
    float    k;                 // Наклон (slope), г/ADC
    float    b;                 // Смещение (intercept), г
    float    r2;                // Коэффициент детерминации
    uint32_t timestamp;         // millis() момента калибровки
};
