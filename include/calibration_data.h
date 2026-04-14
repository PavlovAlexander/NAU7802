#pragma once
#include <stdint.h>

// Одна калибровочная точка: среднее из 50 отсчётов ADC + эталонный вес.
struct CalibrationPoint {
    int32_t rawADC;  // Среднее из 50 отсчётов
    float   weight;  // Эталонный вес, г
};

// Методы калибровки
enum CalibrationMethod : uint8_t {
    CAL_LINEAR = 0,           // y = kx + b (текущий, по умолчанию)
    CAL_PIECEWISE_LINEAR = 1, // Кусочно-линейная интерполяция
    CAL_POLYNOMIAL_2 = 2,     // Полином 2-й степени
    CAL_POLYNOMIAL_3 = 3      // Полином 3-й степени
};

// Полный набор калибровочных данных (расширенная версия).
struct CalibrationData {
    // Базовые поля (обратная совместимость)
    CalibrationPoint points[4]; // 0, 10, 20, 30 г
    float    k;                 // Наклон (slope), г/ADC
    float    b;                 // Смещение (intercept), г
    float    r2;                // Коэффициент детерминации
    uint32_t timestamp;         // millis() момента калибровки
    
    // Новые поля для нелинейной калибровки
    CalibrationMethod method;   // Метод калибровки (по умолчанию CAL_LINEAR)
    float    coeffs[4];         // Коэффициенты для полиномиальной (c0, c1, c2, c3)
    
    // Новые поля для температурной компенсации
    float    T_cal;             // Температура калибровки, °C (по умолчанию 25.0)
    float    alpha;             // Температурный коэффициент k, ppm/°C (по умолчанию 0.00002)
    float    beta;              // Температурный коэффициент b, мг/°C (по умолчанию 0.5)
};

// Значения по умолчанию для новых полей (для обратной совместимости)
constexpr CalibrationData DEFAULT_CALIBRATION_EXTENSION = {
    .points = {},
    .k = 0.0f,
    .b = 0.0f,
    .r2 = 0.0f,
    .timestamp = 0,
    .method = CAL_LINEAR,
    .coeffs = {0.0f, 0.0f, 0.0f, 0.0f},
    .T_cal = 25.0f,
    .alpha = 0.00002f,  // 20 ppm/°C
    .beta = 0.5f        // 0.5 мг/°C
};
