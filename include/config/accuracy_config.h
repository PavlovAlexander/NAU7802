#pragma once
#include <stdint.h>

// Методы фильтрации выбросов
enum OutlierFilterMethod : uint8_t {
    OUTLIER_NONE = 0,      // Отключён
    OUTLIER_MEDIAN = 1,    // Медианный фильтр
    OUTLIER_SIGMA = 2      // Статистический 3σ фильтр
};

// Конфигурация всех компонентов точности
struct AccuracyConfig {
    // Версия конфигурации (для совместимости при обновлениях)
    uint16_t version;
    
    // Флаги включения компонентов
    bool outlierFilterEnabled;
    bool temperatureCompEnabled;
    bool adaptiveAvgEnabled;
    
    // Параметры OutlierFilter
    uint8_t outlierMethod;          // OutlierFilterMethod
    uint8_t outlierWindowSize;      // Размер окна для медианного (3, 5, 7)
    uint8_t outlierSigmaWindow;     // Размер окна для σ вычисления (10-20)
    float   outlierSigmaThreshold;  // Порог в σ (обычно 3.0)
    
    // Параметры TemperatureCompensator
    float tempAlpha;                // Температурный коэффициент для k (ppm/°C)
    float tempBeta;                 // Температурный коэффициент для b (мг/°C)
    
    // Параметры AdaptiveAverager
    float   adaptiveStabilityThreshold;  // Порог σ для определения стабильности, мг
    uint8_t adaptiveMinSamples;          // Минимальное количество отсчётов (быстрый режим)
    uint8_t adaptiveMaxSamples;          // Максимальное количество отсчётов (точный режим)
    uint8_t adaptiveTransitionSteps;     // Количество шагов для плавного перехода
    
    // Контрольная сумма (для проверки целостности)
    uint16_t crc16;
};

// Значения по умолчанию
constexpr AccuracyConfig DEFAULT_ACCURACY_CONFIG = {
    .version = 1,
    .outlierFilterEnabled = false,
    .temperatureCompEnabled = false,
    .adaptiveAvgEnabled = false,
    .outlierMethod = OUTLIER_NONE,
    .outlierWindowSize = 5,
    .outlierSigmaWindow = 15,
    .outlierSigmaThreshold = 3.0f,
    .tempAlpha = 0.00002f,      // 20 ppm/°C
    .tempBeta = 0.5f,           // 0.5 мг/°C
    .adaptiveStabilityThreshold = 5.0f,  // 5 мг
    .adaptiveMinSamples = 10,
    .adaptiveMaxSamples = 50,
    .adaptiveTransitionSteps = 5,
    .crc16 = 0  // Вычисляется при сохранении
};
