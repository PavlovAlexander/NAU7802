#include "scale_weighing.h"
#include "ui_console.h"
#include "calibration/nonlinear_calibration.h"
#include "calibration/temperature_compensation.h"
#include "filters/outlier_filter.h"
#include "filters/adaptive_averager.h"
#include "config/accuracy_config_manager.h"

// ============================================================================
// Глобальные экземпляры компонентов точности
// ============================================================================

static CalibrationModule g_calibration;
static TemperatureCompensator g_tempComp;
static OutlierFilter g_outlierFilter;
static AdaptiveAverager g_adaptiveAvg;
static AccuracyConfigManager g_config;

// ============================================================================
// Инициализация компонентов точности
// ============================================================================

void initAccuracyComponents() {
    Serial.println("[Accuracy] Initializing components...");
    
    // Загружаем конфигурацию из NVS
    g_config.init();
    
    // Применяем конфигурацию к компонентам
    const AccuracyConfig& cfg = g_config.getConfig();
    
    // OutlierFilter
    g_outlierFilter.setMethod((OutlierFilterMethod)cfg.outlierMethod);
    g_outlierFilter.setWindowSize(cfg.outlierWindowSize);
    g_outlierFilter.setSigmaWindow(cfg.outlierSigmaWindow);
    g_outlierFilter.setSigmaThreshold(cfg.outlierSigmaThreshold);
    
    // AdaptiveAverager
    g_adaptiveAvg.setEnabled(cfg.adaptiveAvgEnabled);
    g_adaptiveAvg.setStabilityThreshold(cfg.adaptiveStabilityThreshold);
    g_adaptiveAvg.setSampleRange(cfg.adaptiveMinSamples, cfg.adaptiveMaxSamples);
    g_adaptiveAvg.setTransitionSteps(cfg.adaptiveTransitionSteps);
    
    // TemperatureCompensator
    g_tempComp.setEnabled(cfg.temperatureCompEnabled);
    g_tempComp.setCoefficients(cfg.tempAlpha, cfg.tempBeta);
    
    Serial.println("[Accuracy] Components initialized");
}

// ============================================================================
// Функции доступа к компонентам (для консольных команд)
// ============================================================================

CalibrationModule& getCalibrationModule() {
    return g_calibration;
}

TemperatureCompensator& getTemperatureCompensator() {
    return g_tempComp;
}

OutlierFilter& getOutlierFilter() {
    return g_outlierFilter;
}

AdaptiveAverager& getAdaptiveAverager() {
    return g_adaptiveAvg;
}

AccuracyConfigManager& getAccuracyConfig() {
    return g_config;
}

// ============================================================================
// Вычисление веса (устаревшая функция для обратной совместимости)
// ============================================================================

float computeWeight(float k, float b, int32_t rawADC) {
    return k * (float)rawADC + b;
}

// ============================================================================
// Основной цикл взвешивания с интеграцией компонентов точности
// ============================================================================

void weighingTick(NAU7802& scale,
                  const CalibrationData& cal) {
    const AccuracyConfig& cfg = g_config.getConfig();
    
    // Шаг 1: Определить количество отсчётов
    int numSamples = 16;  // Дефолтное значение
    if (cfg.adaptiveAvgEnabled) {
        numSamples = g_adaptiveAvg.computeSampleCount();
    }
    
    // Шаг 2: Собрать отсчёты с применением фильтра выбросов
    int64_t sum = 0;
    int validSamples = 0;
    
    for (int i = 0; i < numSamples; i++) {
        while (!scale.available()) {
            delay(1);
        }
        int32_t rawReading = scale.getReading();
        
        // Применяем фильтр выбросов (если включён)
        int32_t filteredReading = rawReading;
        if (cfg.outlierFilterEnabled) {
            filteredReading = g_outlierFilter.filter(rawReading);
        }
        
        sum += filteredReading;
        validSamples++;
    }
    
    // Вычисляем среднее
    int32_t rawADC = (int32_t)(sum / validSamples);
    
    // Шаг 3: Применить калибровку (нелинейную, если настроена)
    float weight = 0.0f;
    if (cal.method != CAL_LINEAR) {
        // Используем нелинейную калибровку
        g_calibration.loadFrom(cal);
        weight = g_calibration.rawToWeight(rawADC);
    } else {
        // Используем линейную калибровку (обратная совместимость)
        weight = computeWeight(cal.k, cal.b, rawADC);
    }
    
    // Шаг 4: Применить температурную компенсацию (если включена)
    if (cfg.temperatureCompEnabled) {
        float temperature = g_tempComp.readTemperature(scale);
        if (!isnan(temperature)) {
            float k = cal.k;
            float b = cal.b;
            g_tempComp.compensate(temperature, k, b);
            // Пересчитываем вес с компенсированными коэффициентами
            weight = computeWeight(k, b, rawADC);
        }
    }
    
    // Шаг 5: Обновить историю адаптивного усреднения
    if (cfg.adaptiveAvgEnabled) {
        g_adaptiveAvg.updateHistory(weight);
    }
    
    // Вывод результата
    printWeighLine(rawADC, weight);
}
