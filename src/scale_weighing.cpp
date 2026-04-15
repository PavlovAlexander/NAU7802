#include "scale_weighing.h"
#include "ui_console.h"
#include "calibration/nonlinear_calibration.h"
#include "calibration/temperature_compensation.h"
#include "filters/outlier_filter.h"
#include "filters/adaptive_averager.h"
#include "config/accuracy_config_manager.h"
#include "measurement_logger.h"

// ============================================================================
// Глобальные экземпляры компонентов точности
// ============================================================================

static CalibrationModule g_calibration;
static TemperatureCompensator g_tempComp;
static OutlierFilter g_outlierFilter;
static AdaptiveAverager g_adaptiveAvg;
static AccuracyConfigManager g_config;

// Кэш последней загруженной калибровки для оптимизации (не вызываем loadFrom на каждом тике)
static CalibrationMethod g_lastCalMethod = CAL_LINEAR;
static uint32_t g_lastCalTimestamp = 0;

// Статистика
static uint32_t g_totalMeasurements = 0;
static uint32_t g_tempCompensations = 0;

// Кэш температуры: обновляем не чаще раза в 10 секунд, чтобы не блокировать
// цикл взвешивания (~100-200 мс на одно чтение из NAU7802)
static float          g_cachedTemperature  = NAN;
static unsigned long  g_lastTempReadMs     = 0;
static const unsigned long TEMP_CACHE_INTERVAL_MS = 10000UL;

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
    bool diagnosticMode = isDiagnosticMode();
    
    g_totalMeasurements++;
    
    if (diagnosticMode) {
        Serial.println("=== Diagnostic Mode ===");
    }
    
    // Шаг 1: Определить количество отсчётов
    int numSamples = 16;  // Дефолтное значение
    if (cfg.adaptiveAvgEnabled) {
        numSamples = g_adaptiveAvg.computeSampleCount();
        if (diagnosticMode) {
            Serial.printf("[DIAG] Adaptive samples: %d (σ=%.3f mg)\n", 
                         numSamples, g_adaptiveAvg.getCurrentStdDev());
        }
    }
    
    // Шаг 2: Собрать отсчёты с применением фильтра выбросов
    int64_t sum = 0;
    int validSamples = 0;
    int32_t firstRawReading = 0;
    int32_t firstFilteredReading = 0;
    bool outlierDetected = false;
    
    for (int i = 0; i < numSamples; i++) {
        while (!scale.available()) {
            delay(1);
        }
        int32_t rawReading = scale.getReading();
        
        if (i == 0) {
            firstRawReading = rawReading;
        }
        
        // Применяем фильтр выбросов (если включён)
        int32_t filteredReading = rawReading;
        if (cfg.outlierFilterEnabled) {
            filteredReading = g_outlierFilter.filter(rawReading);
            if (filteredReading != rawReading && i == 0) {
                outlierDetected = true;
            }
        }
        
        if (i == 0) {
            firstFilteredReading = filteredReading;
        }
        
        sum += filteredReading;
        validSamples++;
    }
    
    // Вычисляем среднее
    int32_t rawADC = (int32_t)(sum / validSamples);
    
    if (diagnosticMode) {
        Serial.printf("[DIAG] Raw ADC (avg): %ld\n", (long)rawADC);
        if (cfg.outlierFilterEnabled) {
            Serial.printf("[DIAG] Outlier filter: %s (rejected: %lu/%lu)\n",
                         outlierDetected ? "ACTIVE" : "PASS",
                         g_outlierFilter.getOutlierCount(),
                         g_outlierFilter.getTotalCount());
        }
    }
    
    // Шаг 3: Применить калибровку (нелинейную, если настроена)
    // Оптимизация: перезагружаем g_calibration только при смене метода или обновлении калибровки
    if (cal.method != g_lastCalMethod || cal.timestamp != g_lastCalTimestamp) {
        g_calibration.loadFrom(cal);
        g_lastCalMethod = cal.method;
        g_lastCalTimestamp = cal.timestamp;
    }

    float weight = g_calibration.rawToWeight(rawADC);

    if (diagnosticMode) {
        const char* methodName = "LINEAR";
        if (cal.method == CAL_PIECEWISE_LINEAR) methodName = "PIECEWISE_LINEAR";
        else if (cal.method == CAL_POLYNOMIAL_2) methodName = "POLYNOMIAL_2";
        else if (cal.method == CAL_POLYNOMIAL_3) methodName = "POLYNOMIAL_3";
        Serial.printf("[DIAG] Calibration: %s (R²=%.4f)\n", methodName, cal.r2);
        Serial.printf("[DIAG] Weight (before temp comp): %.3f g\n", weight);
    }

    // Шаг 4: Применить температурную компенсацию (если включена)
    // Компенсация применяется как аддитивная поправка к уже вычисленному весу,
    // чтобы не разрушать результат нелинейной калибровки.
    // Поправка вычисляется как разница между компенсированным и некомпенсированным
    // линейным весом: delta = (k_corr * raw + b_corr) - (k * raw + b)
    float temperature = NAN;
    if (cfg.temperatureCompEnabled) {
        // Читаем температуру не чаще раза в TEMP_CACHE_INTERVAL_MS, чтобы избежать
        // задержки ~100-200 мс от переключения мультиплексора NAU7802
        unsigned long now = millis();
        if (isnan(g_cachedTemperature) || now - g_lastTempReadMs >= TEMP_CACHE_INTERVAL_MS) {
            g_cachedTemperature = g_tempComp.readTemperature(scale);
            g_lastTempReadMs = now;
        }
        temperature = g_cachedTemperature;
        if (!isnan(temperature)) {
            float k = cal.k;
            float b = cal.b;
            float kCorr = k;
            float bCorr = b;
            g_tempComp.compensate(temperature, kCorr, bCorr);

            // Аддитивная температурная поправка к весу
            float tempDelta = computeWeight(kCorr, bCorr, rawADC) - computeWeight(k, b, rawADC);
            float weightBefore = weight;
            weight += tempDelta;
            g_tempCompensations++;

            if (diagnosticMode) {
                Serial.printf("[DIAG] Temperature: %.2f °C\n", temperature);
                Serial.printf("[DIAG] Temp compensation delta: %.3f mg\n",
                             (weight - weightBefore) * 1000.0f);
            }
        }
    }
    
    // Шаг 5: Обновить историю адаптивного усреднения
    if (cfg.adaptiveAvgEnabled) {
        g_adaptiveAvg.updateHistory(weight);
    }
    
    // Шаг 6: Логирование (если включено)
    MeasurementLogger& logger = getMeasurementLogger();
    logger.log(firstRawReading, firstFilteredReading, weight, temperature, outlierDetected);
    
    if (diagnosticMode) {
        Serial.printf("[DIAG] Final weight: %.3f g\n", weight);
        Serial.println("======================");
    }
    
    // Вывод результата
    printWeighLine(rawADC, weight);
}

// ============================================================================
// Статистика
// ============================================================================

void printAccuracyStatistics() {
    Serial.println("=== Accuracy Statistics ===");
    Serial.printf("Total measurements:    %lu\n", g_totalMeasurements);
    Serial.printf("Temp compensations:    %lu\n", g_tempCompensations);
    Serial.printf("Outliers rejected:     %lu\n", g_outlierFilter.getOutlierCount());
    
    if (g_totalMeasurements > 0) {
        float tempCompRate = 100.0f * g_tempCompensations / g_totalMeasurements;
        Serial.printf("Temp comp rate:        %.1f%%\n", tempCompRate);
    }
    
    if (g_outlierFilter.getTotalCount() > 0) {
        float outlierRate = 100.0f * g_outlierFilter.getOutlierCount() / g_outlierFilter.getTotalCount();
        Serial.printf("Outlier rate:          %.1f%%\n", outlierRate);
    }
    
    Serial.printf("Uptime:                %lu ms\n", millis());
    Serial.println("===========================");
}

void resetAccuracyStatistics() {
    g_totalMeasurements = 0;
    g_tempCompensations = 0;
    g_outlierFilter.resetStatistics();
    Serial.println("[STATS] Statistics reset");
}
