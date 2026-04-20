#include "scale_weighing.h"
#include "ui_console.h"
#include "calibration/nonlinear_calibration.h"
#include "calibration/temperature_compensation.h"
#include "filters/outlier_filter.h"
#include "filters/adaptive_averager.h"
#include "config/accuracy_config_manager.h"
#include "measurement_logger.h"
#include <math.h>

// ============================================================================
// Константы режима взвешивания (пользовательский уровень)
// ============================================================================
// Deadband отображения: колебания < 2 мг не «дёргают» показанный вес.
static constexpr float WEIGH_DEADBAND_G      = 0.002f;   // 2 мг
// Адаптивное экспоненциальное сглаживание: alpha по величине изменения.
static constexpr float WEIGH_ALPHA_FAST      = 0.70f;    // резкое изменение → быстрая реакция
static constexpr float WEIGH_ALPHA_SLOW      = 0.15f;    // малое изменение → плавное успокоение
static constexpr float WEIGH_BIG_DELTA_G     = 0.030f;   // 30 мг — порог «большого» изменения
// Окно для вычисления σ и индикатора стабильности.
static constexpr int   WEIGH_STAB_WINDOW     = 8;
static constexpr float WEIGH_STAB_SIGMA_G    = 0.001f;   // σ < 1 мг => стабильно
// Перегрузка по диапазону датчика (выбор A: 50 г safe).
static constexpr float WEIGH_WARN_G          = 40.0f;    // предупреждение
static constexpr float WEIGH_OVERLOAD_G      = 50.0f;    // превышение safe-диапазона

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

// Четыре модели калибровки для параллельного вывода (LIN / PW / P2 / P3).
// Пересоздаются при обновлении набора точек (по cal.timestamp).
static CalibrationModule g_calLin;
static CalibrationModule g_calPw;
static CalibrationModule g_calP2;
static CalibrationModule g_calP3;
static uint32_t g_cal4Timestamp = 0;

// Пользовательский уровень: tare, сглаживание, окно стабильности, overload.
static bool    g_tarePending     = false;
static float   g_tareOffset_g    = 0.0f;
static float   g_smoothedWeight  = NAN;
static float   g_displayedWeight = NAN;   // после deadband (что видит пользователь)
static float   g_stabBuf[WEIGH_STAB_WINDOW] = {0};
static int     g_stabIdx         = 0;
static int     g_stabCount       = 0;
static bool    g_overloadLatched = false; // защёлка, чтобы не спамить OVERLOAD

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
// Пользовательский уровень: tare, сглаживание, стабильность
// ============================================================================

void requestTare()      { g_tarePending = true; }
void clearTare()        { g_tareOffset_g = 0.0f; g_tarePending = false;
                          g_smoothedWeight = NAN; g_displayedWeight = NAN;
                          g_stabCount = 0; g_stabIdx = 0;
                          g_overloadLatched = false; }
float getTareOffset()   { return g_tareOffset_g; }

// Пересобирает 4 модели калибровки из актуальных точек (если они изменились).
static void refreshFourModels(const CalibrationData& cal) {
    if (cal.timestamp == g_cal4Timestamp) return;
    // Требуется 4 валидные точки (по текущему мастеру калибровки всегда 4).
    g_calLin.setMethod(CAL_LINEAR);           g_calLin.calibrate(cal.points, 4);
    g_calPw .setMethod(CAL_PIECEWISE_LINEAR); g_calPw .calibrate(cal.points, 4);
    g_calP2 .setMethod(CAL_POLYNOMIAL_2);     g_calP2 .calibrate(cal.points, 4);
    g_calP3 .setMethod(CAL_POLYNOMIAL_3);     g_calP3 .calibrate(cal.points, 4);
    g_cal4Timestamp = cal.timestamp;
}

// Адаптивное экспоненциальное сглаживание.
static float smoothWeight(float raw) {
    if (isnan(g_smoothedWeight)) { g_smoothedWeight = raw; return raw; }
    float delta = fabsf(raw - g_smoothedWeight);
    float alpha = (delta > WEIGH_BIG_DELTA_G) ? WEIGH_ALPHA_FAST : WEIGH_ALPHA_SLOW;
    g_smoothedWeight += alpha * (raw - g_smoothedWeight);
    return g_smoothedWeight;
}

// Display deadband: обновляем показ только при изменении > WEIGH_DEADBAND_G.
static float applyDisplayDeadband(float smoothed) {
    if (isnan(g_displayedWeight)) { g_displayedWeight = smoothed; return smoothed; }
    if (fabsf(smoothed - g_displayedWeight) >= WEIGH_DEADBAND_G) {
        g_displayedWeight = smoothed;
    }
    return g_displayedWeight;
}

// σ последних WEIGH_STAB_WINDOW сглаженных значений, граммы.
static float pushAndComputeSigma(float smoothed) {
    g_stabBuf[g_stabIdx] = smoothed;
    g_stabIdx = (g_stabIdx + 1) % WEIGH_STAB_WINDOW;
    if (g_stabCount < WEIGH_STAB_WINDOW) g_stabCount++;
    if (g_stabCount < 2) return NAN;

    float mean = 0.0f;
    for (int i = 0; i < g_stabCount; i++) mean += g_stabBuf[i];
    mean /= g_stabCount;
    float var = 0.0f;
    for (int i = 0; i < g_stabCount; i++) {
        float d = g_stabBuf[i] - mean;
        var += d * d;
    }
    var /= g_stabCount;
    return sqrtf(var);
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

    // ------------------------------------------------------------------
    // Пользовательский уровень: 4 модели, tare, сглаживание, deadband, σ.
    // ------------------------------------------------------------------
    refreshFourModels(cal);
    float wLin = g_calLin.rawToWeight(rawADC);
    float wPw  = g_calPw .rawToWeight(rawADC);
    float wP2  = g_calP2 .rawToWeight(rawADC);
    float wP3  = g_calP3 .rawToWeight(rawADC);

    // weight (основной) идёт из полного конвейера (outlier+temp+nonlinear по cal.method)
    // и является «эталонным» для сглаживания и индикатора стабильности.
    float mainRaw = weight;

    // Захват tare: фиксируем текущее сглаженное значение как ноль.
    if (g_tarePending) {
        float base = isnan(g_smoothedWeight) ? mainRaw : g_smoothedWeight;
        g_tareOffset_g += base; // суммируем, чтобы повторное tare работало
        g_tarePending = false;
        // После tare сбрасываем сглаживание и окно стабильности.
        g_smoothedWeight = NAN;
        g_displayedWeight = NAN;
        g_stabCount = 0; g_stabIdx = 0;
        printTagged("TARE", "Zero captured");
    }

    // Применяем tare ко всем моделям и основному весу.
    float mainTared = mainRaw - g_tareOffset_g;
    wLin -= g_tareOffset_g;
    wPw  -= g_tareOffset_g;
    wP2  -= g_tareOffset_g;
    wP3  -= g_tareOffset_g;

    float smoothed = smoothWeight(mainTared);
    float display  = applyDisplayDeadband(smoothed);
    float sigma_g  = pushAndComputeSigma(smoothed);

    bool stable = (!isnan(sigma_g) && sigma_g < WEIGH_STAB_SIGMA_G && g_stabCount >= WEIGH_STAB_WINDOW);
    const char* stabMark = stable ? "●" : "~";

    // Overload: предупреждение и «защёлка» превышения safe-диапазона.
    float absW = fabsf(display);
    if (absW >= WEIGH_OVERLOAD_G) {
        if (!g_overloadLatched) {
            printTagged("OVERLOAD", "Sensor safe range exceeded (>50g). Remove load!");
            g_overloadLatched = true;
        }
    } else if (absW >= WEIGH_WARN_G) {
        // Предупреждение — раз в несколько тиков, не защёлкиваем.
        static uint32_t lastWarnMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastWarnMs > 2000) {
            printTagged("WARN", "Near sensor max (>=40g)");
            lastWarnMs = nowMs;
        }
        g_overloadLatched = false;
    } else {
        g_overloadLatched = false;
    }

    float sigma_mg = isnan(sigma_g) ? NAN : sigma_g * 1000.0f;
    if (isnan(sigma_mg)) {
        Serial.printf("[WEIGH] [%s] W=%.3fg | LIN=%.3f PW=%.3f P2=%.3f P3=%.3f | RAW=%ld | sigma=--\n",
                      stabMark, display, wLin, wPw, wP2, wP3, (long)rawADC);
    } else {
        Serial.printf("[WEIGH] [%s] W=%.3fg | LIN=%.3f PW=%.3f P2=%.3f P3=%.3f | RAW=%ld | sigma=%.2fmg\n",
                      stabMark, display, wLin, wPw, wP2, wP3, (long)rawADC, sigma_mg);
    }
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
