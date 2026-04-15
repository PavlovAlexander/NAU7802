#include "ui_console.h"
#include <Arduino.h>
#include <string.h>

void printTagged(const char* tag, const char* msg) {
    Serial.printf("[%s] %s\n", tag, msg);
}

void printWeighLine(int32_t rawADC, float calibrated) {
    Serial.printf("[WEIGH] Raw: %ld | Calibrated: %.3fg\n",
                  (long)rawADC, calibrated);
}

void printStatus(const CalibrationData& cal) {
    Serial.printf("[STAT] I2C: 0x2A | Gain: 128 | k: %.6f | b: %.6f | R2: %.6f | ts: %lu\n",
                  cal.k, cal.b, cal.r2, (unsigned long)cal.timestamp);
}

void printError(const char* msg) {
    Serial.printf("[ERROR] %s\n", msg);
}

// ============================================================================
// Команды управления компонентами точности
// ============================================================================

#include "scale_weighing.h"
#include "config/accuracy_config_manager.h"
#include "config/accuracy_config.h"
#include "filters/outlier_filter.h"
#include "filters/adaptive_averager.h"

void cmd_accuracy_status() {
    AccuracyConfigManager& config = getAccuracyConfig();
    config.printStatus();
}

void cmd_accuracy_enable(const char* component) {
    AccuracyConfigManager& config = getAccuracyConfig();
    
    if (strcmp(component, "outlier") == 0) {
        config.enableOutlierFilter(true);
        printTagged("ACCURACY", "Outlier filter ENABLED");
    } else if (strcmp(component, "temp") == 0) {
        config.enableTemperatureComp(true);
        printTagged("ACCURACY", "Temperature compensation ENABLED");
    } else if (strcmp(component, "adaptive") == 0) {
        config.enableAdaptiveAveraging(true);
        printTagged("ACCURACY", "Adaptive averaging ENABLED");
    } else {
        printError("Unknown component. Use: outlier, temp, adaptive");
    }
}

void cmd_accuracy_disable(const char* component) {
    AccuracyConfigManager& config = getAccuracyConfig();
    
    if (strcmp(component, "outlier") == 0) {
        config.enableOutlierFilter(false);
        printTagged("ACCURACY", "Outlier filter DISABLED");
    } else if (strcmp(component, "temp") == 0) {
        config.enableTemperatureComp(false);
        printTagged("ACCURACY", "Temperature compensation DISABLED");
    } else if (strcmp(component, "adaptive") == 0) {
        config.enableAdaptiveAveraging(false);
        printTagged("ACCURACY", "Adaptive averaging DISABLED");
    } else {
        printError("Unknown component. Use: outlier, temp, adaptive");
    }
}

void cmd_accuracy_set_outlier_method(const char* method) {
    AccuracyConfigManager& config = getAccuracyConfig();
    
    if (strcmp(method, "median") == 0) {
        config.setOutlierMethod(OUTLIER_MEDIAN);
        printTagged("ACCURACY", "Outlier method set to MEDIAN");
    } else if (strcmp(method, "sigma") == 0) {
        config.setOutlierMethod(OUTLIER_SIGMA);
        printTagged("ACCURACY", "Outlier method set to SIGMA");
    } else if (strcmp(method, "none") == 0) {
        config.setOutlierMethod(OUTLIER_NONE);
        printTagged("ACCURACY", "Outlier method set to NONE");
    } else {
        printError("Unknown method. Use: median, sigma, none");
    }
}

void cmd_accuracy_set_outlier_window(uint8_t size) {
    if (size != 3 && size != 5 && size != 7) {
        printError("Window size must be 3, 5, or 7");
        return;
    }
    
    AccuracyConfigManager& config = getAccuracyConfig();
    config.setOutlierWindowSize(size);
    
    char buf[64];
    snprintf(buf, sizeof(buf), "Outlier window size set to %u", size);
    printTagged("ACCURACY", buf);
}

void cmd_accuracy_set_temp_alpha(float value) {
    AccuracyConfigManager& config = getAccuracyConfig();
    float alpha, beta;
    config.getTempCoefficients(alpha, beta);
    config.setTempCoefficients(value, beta);
    
    char buf[64];
    snprintf(buf, sizeof(buf), "Temperature alpha set to %.6f", value);
    printTagged("ACCURACY", buf);
}

void cmd_accuracy_set_temp_beta(float value) {
    AccuracyConfigManager& config = getAccuracyConfig();
    float alpha, beta;
    config.getTempCoefficients(alpha, beta);
    config.setTempCoefficients(alpha, value);
    
    char buf[64];
    snprintf(buf, sizeof(buf), "Temperature beta set to %.2f", value);
    printTagged("ACCURACY", buf);
}

void cmd_accuracy_set_adaptive_threshold(float value) {
    if (value <= 0.0f) {
        printError("Threshold must be positive");
        return;
    }
    
    AccuracyConfigManager& config = getAccuracyConfig();
    float threshold;
    uint8_t minSamples, maxSamples;
    config.getAdaptiveParams(threshold, minSamples, maxSamples);
    config.setAdaptiveParams(value, minSamples, maxSamples, 5);
    
    char buf[64];
    snprintf(buf, sizeof(buf), "Adaptive threshold set to %.2f mg", value);
    printTagged("ACCURACY", buf);
}

void cmd_accuracy_set_adaptive_samples(uint8_t minSamples, uint8_t maxSamples) {
    if (minSamples == 0 || minSamples > maxSamples) {
        printError("Invalid sample range");
        return;
    }
    
    AccuracyConfigManager& config = getAccuracyConfig();
    float threshold;
    uint8_t oldMin, oldMax;
    config.getAdaptiveParams(threshold, oldMin, oldMax);
    config.setAdaptiveParams(threshold, minSamples, maxSamples, 5);
    
    char buf[64];
    snprintf(buf, sizeof(buf), "Adaptive samples set to %u-%u", minSamples, maxSamples);
    printTagged("ACCURACY", buf);
}

void cmd_accuracy_export() {
    AccuracyConfigManager& config = getAccuracyConfig();
    char buffer[512];
    config.exportConfig(buffer, sizeof(buffer));
    
    Serial.println("=== Accuracy Configuration Export ===");
    Serial.print(buffer);
    Serial.println("=====================================");
}

void cmd_accuracy_import(const char* configText) {
    AccuracyConfigManager& config = getAccuracyConfig();
    
    if (config.importConfig(configText)) {
        printTagged("ACCURACY", "Configuration imported successfully");
    } else {
        printError("Failed to import configuration");
    }
}

void cmd_accuracy_import_interactive() {
    // Накапливаем многострочный ввод до пустой строки или тайм-аута 10 сек
    static char importBuf[512];
    size_t totalLen = 0;
    importBuf[0] = '\0';

    printTagged("ACCURACY", "Paste config lines, then send empty line (or wait 10s to finish):");

    const unsigned long kTimeoutMs = 10000UL;
    unsigned long lastActivity = millis();

    while (millis() - lastActivity < kTimeoutMs) {
        if (!Serial.available()) {
            delay(1);
            continue;
        }

        char lineBuf[128];
        size_t lineLen = 0;

        // Читаем одну строку (до \n)
        unsigned long lineStart = millis();
        while (millis() - lineStart < 2000UL) {
            if (!Serial.available()) {
                delay(1);
                continue;
            }
            char c = (char)Serial.read();
            if (c == '\r') continue;
            if (c == '\n') break;
            if (lineLen < sizeof(lineBuf) - 1) {
                lineBuf[lineLen++] = c;
            }
        }
        lineBuf[lineLen] = '\0';

        // Пустая строка — завершаем ввод
        if (lineLen == 0) {
            break;
        }

        // Добавляем строку в общий буфер
        if (totalLen + lineLen + 2 < sizeof(importBuf)) {
            memcpy(importBuf + totalLen, lineBuf, lineLen);
            totalLen += lineLen;
            importBuf[totalLen++] = '\n';
            importBuf[totalLen] = '\0';
        }

        lastActivity = millis();
    }

    if (totalLen == 0) {
        printError("No config data received");
        return;
    }

    AccuracyConfigManager& config = getAccuracyConfig();
    if (config.importConfig(importBuf)) {
        printTagged("ACCURACY", "Configuration imported successfully");
    } else {
        printError("Failed to import configuration — check format");
    }
}

void cmd_accuracy_reset() {
    AccuracyConfigManager& config = getAccuracyConfig();
    config.reset();
    printTagged("ACCURACY", "Configuration reset to defaults");
}

void cmd_accuracy_save() {
    AccuracyConfigManager& config = getAccuracyConfig();
    
    if (config.save()) {
        printTagged("ACCURACY", "Configuration saved to NVS");
    } else {
        printError("Failed to save configuration");
    }
}

// ============================================================================
// Диагностические команды
// ============================================================================

// Глобальный флаг режима диагностики
static bool g_diagnosticMode = false;

void cmd_accuracy_diag_on() {
    g_diagnosticMode = true;
    printTagged("DIAG", "Diagnostic mode ENABLED");
}

void cmd_accuracy_diag_off() {
    g_diagnosticMode = false;
    printTagged("DIAG", "Diagnostic mode DISABLED");
}

bool isDiagnosticMode() {
    return g_diagnosticMode;
}

void cmd_accuracy_stats() {
    OutlierFilter& outlierFilter = getOutlierFilter();
    AdaptiveAverager& adaptiveAvg = getAdaptiveAverager();
    
    Serial.println("=== Accuracy Statistics ===");
    Serial.printf("Uptime: %lu ms\n", millis());
    Serial.println();
    
    Serial.println("Outlier Filter:");
    Serial.printf("  Total samples:     %lu\n", outlierFilter.getTotalCount());
    Serial.printf("  Outliers rejected: %lu\n", outlierFilter.getOutlierCount());
    if (outlierFilter.getTotalCount() > 0) {
        float rejectRate = 100.0f * outlierFilter.getOutlierCount() / outlierFilter.getTotalCount();
        Serial.printf("  Rejection rate:    %.2f%%\n", rejectRate);
    }
    Serial.println();
    
    Serial.println("Adaptive Averager:");
    Serial.printf("  Current samples:   %d\n", adaptiveAvg.getCurrentSampleCount());
    Serial.printf("  Current StdDev:    %.3f mg\n", adaptiveAvg.getCurrentStdDev());
    Serial.println();
    
    Serial.println("===========================");
}

// Примечание: cmd_accuracy_log_export() и cmd_accuracy_log_clear() 
// будут реализованы в Task 12.1 вместе с MeasurementLogger

#include "measurement_logger.h"

// Глобальный экземпляр логгера
static MeasurementLogger g_measurementLogger;

MeasurementLogger& getMeasurementLogger() {
    return g_measurementLogger;
}

void cmd_accuracy_log_export() {
    g_measurementLogger.exportCSV();
}

void cmd_accuracy_log_clear() {
    g_measurementLogger.clear();
}

// ============================================================================
// Тестовые команды
// ============================================================================

#include "calibration/nonlinear_calibration.h"
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

void cmd_accuracy_test_stability(NAU7802& scale, const CalibrationData& cal) {
    printTagged("TEST", "Starting stability test (100 measurements)...");
    
    const int numMeasurements = 100;
    float measurements[numMeasurements];
    
    // Собираем 100 измерений
    for (int i = 0; i < numMeasurements; i++) {
        // Собираем 16 отсчётов
        int64_t sum = 0;
        for (int j = 0; j < 16; j++) {
            while (!scale.available()) {
                delay(1);
            }
            sum += scale.getReading();
        }
        int32_t rawADC = (int32_t)(sum / 16);
        
        // Вычисляем вес
        float weight = cal.k * (float)rawADC + cal.b;
        measurements[i] = weight;
        
        // Прогресс
        if ((i + 1) % 10 == 0) {
            Serial.printf("[TEST] Progress: %d/100\n", i + 1);
        }
    }
    
    // Вычисляем статистику
    float minVal = measurements[0];
    float maxVal = measurements[0];
    float sum = 0.0f;
    
    for (int i = 0; i < numMeasurements; i++) {
        if (measurements[i] < minVal) minVal = measurements[i];
        if (measurements[i] > maxVal) maxVal = measurements[i];
        sum += measurements[i];
    }
    
    float mean = sum / numMeasurements;
    
    // Вычисляем стандартное отклонение
    float variance = 0.0f;
    for (int i = 0; i < numMeasurements; i++) {
        float diff = measurements[i] - mean;
        variance += diff * diff;
    }
    variance /= numMeasurements;
    float stddev = sqrtf(variance);
    
    // Выводим результаты
    Serial.println("=== Stability Test Results ===");
    Serial.printf("Measurements: %d\n", numMeasurements);
    Serial.printf("Min:          %.3f g\n", minVal);
    Serial.printf("Max:          %.3f g\n", maxVal);
    Serial.printf("Mean:         %.3f g\n", mean);
    Serial.printf("StdDev:       %.3f mg\n", stddev * 1000.0f);
    Serial.printf("Range:        %.3f mg\n", (maxVal - minVal) * 1000.0f);
    
    if (stddev * 1000.0f < 2.0f) {
        Serial.println("Status:       PASS (σ < 2 mg)");
    } else {
        Serial.println("Status:       FAIL (σ >= 2 mg)");
    }
    Serial.println("==============================");
}

void cmd_accuracy_compare_calibration(NAU7802& scale, const CalibrationData& cal) {
    printTagged("TEST", "Comparing calibration methods...");
    
    // Собираем одно измерение
    int64_t sum = 0;
    for (int i = 0; i < 50; i++) {
        while (!scale.available()) {
            delay(1);
        }
        sum += scale.getReading();
    }
    int32_t rawADC = (int32_t)(sum / 50);
    
    Serial.println("=== Calibration Method Comparison ===");
    Serial.printf("Raw ADC: %ld\n", (long)rawADC);
    Serial.println();
    
    // Метод 1: Линейная
    CalibrationModule calibModule;
    calibModule.setMethod(CAL_LINEAR);
    calibModule.calibrate(cal.points, 4);
    float weight1 = calibModule.rawToWeight(rawADC);
    Serial.printf("LINEAR:            %.3f g (R²=%.4f)\n", weight1, calibModule.getR2());
    
    // Метод 2: Кусочно-линейная
    calibModule.setMethod(CAL_PIECEWISE_LINEAR);
    calibModule.calibrate(cal.points, 4);
    float weight2 = calibModule.rawToWeight(rawADC);
    Serial.printf("PIECEWISE_LINEAR:  %.3f g (R²=%.4f)\n", weight2, calibModule.getR2());
    
    // Метод 3: Полином 2-й степени
    calibModule.setMethod(CAL_POLYNOMIAL_2);
    calibModule.calibrate(cal.points, 4);
    float weight3 = calibModule.rawToWeight(rawADC);
    Serial.printf("POLYNOMIAL_2:      %.3f g (R²=%.4f)\n", weight3, calibModule.getR2());
    
    // Метод 4: Полином 3-й степени
    calibModule.setMethod(CAL_POLYNOMIAL_3);
    calibModule.calibrate(cal.points, 4);
    float weight4 = calibModule.rawToWeight(rawADC);
    Serial.printf("POLYNOMIAL_3:      %.3f g (R²=%.4f)\n", weight4, calibModule.getR2());
    
    Serial.println();
    Serial.printf("Max difference:    %.3f mg\n", 
                  fabs(fmax(fmax(weight1, weight2), fmax(weight3, weight4)) - 
                       fmin(fmin(weight1, weight2), fmin(weight3, weight4))) * 1000.0f);
    Serial.println("=====================================");
}

// ============================================================================
// Test Wizard команды
// ============================================================================

#include "test_wizard.h"

static TestWizard g_testWizard;

void cmd_test_wizard_start(NAU7802& scale, CalibrationData& cal) {
    g_testWizard.start(scale, cal);
}

void cmd_test_wizard_report() {
    g_testWizard.printReport();
}

void cmd_test_wizard_export() {
    g_testWizard.exportCSV();
}

void cmd_test_wizard_resume(NAU7802& scale, CalibrationData& cal) {
    g_testWizard.resume(scale, cal);
}

void cmd_test_wizard_skip() {
    g_testWizard.skipCurrent();
}

void cmd_test_wizard_reset() {
    g_testWizard.reset();
    printTagged("WIZARD", "Test results reset");
}
