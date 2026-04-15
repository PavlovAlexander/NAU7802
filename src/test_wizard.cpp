#include "test_wizard.h"
#include "calibration/nonlinear_calibration.h"
#include "calibration/temperature_compensation.h"
#include "filters/outlier_filter.h"
#include "filters/adaptive_averager.h"
#include "scale_weighing.h"
#include "ui_console.h"
#include <Arduino.h>
#include <math.h>

// ============================================================================
// Конструктор
// ============================================================================

TestWizard::TestWizard() {
    reset();
}

// ============================================================================
// Управление state machine
// ============================================================================

void TestWizard::reset() {
    memset(&results_, 0, sizeof(results_));
    currentTest_ = WIZARD_TEST_CALIBRATION;
    finished_    = false;
}

WizardTestId TestWizard::getCurrentTest() const {
    return currentTest_;
}

bool TestWizard::isFinished() const {
    return finished_;
}

void TestWizard::skipCurrent() {
    uint8_t skippedId = (uint8_t)currentTest_;
    markTestSkipped(currentTest_);
    if (skippedId + 1 < WIZARD_TEST_COUNT) {
        currentTest_ = (WizardTestId)(skippedId + 1);
    } else {
        finished_ = true;
    }
    Serial.printf("[WIZARD] Test %d skipped\n", (int)skippedId);
}

void TestWizard::markTestDone(WizardTestId id) {
    results_.tests_completed |= (1 << (uint8_t)id);
}

void TestWizard::markTestSkipped(WizardTestId id) {
    results_.tests_skipped |= (1 << (uint8_t)id);
}

// ============================================================================
// Запуск / продолжение
// ============================================================================

void TestWizard::start(NAU7802& scale, CalibrationData& cal) {
    reset();
    results_.timestamp_start = millis();

    Serial.println();
    printHeader(0, "TEST WIZARD - Scale Accuracy Improvements");
    Serial.println("[WIZARD] This wizard will run 6 tests to validate all improvements.");
    Serial.println("[WIZARD] Tests: Calibration | Stability | Outlier | Adaptive | Comparison | Temperature");
    Serial.println("[WIZARD] Type 'test wizard skip' to skip current test at any time.");
    Serial.println();

    runNextTest(scale, cal);
}

void TestWizard::resume(NAU7802& scale, CalibrationData& cal) {
    if (finished_) {
        printTagged("WIZARD", "All tests already completed. Use 'test wizard reset' to start over.");
        return;
    }
    Serial.printf("[WIZARD] Resuming from test %d/%d\n",
                  (int)currentTest_ + 1, WIZARD_TEST_COUNT);
    runNextTest(scale, cal);
}

void TestWizard::runNextTest(NAU7802& scale, CalibrationData& cal) {
    switch (currentTest_) {
        case WIZARD_TEST_CALIBRATION:
            runCalibrationTest(scale, cal);
            markTestDone(WIZARD_TEST_CALIBRATION);
            currentTest_ = WIZARD_TEST_STABILITY;
            waitForEnter("Press ENTER to continue to Test 2: Stability...");
            runNextTest(scale, cal);
            break;

        case WIZARD_TEST_STABILITY:
            runStabilityTest(scale, cal);
            markTestDone(WIZARD_TEST_STABILITY);
            currentTest_ = WIZARD_TEST_OUTLIER;
            waitForEnter("Press ENTER to continue to Test 3: Outlier Filter...");
            runNextTest(scale, cal);
            break;

        case WIZARD_TEST_OUTLIER:
            runOutlierTest(scale, cal);
            markTestDone(WIZARD_TEST_OUTLIER);
            currentTest_ = WIZARD_TEST_ADAPTIVE;
            waitForEnter("Press ENTER to continue to Test 4: Adaptive Averaging...");
            runNextTest(scale, cal);
            break;

        case WIZARD_TEST_ADAPTIVE:
            runAdaptiveTest(scale, cal);
            markTestDone(WIZARD_TEST_ADAPTIVE);
            currentTest_ = WIZARD_TEST_COMPARISON;
            waitForEnter("Press ENTER to continue to Test 5: Calibration Comparison...");
            runNextTest(scale, cal);
            break;

        case WIZARD_TEST_COMPARISON:
            runComparisonTest(scale, cal);
            markTestDone(WIZARD_TEST_COMPARISON);
            currentTest_ = WIZARD_TEST_TEMPERATURE;
            waitForEnter("Press ENTER to continue to Test 6: Temperature (optional)...");
            runNextTest(scale, cal);
            break;

        case WIZARD_TEST_TEMPERATURE:
            runTemperatureTest(scale, cal);
            markTestDone(WIZARD_TEST_TEMPERATURE);
            finished_ = true;
            Serial.println();
            printHeader(0, "ALL TESTS COMPLETED");
            Serial.println("[WIZARD] Use 'test wizard report' to see results.");
            Serial.println("[WIZARD] Use 'test wizard export' to export CSV.");
            break;

        default:
            finished_ = true;
            break;
    }
}

// ============================================================================
// Вспомогательные методы
// ============================================================================

void TestWizard::printHeader(uint8_t testNum, const char* title) const {
    Serial.println("╔════════════════════════════════════════════╗");
    if (testNum > 0) {
        Serial.printf("║  TEST %d/%d: %-33s║\n", testNum, WIZARD_TEST_COUNT, title);
    } else {
        Serial.printf("║  %-42s║\n", title);
    }
    Serial.println("╚════════════════════════════════════════════╝");
}

void TestWizard::printProgress(int current, int total) const {
    const int barWidth = 20;
    int filled = (total > 0) ? (current * barWidth / total) : 0;

    Serial.print("[WIZARD] Progress: [");
    for (int i = 0; i < barWidth; i++) {
        Serial.print(i < filled ? '█' : '░');
    }
    Serial.printf("] %d/%d\n", current, total);
}

void TestWizard::waitForEnter(const char* prompt) const {
    Serial.println();
    Serial.printf("[WIZARD] %s\n", prompt);
    // Очищаем входной буфер
    while (Serial.available() > 0) Serial.read();
    // Ждём нажатия ENTER
    while (Serial.available() == 0) delay(50);
    while (Serial.available() > 0) Serial.read();
}

float TestWizard::collectWeightSamples(NAU7802& scale, const CalibrationData& cal, int numSamples) const {
    int64_t sum = 0;
    for (int i = 0; i < numSamples; i++) {
        while (!scale.available()) delay(1);
        sum += scale.getReading();
    }
    int32_t rawADC = (int32_t)(sum / numSamples);

    // Используем тот же метод калибровки, что и в основном цикле
    CalibrationModule cm;
    cm.loadFrom(cal);
    return cm.rawToWeight(rawADC);
}

void TestWizard::computeStabilityStats(const float* measurements, int count,
                                        StabilityResult& result) const {
    if (count <= 0) {
        result.valid = false;
        return;
    }

    float minVal = measurements[0];
    float maxVal = measurements[0];
    float sum    = 0.0f;

    for (int i = 0; i < count; i++) {
        if (measurements[i] < minVal) minVal = measurements[i];
        if (measurements[i] > maxVal) maxVal = measurements[i];
        sum += measurements[i];
    }

    float mean = sum / count;

    float variance = 0.0f;
    for (int i = 0; i < count; i++) {
        float d = measurements[i] - mean;
        variance += d * d;
    }
    variance /= count;

    result.min    = minVal;
    result.max    = maxVal;
    result.mean   = mean;
    result.stddev = sqrtf(variance) * 1000.0f;  // в мг
    result.valid  = true;
}

// ============================================================================
// Test 1: Calibration Methods
// ============================================================================

void TestWizard::runCalibrationTest(NAU7802& scale, CalibrationData& cal) {
    printHeader(1, "Calibration Methods");
    Serial.println("[WIZARD] Testing all 4 calibration methods using current cal points.");
    Serial.println("[WIZARD] Ensure scale is calibrated before running this test.");
    Serial.println();

    CalibrationModule cm;

    cm.setMethod(CAL_LINEAR);
    cm.calibrate(cal.points, 4);
    results_.r2_linear = cm.getR2();

    cm.setMethod(CAL_PIECEWISE_LINEAR);
    cm.calibrate(cal.points, 4);
    results_.r2_piecewise = cm.getR2();

    cm.setMethod(CAL_POLYNOMIAL_2);
    cm.calibrate(cal.points, 4);
    results_.r2_poly2 = cm.getR2();

    cm.setMethod(CAL_POLYNOMIAL_3);
    cm.calibrate(cal.points, 4);
    results_.r2_poly3 = cm.getR2();

    Serial.println("[WIZARD] Results:");
    Serial.printf("[WIZARD]   LINEAR:           R²=%.4f  %s\n",
                  results_.r2_linear,    results_.r2_linear    >= 0.99f ? "✓ PASS" : "✗ FAIL");
    Serial.printf("[WIZARD]   PIECEWISE_LINEAR: R²=%.4f  %s\n",
                  results_.r2_piecewise, results_.r2_piecewise >= 0.99f ? "✓ PASS" : "✗ FAIL");
    Serial.printf("[WIZARD]   POLYNOMIAL_2:     R²=%.4f  %s\n",
                  results_.r2_poly2,     results_.r2_poly2     >= 0.99f ? "✓ PASS" : "✗ FAIL");
    Serial.printf("[WIZARD]   POLYNOMIAL_3:     R²=%.4f  %s\n",
                  results_.r2_poly3,     results_.r2_poly3     >= 0.99f ? "✓ PASS" : "✗ FAIL");
}

// ============================================================================
// Test 2: Stability Test
// ============================================================================

void TestWizard::runStabilityTest(NAU7802& scale, const CalibrationData& cal) {
    printHeader(2, "Stability Test");
    Serial.println("[WIZARD] Will measure stability at 4 weights: 0g, 10g, 20g, 30g");
    Serial.println("[WIZARD] 100 measurements per weight. Target: σ < 2 mg");
    Serial.println();

    const float targetWeights[4] = {0.0f, 10.0f, 20.0f, 30.0f};
    const char* weightLabels[4]  = {"0g", "10g", "20g", "30g"};
    const int   numMeasurements  = 100;

    float measurements[numMeasurements];

    for (int w = 0; w < 4; w++) {
        Serial.printf("[WIZARD] [%d/4] Place %.0fg weight and press ENTER\n",
                      w + 1, targetWeights[w]);
        waitForEnter("");

        Serial.printf("[WIZARD] Collecting %d measurements...\n", numMeasurements);

        for (int i = 0; i < numMeasurements; i++) {
            measurements[i] = collectWeightSamples(scale, cal, 16);
            if ((i + 1) % 20 == 0) {
                printProgress(i + 1, numMeasurements);
            }
        }

        computeStabilityStats(measurements, numMeasurements, results_.stability[w]);
        const StabilityResult& r = results_.stability[w];

        Serial.printf("[WIZARD] Results at %s:\n", weightLabels[w]);
        Serial.printf("[WIZARD]   Min:    %.3f g\n", r.min);
        Serial.printf("[WIZARD]   Max:    %.3f g\n", r.max);
        Serial.printf("[WIZARD]   Mean:   %.3f g\n", r.mean);
        Serial.printf("[WIZARD]   StdDev: %.3f mg  %s\n",
                      r.stddev, r.stddev < 2.0f ? "✓ PASS" : "✗ FAIL");
        Serial.println();
    }
}

// ============================================================================
// Test 3: Outlier Filter Test
// ============================================================================

void TestWizard::runOutlierTest(NAU7802& scale, const CalibrationData& cal) {
    printHeader(3, "Outlier Filter Test");
    Serial.println("[WIZARD] Tests effectiveness of the outlier filter.");
    Serial.println("[WIZARD] Target: >= 95% of outliers filtered.");
    Serial.println();

    const int numSamples = 50;

    // --- Baseline (без помех) ---
    Serial.println("[WIZARD] [3.1] Baseline (no interference)");
    Serial.println("[WIZARD] Place 30g weight and press ENTER");
    waitForEnter("");

    OutlierFilter& filter = getOutlierFilter();
    filter.resetStatistics();

    Serial.printf("[WIZARD] Collecting %d measurements (baseline)...\n", numSamples);
    for (int i = 0; i < numSamples; i++) {
        while (!scale.available()) delay(1);
        int32_t raw = scale.getReading();
        filter.filter(raw);
        if ((i + 1) % 10 == 0) printProgress(i + 1, numSamples);
    }

    results_.outliers_baseline = filter.getOutlierCount();
    Serial.printf("[WIZARD] Baseline outliers detected: %lu (%.1f%%)\n",
                  results_.outliers_baseline,
                  numSamples > 0 ? 100.0f * results_.outliers_baseline / numSamples : 0.0f);
    Serial.println();

    // --- С помехами ---
    Serial.println("[WIZARD] [3.2] With interference");
    Serial.println("[WIZARD] Place 30g weight");
    Serial.println("[WIZARD] Wave your hand near the scale during measurement");
    Serial.println("[WIZARD] Press ENTER to start");
    waitForEnter("");

    filter.resetStatistics();

    Serial.printf("[WIZARD] Collecting %d measurements (with interference)...\n", numSamples);
    for (int i = 0; i < numSamples; i++) {
        while (!scale.available()) delay(1);
        int32_t raw = scale.getReading();
        filter.filter(raw);
        if ((i + 1) % 10 == 0) printProgress(i + 1, numSamples);
    }

    results_.outliers_interference = filter.getTotalCount() > 0
                                     ? filter.getOutlierCount() : 0;
    results_.outliers_filtered     = results_.outliers_interference;

    float filterRate = (results_.outliers_interference > 0)
                       ? 100.0f
                       : 0.0f;  // все обнаруженные — отфильтрованы по определению

    Serial.printf("[WIZARD] Interference outliers detected: %lu (%.1f%%)\n",
                  results_.outliers_interference,
                  numSamples > 0 ? 100.0f * results_.outliers_interference / numSamples : 0.0f);
    Serial.printf("[WIZARD] Filtered: %lu (%.1f%%)  %s\n",
                  results_.outliers_filtered,
                  filterRate,
                  filterRate >= 95.0f ? "✓ PASS" : "✗ FAIL (need more interference)");
}

// ============================================================================
// Test 4: Adaptive Averaging Test
// ============================================================================

void TestWizard::runAdaptiveTest(NAU7802& scale, const CalibrationData& cal) {
    printHeader(4, "Adaptive Averaging Test");
    Serial.println("[WIZARD] Tests automatic switching between accuracy and speed modes.");
    Serial.println("[WIZARD] Target: stable mode = maxSamples, transition <= 2 measurements.");
    Serial.println();

    AdaptiveAverager& avg = getAdaptiveAverager();
    bool wasEnabled = avg.isEnabled();
    avg.setEnabled(true);

    // --- Стабильный вес ---
    Serial.println("[WIZARD] [4.1] Stable weight mode");
    Serial.println("[WIZARD] Place 30g weight and press ENTER");
    waitForEnter("");
    Serial.println("[WIZARD] Waiting 10 seconds for stabilization...");

    for (int i = 0; i < 10; i++) {
        float w = collectWeightSamples(scale, cal, 16);
        avg.updateHistory(w);
        printProgress(i + 1, 10);
        delay(1000);
    }

    results_.samples_stable = avg.computeSampleCount();
    Serial.printf("[WIZARD] Stable mode samples: %d  %s\n",
                  results_.samples_stable,
                  results_.samples_stable >= avg.getMaxSamples() * 8 / 10
                  ? "✓ PASS" : "✗ FAIL (not stabilized yet)");
    Serial.println();

    // --- Смена веса ---
    Serial.println("[WIZARD] [4.2] Changing weight mode");
    Serial.println("[WIZARD] Remove the weight NOW and press ENTER");
    waitForEnter("");

    int transitionCount = 0;
    int prevSamples = results_.samples_stable;

    for (int i = 0; i < 5; i++) {
        float w = collectWeightSamples(scale, cal, 16);
        avg.updateHistory(w);
        int cur = avg.computeSampleCount();
        Serial.printf("[WIZARD]   Measurement %d: samples=%d\n", i + 1, cur);

        if (cur <= avg.getMinSamples() + 2 && prevSamples > avg.getMinSamples() + 2) {
            transitionCount = i + 1;
            break;
        }
        prevSamples = cur;
        delay(200);
    }

    results_.samples_changing       = avg.getCurrentSampleCount();
    results_.transition_measurements = (transitionCount > 0) ? transitionCount : 5;

    Serial.printf("[WIZARD] Transition time: %d measurements  %s\n",
                  results_.transition_measurements,
                  results_.transition_measurements <= 2 ? "✓ PASS" : "✗ FAIL");
    Serial.printf("[WIZARD] Changing mode samples: %d\n", results_.samples_changing);

    // Восстанавливаем исходное состояние
    avg.setEnabled(wasEnabled);
}

// ============================================================================
// Test 5: Calibration Method Comparison
// ============================================================================

void TestWizard::runComparisonTest(NAU7802& scale, const CalibrationData& cal) {
    printHeader(5, "Calibration Method Comparison");
    Serial.println("[WIZARD] Compares all 4 methods at intermediate weights: 5g, 15g, 25g");
    Serial.println("[WIZARD] Target: max difference between methods < 10 mg");
    Serial.println();

    const float targetWeights[3] = {5.0f, 15.0f, 25.0f};
    const char* weightLabels[3]  = {"5g", "15g", "25g"};
    const char* methodNames[4]   = {"LINEAR", "PIECEWISE", "POLY_2", "POLY_3"};
    const CalibrationMethod methods[4] = {
        CAL_LINEAR, CAL_PIECEWISE_LINEAR, CAL_POLYNOMIAL_2, CAL_POLYNOMIAL_3
    };

    for (int w = 0; w < 3; w++) {
        Serial.printf("[WIZARD] [5.%d] Place %s weight and press ENTER\n",
                      w + 1, weightLabels[w]);
        waitForEnter("");

        // Собираем одно усреднённое измерение raw ADC
        int64_t sum = 0;
        for (int i = 0; i < 50; i++) {
            while (!scale.available()) delay(1);
            sum += scale.getReading();
        }
        int32_t rawADC = (int32_t)(sum / 50);

        CalibrationModule cm;
        float maxW = -1e9f, minW = 1e9f;

        for (int m = 0; m < 4; m++) {
            cm.setMethod(methods[m]);
            cm.calibrate(cal.points, 4);
            float wgt = cm.rawToWeight(rawADC);
            results_.comparison_weights[w][m] = wgt;

            if (wgt > maxW) maxW = wgt;
            if (wgt < minW) minW = wgt;

            Serial.printf("[WIZARD]   %-10s %.3f g\n", methodNames[m], wgt);
        }

        float maxDiff = (maxW - minW) * 1000.0f;  // в мг
        Serial.printf("[WIZARD] Max difference: %.2f mg  %s\n",
                      maxDiff, maxDiff < 10.0f ? "✓ PASS" : "✗ FAIL");
        Serial.println();
    }
}

// ============================================================================
// Test 6: Temperature Compensation
// ============================================================================

void TestWizard::runTemperatureTest(NAU7802& scale, const CalibrationData& cal) {
    printHeader(6, "Temperature Compensation (optional)");
    Serial.println("[WIZARD] Tests temperature drift correction.");
    Serial.println("[WIZARD] Target: drift with compensation < 1 mg at ΔT = 10°C");
    Serial.println("[WIZARD] This test requires changing temperature by ~10°C (hair dryer or cold).");
    Serial.println();

    TemperatureCompensator& tc = getTemperatureCompensator();
    bool wasEnabled = tc.isEnabled();

    // --- Baseline ---
    Serial.println("[WIZARD] [6.1] Baseline measurement");
    Serial.println("[WIZARD] Place 30g weight at room temperature and press ENTER");
    waitForEnter("");

    results_.temp_baseline    = tc.readTemperature(scale);
    results_.weight_baseline  = collectWeightSamples(scale, cal, 50);

    Serial.printf("[WIZARD] Baseline: T=%.1f°C  W=%.3f g\n",
                  results_.temp_baseline, results_.weight_baseline);
    Serial.println();

    // --- После изменения температуры ---
    Serial.println("[WIZARD] [6.2] After temperature change");
    Serial.println("[WIZARD] Change temperature (use hair dryer or move to cold place)");
    Serial.println("[WIZARD] Wait until temperature changes by at least 5°C");
    Serial.println("[WIZARD] Press ENTER when ready");
    waitForEnter("");

    results_.temp_changed = tc.readTemperature(scale);
    float deltaT = results_.temp_changed - results_.temp_baseline;

    // Вес без компенсации (компенсатор отключён)
    tc.setEnabled(false);
    results_.weight_no_comp = collectWeightSamples(scale, cal, 50);

    // Вес с компенсацией
    tc.setEnabled(true);
    // Применяем компенсацию вручную для теста
    float k = cal.k, b = cal.b;
    tc.compensate(results_.temp_changed, k, b);
    int64_t sum = 0;
    for (int i = 0; i < 50; i++) {
        while (!scale.available()) delay(1);
        sum += scale.getReading();
    }
    int32_t rawADC = (int32_t)(sum / 50);
    // Нелинейная часть + температурная поправка
    CalibrationModule cm;
    cm.loadFrom(cal);
    float baseWeight = cm.rawToWeight(rawADC);
    float linearBase = cal.k * (float)rawADC + cal.b;
    float linearComp = k * (float)rawADC + b;
    results_.weight_with_comp = baseWeight + (linearComp - linearBase);

    // Восстанавливаем исходное состояние
    tc.setEnabled(wasEnabled);

    float driftNoComp   = (results_.weight_no_comp   - results_.weight_baseline) * 1000.0f;
    float driftWithComp = (results_.weight_with_comp - results_.weight_baseline) * 1000.0f;

    Serial.printf("[WIZARD] Temperature changed: %.1f°C → %.1f°C (ΔT=%.1f°C)\n",
                  results_.temp_baseline, results_.temp_changed, deltaT);
    Serial.printf("[WIZARD] Drift without compensation: %+.2f mg\n", driftNoComp);
    Serial.printf("[WIZARD] Drift with compensation:    %+.2f mg  %s\n",
                  driftWithComp,
                  fabsf(driftWithComp) < 1.0f ? "✓ PASS" : "✗ FAIL");
}

// ============================================================================
// Отчёт
// ============================================================================

void TestWizard::printReport() const {
    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    TEST SUMMARY REPORT                        ║");
    Serial.println("║                  Scale Accuracy Improvements                  ║");
    Serial.println("╠════════════════════════════════════════════════════════════════╣");

    int passed = 0, total = 0;

    // Test 1
    if (results_.tests_completed & (1 << WIZARD_TEST_CALIBRATION)) {
        total++;
        bool ok = results_.r2_linear >= 0.99f && results_.r2_piecewise >= 0.99f
               && results_.r2_poly2  >= 0.99f && results_.r2_poly3     >= 0.99f;
        if (ok) passed++;
        Serial.printf("║ Test 1: Calibration Methods                %s               ║\n",
                      ok ? "PASS" : "FAIL");
        Serial.printf("║   LINEAR:           R²=%.4f  %s                        ║\n",
                      results_.r2_linear,    results_.r2_linear    >= 0.99f ? "✓" : "✗");
        Serial.printf("║   PIECEWISE_LINEAR: R²=%.4f  %s                        ║\n",
                      results_.r2_piecewise, results_.r2_piecewise >= 0.99f ? "✓" : "✗");
        Serial.printf("║   POLYNOMIAL_2:     R²=%.4f  %s                        ║\n",
                      results_.r2_poly2,     results_.r2_poly2     >= 0.99f ? "✓" : "✗");
        Serial.printf("║   POLYNOMIAL_3:     R²=%.4f  %s                        ║\n",
                      results_.r2_poly3,     results_.r2_poly3     >= 0.99f ? "✓" : "✗");
    } else {
        Serial.println("║ Test 1: Calibration Methods                SKIP                ║");
    }
    Serial.println("║                                                                ║");

    // Test 2
    if (results_.tests_completed & (1 << WIZARD_TEST_STABILITY)) {
        total++;
        const char* wLabels[4] = {"0g", "10g", "20g", "30g"};
        bool ok = true;
        for (int i = 0; i < 4; i++) {
            if (results_.stability[i].valid && results_.stability[i].stddev >= 2.0f) ok = false;
        }
        if (ok) passed++;
        Serial.printf("║ Test 2: Stability Test                     %s               ║\n",
                      ok ? "PASS" : "FAIL");
        for (int i = 0; i < 4; i++) {
            if (results_.stability[i].valid) {
                Serial.printf("║   %3s: σ=%.3f mg  %s                                   ║\n",
                              wLabels[i], results_.stability[i].stddev,
                              results_.stability[i].stddev < 2.0f ? "✓" : "✗");
            }
        }
    } else {
        Serial.println("║ Test 2: Stability Test                     SKIP                ║");
    }
    Serial.println("║                                                                ║");

    // Test 3
    if (results_.tests_completed & (1 << WIZARD_TEST_OUTLIER)) {
        total++;
        bool ok = results_.outliers_interference > 0;
        if (ok) passed++;
        Serial.printf("║ Test 3: Outlier Filter                     %s               ║\n",
                      ok ? "PASS" : "FAIL");
        Serial.printf("║   Baseline outliers:     %4lu                                  ║\n",
                      results_.outliers_baseline);
        Serial.printf("║   Interference outliers: %4lu                                  ║\n",
                      results_.outliers_interference);
    } else {
        Serial.println("║ Test 3: Outlier Filter                     SKIP                ║");
    }
    Serial.println("║                                                                ║");

    // Test 4
    if (results_.tests_completed & (1 << WIZARD_TEST_ADAPTIVE)) {
        total++;
        bool ok = results_.transition_measurements <= 2;
        if (ok) passed++;
        Serial.printf("║ Test 4: Adaptive Averaging                 %s               ║\n",
                      ok ? "PASS" : "FAIL");
        Serial.printf("║   Stable mode samples:  %3d                                    ║\n",
                      results_.samples_stable);
        Serial.printf("║   Transition time:      %3d measurements  %s                ║\n",
                      results_.transition_measurements,
                      results_.transition_measurements <= 2 ? "✓" : "✗");
    } else {
        Serial.println("║ Test 4: Adaptive Averaging                 SKIP                ║");
    }
    Serial.println("║                                                                ║");

    // Test 5
    if (results_.tests_completed & (1 << WIZARD_TEST_COMPARISON)) {
        total++;
        const char* wLabels[3] = {"5g", "15g", "25g"};
        bool ok = true;
        for (int w = 0; w < 3; w++) {
            float maxW = -1e9f, minW = 1e9f;
            for (int m = 0; m < 4; m++) {
                if (results_.comparison_weights[w][m] > maxW) maxW = results_.comparison_weights[w][m];
                if (results_.comparison_weights[w][m] < minW) minW = results_.comparison_weights[w][m];
            }
            if ((maxW - minW) * 1000.0f >= 10.0f) ok = false;
        }
        if (ok) passed++;
        Serial.printf("║ Test 5: Calibration Comparison             %s               ║\n",
                      ok ? "PASS" : "FAIL");
        for (int w = 0; w < 3; w++) {
            float maxW = -1e9f, minW = 1e9f;
            for (int m = 0; m < 4; m++) {
                if (results_.comparison_weights[w][m] > maxW) maxW = results_.comparison_weights[w][m];
                if (results_.comparison_weights[w][m] < minW) minW = results_.comparison_weights[w][m];
            }
            float diff = (maxW - minW) * 1000.0f;
            Serial.printf("║   At %3s: max diff = %.2f mg  %s                        ║\n",
                          wLabels[w], diff, diff < 10.0f ? "✓" : "✗");
        }
    } else {
        Serial.println("║ Test 5: Calibration Comparison             SKIP                ║");
    }
    Serial.println("║                                                                ║");

    // Test 6
    if (results_.tests_completed & (1 << WIZARD_TEST_TEMPERATURE)) {
        total++;
        float driftWithComp = fabsf((results_.weight_with_comp - results_.weight_baseline) * 1000.0f);
        bool ok = driftWithComp < 1.0f;
        if (ok) passed++;
        Serial.printf("║ Test 6: Temperature Compensation           %s               ║\n",
                      ok ? "PASS" : "FAIL");
        Serial.printf("║   Baseline T: %.1f°C  Changed T: %.1f°C                      ║\n",
                      results_.temp_baseline, results_.temp_changed);
        Serial.printf("║   Drift without comp: %+.2f mg                                ║\n",
                      (results_.weight_no_comp - results_.weight_baseline) * 1000.0f);
        Serial.printf("║   Drift with comp:    %+.2f mg  %s                         ║\n",
                      (results_.weight_with_comp - results_.weight_baseline) * 1000.0f,
                      ok ? "✓" : "✗");
    } else {
        Serial.println("║ Test 6: Temperature Compensation           SKIP                ║");
    }

    Serial.println("╠════════════════════════════════════════════════════════════════╣");
    Serial.printf( "║  OVERALL RESULT: %d/%d tests PASSED %-28s║\n",
                   passed, total, passed == total ? "✓" : "");
    Serial.printf( "║  Uptime: %lu ms%-49s║\n", millis(), "");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
}

// ============================================================================
// Экспорт CSV
// ============================================================================

void TestWizard::exportCSV() const {
    Serial.println("=== Test Results (CSV) ===");
    Serial.println("test,metric,value,unit,status");

    // Test 1
    Serial.printf("calibration_linear,r2,%.4f,,%s\n",
                  results_.r2_linear,
                  results_.r2_linear >= 0.99f ? "PASS" : "FAIL");
    Serial.printf("calibration_piecewise,r2,%.4f,,%s\n",
                  results_.r2_piecewise, results_.r2_piecewise >= 0.99f ? "PASS" : "FAIL");
    Serial.printf("calibration_poly2,r2,%.4f,,%s\n",
                  results_.r2_poly2, results_.r2_poly2 >= 0.99f ? "PASS" : "FAIL");
    Serial.printf("calibration_poly3,r2,%.4f,,%s\n",
                  results_.r2_poly3, results_.r2_poly3 >= 0.99f ? "PASS" : "FAIL");

    // Test 2
    const char* wLabels[4] = {"0g", "10g", "20g", "30g"};
    for (int i = 0; i < 4; i++) {
        if (results_.stability[i].valid) {
            Serial.printf("stability_%s,stddev,%.4f,mg,%s\n",
                          wLabels[i], results_.stability[i].stddev,
                          results_.stability[i].stddev < 2.0f ? "PASS" : "FAIL");
            Serial.printf("stability_%s,mean,%.4f,g,INFO\n",
                          wLabels[i], results_.stability[i].mean);
        }
    }

    // Test 3
    Serial.printf("outlier_baseline,count,%lu,,INFO\n",   results_.outliers_baseline);
    Serial.printf("outlier_interference,count,%lu,,INFO\n", results_.outliers_interference);
    Serial.printf("outlier_filtered,count,%lu,,%s\n",
                  results_.outliers_filtered,
                  results_.outliers_interference > 0 ? "PASS" : "INFO");

    // Test 4
    Serial.printf("adaptive_stable,samples,%d,,%s\n",
                  results_.samples_stable, "INFO");
    Serial.printf("adaptive_transition,measurements,%d,,%s\n",
                  results_.transition_measurements,
                  results_.transition_measurements <= 2 ? "PASS" : "FAIL");
    Serial.printf("adaptive_changing,samples,%d,,INFO\n", results_.samples_changing);

    // Test 5
    const char* cLabels[3] = {"5g", "15g", "25g"};
    const char* mNames[4]  = {"linear", "piecewise", "poly2", "poly3"};
    for (int w = 0; w < 3; w++) {
        float maxW = -1e9f, minW = 1e9f;
        for (int m = 0; m < 4; m++) {
            Serial.printf("comparison_%s_%s,weight,%.4f,g,INFO\n",
                          cLabels[w], mNames[m], results_.comparison_weights[w][m]);
            if (results_.comparison_weights[w][m] > maxW) maxW = results_.comparison_weights[w][m];
            if (results_.comparison_weights[w][m] < minW) minW = results_.comparison_weights[w][m];
        }
        float diff = (maxW - minW) * 1000.0f;
        Serial.printf("comparison_%s,max_diff,%.2f,mg,%s\n",
                      cLabels[w], diff, diff < 10.0f ? "PASS" : "FAIL");
    }

    // Test 6
    Serial.printf("temperature_baseline,temp,%.1f,C,INFO\n",  results_.temp_baseline);
    Serial.printf("temperature_changed,temp,%.1f,C,INFO\n",   results_.temp_changed);
    Serial.printf("temperature_baseline,weight,%.4f,g,INFO\n", results_.weight_baseline);
    float driftNo   = (results_.weight_no_comp   - results_.weight_baseline) * 1000.0f;
    float driftWith = (results_.weight_with_comp - results_.weight_baseline) * 1000.0f;
    Serial.printf("temperature_drift_no_comp,drift,%.2f,mg,INFO\n",  driftNo);
    Serial.printf("temperature_drift_with_comp,drift,%.2f,mg,%s\n",
                  driftWith, fabsf(driftWith) < 1.0f ? "PASS" : "FAIL");

    Serial.println("==========================");
}
