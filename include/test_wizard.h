#pragma once
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include "calibration_data.h"

// Индексы тестов (state machine)
enum WizardTestId : uint8_t {
    WIZARD_TEST_CALIBRATION  = 0,  // Test 1: методы калибровки
    WIZARD_TEST_STABILITY    = 1,  // Test 2: стабильность (0g, 10g, 20g, 30g)
    WIZARD_TEST_OUTLIER      = 2,  // Test 3: фильтр выбросов
    WIZARD_TEST_ADAPTIVE     = 3,  // Test 4: адаптивное усреднение
    WIZARD_TEST_COMPARISON   = 4,  // Test 5: сравнение методов (5g, 15g, 25g)
    WIZARD_TEST_TEMPERATURE  = 5,  // Test 6: температурная компенсация
    WIZARD_TEST_COUNT        = 6
};

// Результаты теста стабильности для одного веса
struct StabilityResult {
    float min;
    float max;
    float mean;
    float stddev;  // мг
    bool  valid;
};

// Полная структура результатов всех тестов
struct TestResults {
    // Test 1: Calibration
    float r2_linear;
    float r2_piecewise;
    float r2_poly2;
    float r2_poly3;

    // Test 2: Stability (0g, 10g, 20g, 30g)
    StabilityResult stability[4];

    // Test 3: Outlier filter
    uint32_t outliers_baseline;
    uint32_t outliers_interference;
    uint32_t outliers_filtered;

    // Test 4: Adaptive averaging
    int  samples_stable;
    int  samples_changing;
    int  transition_measurements;

    // Test 5: Calibration comparison (5g, 15g, 25g) × 4 methods
    float comparison_weights[3][4];  // [weight_idx][method_idx]

    // Test 6: Temperature compensation
    float temp_baseline;
    float temp_changed;
    float weight_baseline;
    float weight_no_comp;
    float weight_with_comp;

    // Метаданные
    uint32_t timestamp_start;
    uint8_t  tests_completed;   // битовая маска пройденных тестов
    uint8_t  tests_skipped;     // битовая маска пропущенных тестов
};

// Класс Test Wizard с полным state machine
class TestWizard {
public:
    TestWizard();

    // Запуск/управление
    void start(NAU7802& scale, CalibrationData& cal);
    void resume(NAU7802& scale, CalibrationData& cal);
    void skipCurrent();
    void reset();

    // Текущий тест
    WizardTestId getCurrentTest() const;
    bool isFinished() const;

    // Отчёт
    void printReport() const;
    void exportCSV() const;

    // Индивидуальные тесты (можно вызывать напрямую)
    void runCalibrationTest(NAU7802& scale, CalibrationData& cal);
    void runStabilityTest(NAU7802& scale, const CalibrationData& cal);
    void runOutlierTest(NAU7802& scale, const CalibrationData& cal);
    void runAdaptiveTest(NAU7802& scale, const CalibrationData& cal);
    void runComparisonTest(NAU7802& scale, const CalibrationData& cal);
    void runTemperatureTest(NAU7802& scale, const CalibrationData& cal);

private:
    TestResults     results_;
    WizardTestId    currentTest_;
    bool            finished_;

    // Вспомогательные методы
    void printHeader(uint8_t testNum, const char* title) const;
    void printProgress(int current, int total) const;
    void waitForEnter(const char* prompt) const;
    float collectWeightSamples(NAU7802& scale, const CalibrationData& cal, int numSamples) const;
    void computeStabilityStats(const float* measurements, int count, StabilityResult& result) const;
    void markTestDone(WizardTestId id);
    void markTestSkipped(WizardTestId id);
    void runNextTest(NAU7802& scale, CalibrationData& cal);
};
