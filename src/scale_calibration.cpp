#include "scale_calibration.h"
#include "math_utils.h"
#include "calibration_storage.h"
#include "calibration/nonlinear_calibration.h"
#include "ui_console.h"
#include <Arduino.h>

int32_t collectSamples(NAU7802& scale, int numSamples) {
    int64_t sum = 0;
    for (int i = 0; i < numSamples; i++) {
        sum += scale.getReading();
        delay(100);
    }
    return (int32_t)(sum / numSamples);
}

bool runCalibrationWizard(NAU7802& scale, CalibrationData& cal, CalibrationMethod method) {
    const float targetWeights[4] = {0.0f, 10.0f, 20.0f, 30.0f};
    char buf[64];

    // Устанавливаем метод калибровки
    cal.method = method;

    for (int i = 0; i < 4; i++) {
        snprintf(buf, sizeof(buf), "Place %.0fg weight and press ENTER", targetWeights[i]);
        printTagged("CALIB", buf);

        // Очистить буфер Serial от мусора
        while (Serial.available() > 0) {
            Serial.read();
        }

        // Ждать ENTER (блокирующее ожидание)
        bool enterReceived = false;
        while (!enterReceived) {
            if (Serial.available() > 0) {
                char c = Serial.read();
                if (c == '\n' || c == '\r') {
                    enterReceived = true;
                }
            }
            delay(10); // небольшая задержка чтобы не молотить CPU
        }

        // Очистить остатки (если пришло \r\n)
        delay(50);
        while (Serial.available() > 0) {
            Serial.read();
        }

        int32_t rawADC = collectSamples(scale, 50);
        cal.points[i] = {rawADC, targetWeights[i]};

        snprintf(buf, sizeof(buf), "Point %d collected: rawADC = %ld", i, (long)rawADC);
        printTagged("CALIB", buf);
    }

    // Вычисляем коэффициенты в зависимости от метода
    switch (method) {
        case CAL_LINEAR:
            // Линейная регрессия (обратная совместимость)
            linearRegression(cal.points, 4, cal.k, cal.b);
            cal.r2 = computeR2(cal.points, 4, cal.k, cal.b);
            break;
            
        case CAL_PIECEWISE_LINEAR:
        case CAL_POLYNOMIAL_2:
        case CAL_POLYNOMIAL_3: {
            // Используем CalibrationModule для нелинейной калибровки
            CalibrationModule calibModule;
            calibModule.setMethod(method);
            calibModule.calibrate(cal.points, 4);
            calibModule.saveTo(cal);
            cal.r2 = calibModule.getR2();
            break;
        }
            
        default:
            printTagged("CALIB", "Unknown calibration method");
            return false;
    }

    cal.timestamp = millis();

    // Выводим информацию о методе
    const char* methodName = "LINEAR";
    if (method == CAL_PIECEWISE_LINEAR) methodName = "PIECEWISE_LINEAR";
    else if (method == CAL_POLYNOMIAL_2) methodName = "POLYNOMIAL_2";
    else if (method == CAL_POLYNOMIAL_3) methodName = "POLYNOMIAL_3";
    
    snprintf(buf, sizeof(buf), "Method: %s, R2 = %.4f", methodName, cal.r2);
    printTagged("CALIB", buf);

    if (cal.r2 >= 0.99f) {
        saveCalibration(cal);
        printTagged("CALIB", "Calibration saved successfully");
        return true;
    } else {
        printTagged("CALIB", "R2 < 0.99, repeat calibration");
        return false;
    }
}
