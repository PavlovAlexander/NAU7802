#include "scale_calibration_7semi.h"

#include "calibration_storage.h"
#include "ui_console_7semi.h"

#include <Arduino.h>

namespace {
constexpr int32_t ADC_SATURATION_THRESHOLD = 8388000;
constexpr int32_t CALIBRATION_MIN_RAW_SPAN = 10000;

float computeLinearR2(const CalibrationPoint* p, int n, float k, float b) {
    if (n < 2) return 0.0f;
    float meanY = 0.0f;
    for (int i = 0; i < n; i++) meanY += p[i].weight;
    meanY /= n;

    float ssTot = 0.0f;
    float ssRes = 0.0f;
    for (int i = 0; i < n; i++) {
        const float pred = k * static_cast<float>(p[i].rawADC) + b;
        const float dy = p[i].weight - meanY;
        const float er = p[i].weight - pred;
        ssTot += dy * dy;
        ssRes += er * er;
    }
    if (ssTot <= 1e-12f) return 1.0f;
    return 1.0f - (ssRes / ssTot);
}

bool collectPoint(NAU7802_7Semi& scale, float targetWeight, CalibrationPoint& outPoint) {
    char msg[80];
    snprintf(msg, sizeof(msg), "Place %.0fg and press ENTER", targetWeight);
    printTagged("CALIB", msg);

    while (Serial.available() > 0) Serial.read();
    while (true) {
        if (Serial.available() > 0) {
            const char c = static_cast<char>(Serial.read());
            if (c == '\n' || c == '\r') break;
        }
        delay(10);
    }
    delay(80);
    while (Serial.available() > 0) Serial.read();

    int32_t avgRaw = 0;
    if (!scale.readAverage(avgRaw, 50, 2000)) {
        printError("Failed to collect ADC samples");
        return false;
    }
    if (avgRaw <= -ADC_SATURATION_THRESHOLD || avgRaw >= ADC_SATURATION_THRESHOLD) {
        printError("ADC saturated: check AVDD/excitation/register init");
        snprintf(msg, sizeof(msg), "Saturated raw=%ld for %.0fg", static_cast<long>(avgRaw), targetWeight);
        printTagged("CALIB", msg);
        return false;
    }
    outPoint.rawADC = avgRaw;
    outPoint.weight = targetWeight;
    snprintf(msg, sizeof(msg), "Captured raw=%ld for %.0fg", static_cast<long>(avgRaw), targetWeight);
    printTagged("CALIB", msg);
    return true;
}

}  // namespace

bool runCalibrationWizard(NAU7802_7Semi& scale, CalibrationData& cal) {
    const float targets[4] = {0.0f, 10.0f, 20.0f, 30.0f};
    for (int i = 0; i < 4; i++) {
        if (!collectPoint(scale, targets[i], cal.points[i])) {
            return false;
        }
    }

    // Храним линейную оценку только как служебные коэффициенты для статуса.
    const float x0 = static_cast<float>(cal.points[0].rawADC);
    const float x3 = static_cast<float>(cal.points[3].rawADC);
    const float y0 = cal.points[0].weight;
    const float y3 = cal.points[3].weight;
    if (fabsf(x3 - x0) < static_cast<float>(CALIBRATION_MIN_RAW_SPAN)) {
        printError("Calibration failed: raw span too small");
        return false;
    }
    cal.k = (y3 - y0) / (x3 - x0);
    cal.b = y0 - cal.k * x0;
    cal.r2 = computeLinearR2(cal.points, 4, cal.k, cal.b);
    cal.timestamp = millis();
    cal.method = CAL_PIECEWISE_LINEAR;

    saveCalibration(cal);
    printTagged("CALIB", "Calibration saved (PIECEWISE default)");
    return true;
}
