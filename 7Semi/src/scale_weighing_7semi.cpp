#include "scale_weighing_7semi.h"

#include "ui_console_7semi.h"

#include <Arduino.h>
#include <math.h>

namespace {
constexpr float WEIGH_DEADBAND_G = 0.002f;
constexpr float WEIGH_ALPHA_FAST = 0.70f;
constexpr float WEIGH_ALPHA_SLOW = 0.15f;
constexpr float WEIGH_BIG_DELTA_G = 0.030f;

bool g_tarePending = false;
float g_tareOffset = 0.0f;
float g_smoothed = NAN;
float g_displayed = NAN;

float smoothWeight(float raw) {
    if (isnan(g_smoothed)) {
        g_smoothed = raw;
        return raw;
    }
    const float delta = fabsf(raw - g_smoothed);
    const float alpha = (delta > WEIGH_BIG_DELTA_G) ? WEIGH_ALPHA_FAST : WEIGH_ALPHA_SLOW;
    g_smoothed += alpha * (raw - g_smoothed);
    return g_smoothed;
}

float applyDeadband(float smoothed) {
    if (isnan(g_displayed)) {
        g_displayed = smoothed;
        return smoothed;
    }
    if (fabsf(smoothed - g_displayed) >= WEIGH_DEADBAND_G) {
        g_displayed = smoothed;
    }
    return g_displayed;
}
}  // namespace

void requestTare() {
    g_tarePending = true;
}

void clearTare() {
    g_tarePending = false;
    g_tareOffset = 0.0f;
    resetWeighingFilter();
}

void resetWeighingFilter() {
    g_smoothed = NAN;
    g_displayed = NAN;
}

float getTareOffset() {
    return g_tareOffset;
}

float piecewiseWeightFromRaw(const CalibrationData& cal, int32_t rawADC) {
    const CalibrationPoint* p = cal.points;
    if (rawADC <= p[0].rawADC) return p[0].weight;
    if (rawADC >= p[3].rawADC) return p[3].weight;

    for (int i = 0; i < 3; i++) {
        const int32_t x0 = p[i].rawADC;
        const int32_t x1 = p[i + 1].rawADC;
        if (rawADC >= x0 && rawADC <= x1) {
            if (x1 == x0) return p[i].weight;
            const float t = static_cast<float>(rawADC - x0) / static_cast<float>(x1 - x0);
            return p[i].weight + t * (p[i + 1].weight - p[i].weight);
        }
    }
    return p[0].weight;
}

void weighingTick(NAU7802_7Semi& scale, const CalibrationData& cal) {
    int32_t rawADC = 0;
    if (!scale.readAverage(rawADC, 16, 1200)) {
        printError("Weight read timeout");
        delay(50);
        return;
    }

    float weight = piecewiseWeightFromRaw(cal, rawADC);
    if (g_tarePending) {
        g_tareOffset += weight;
        g_tarePending = false;
        resetWeighingFilter();
        printTagged("TARE", "Zero captured");
    }
    weight -= g_tareOffset;

    const float smoothed = smoothWeight(weight);
    const float display = applyDeadband(smoothed);
    Serial.printf("[WEIGH] W=%.3fg | RAW=%ld\n", display, static_cast<long>(rawADC));
}
