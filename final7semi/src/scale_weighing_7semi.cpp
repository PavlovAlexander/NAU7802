#include "scale_weighing_7semi.h"

#include "fixed_scale_config.h"
#include "scale_init_7semi.h"
#include "ui_console_7semi.h"

#include <Arduino.h>
#include <math.h>

namespace {

constexpr uint8_t WEIGH_READ_ATTEMPTS = 4;
constexpr uint8_t WEIGH_FAIL_STREAK_REINIT = 3;

bool g_tarePending = false;
float g_tareOffset = 0.0f;
float g_smoothed = NAN;
float g_displayed = NAN;
uint8_t g_readFailStreak = 0;

float smoothWeight(float raw) {
    if (isnan(g_smoothed)) {
        g_smoothed = raw;
        return raw;
    }
    const float delta = fabsf(raw - g_smoothed);
    const float alpha = (delta > kFixedBigDeltaG) ? kFixedAlphaFast : kFixedAlphaSlow;
    g_smoothed += alpha * (raw - g_smoothed);
    return g_smoothed;
}

float applyDeadband(float smoothed) {
    if (isnan(g_displayed)) {
        g_displayed = smoothed;
        return smoothed;
    }
    if (fabsf(smoothed - g_displayed) >= kFixedDeadbandG) {
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
    bool readOk = false;
    for (uint8_t attempt = 0; attempt < WEIGH_READ_ATTEMPTS; attempt++) {
        if (scale.readAverage(rawADC, kFixedReadNumSamples, kFixedReadTimeoutMs)) {
            readOk = true;
            break;
        }
        delay(5);
    }

    if (!readOk) {
        g_readFailStreak++;
        Serial.printf("[ERROR] Weight read timeout (ms=%lu streak=%u attempt=%u)\n",
                      static_cast<unsigned long>(millis()),
                      static_cast<unsigned>(g_readFailStreak),
                      static_cast<unsigned>(WEIGH_READ_ATTEMPTS));
        if (g_readFailStreak >= WEIGH_FAIL_STREAK_REINIT) {
            printTagged("RECOVER", "Reinitializing NAU7802 after repeated read timeouts");
            if (initHardware(scale)) {
                g_readFailStreak = 0;
                resetWeighingFilter();
                printTagged("RECOVER", "NAU7802 reinit completed");
            } else {
                printError("NAU7802 reinit failed");
            }
        }
        delay(50);
        return;
    }

    g_readFailStreak = 0;

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
    const float grains = display * kGramsToGrains;
    Serial.printf("[WEIGH] W=%.3fg | RAW=%ld | GRAINS=%.2f\n",
                  display,
                  static_cast<long>(rawADC),
                  grains);
}
