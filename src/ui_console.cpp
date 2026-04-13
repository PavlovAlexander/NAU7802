#include "ui_console.h"
#include <Arduino.h>

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
