#include "ui_console_7semi.h"

#include <Arduino.h>

void printTagged(const char* tag, const char* msg) {
    Serial.printf("[%s] %s\n", tag, msg);
}

void printError(const char* msg) {
    printTagged("ERROR", msg);
}

void printStatus(const CalibrationData& cal) {
    const char* method = (cal.method == CAL_PIECEWISE_LINEAR) ? "PIECEWISE" : "LINEAR";
    Serial.println("=== Calibration Status ===");
    Serial.printf("Method: %s\n", method);
    Serial.printf("R2: %.6f\n", cal.r2);
    Serial.printf("Timestamp: %lu\n", static_cast<unsigned long>(cal.timestamp));
    for (int i = 0; i < 4; i++) {
        Serial.printf("P%d: raw=%ld, weight=%.3fg\n",
                      i,
                      static_cast<long>(cal.points[i].rawADC),
                      cal.points[i].weight);
    }
    Serial.println("==========================");
}

void printHelp() {
    Serial.println();
    printTagged("HELP", "======== Key Commands ========");
    printTagged("HELP", "H - this help");
    printTagged("HELP", "R - restart device");
    printTagged("HELP", "--- READY state ---");
    printTagged("HELP", "L - load calibration & start weighing");
    printTagged("HELP", "N - new calibration");
    printTagged("HELP", "--- WEIGHING state ---");
    printTagged("HELP", "Q - exit weighing, back to menu");
    printTagged("HELP", "0 - tare (zero current reading)");
    printTagged("HELP", "C - recalibrate");
    printTagged("HELP", "S - show calibration status");
    printTagged("HELP", "J - cycle ADC chopper delay mode (REG0x15[1:0])");
    printTagged("HELP", "U - print NAU7802 register dump");
    printTagged("HELP", "==============================");
}
