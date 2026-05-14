#include <Arduino.h>

#include "7Semi_NAU7802.h"
#include "chopper_control_7semi.h"
#include "calibration_data.h"
#include "calibration_storage.h"
#include "scale_calibration_7semi.h"
#include "scale_init_7semi.h"
#include "scale_weighing_7semi.h"
#include "ui_console_7semi.h"

enum AppState {
    STATE_INIT,
    STATE_READY,
    STATE_CALIBRATING,
    STATE_WEIGHING,
    STATE_ERROR
};

NAU7802_7Semi myScale;
CalibrationData calibration = {};
AppState currentState = STATE_INIT;

void handleSingleChar(char cmd);

void setup() {
    Serial.begin(115200);
    delay(500);

    printTagged("INIT", "Booting...");
    printTagged("INIT", "Starting I2C and NAU7802 (final7semi)...");

    if (initHardware(myScale)) {
        currentState = STATE_READY;
    } else {
        currentState = STATE_ERROR;
        printTagged("INIT", "Hardware init failed; keys H/R still available.");
    }

    printHelp();
}

void loop() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c != '\r' && c != '\n') {
            handleSingleChar(c);
        }
    }

    switch (currentState) {
        case STATE_READY: {
            static bool calCheckDone = false;
            if (!calCheckDone) {
                const bool hasCalibration = loadCalibration(calibration);
                calCheckDone = true;
                if (hasCalibration) {
                    printTagged("INIT", "Calibration found in memory");
                    printTagged("INIT", "Press 'L' to LOAD calibration and start weighing");
                    printTagged("INIT", "Press 'N' to start NEW calibration");
                } else {
                    printTagged("INIT", "No calibration found. Press 'N' to calibrate");
                }
            }
            delay(100);
            break;
        }
        case STATE_CALIBRATING:
            if (runCalibrationWizard(myScale, calibration)) {
                clearTare();
                currentState = STATE_WEIGHING;
                printTagged("INFO", "Calibration done. Weighing started.");
            } else {
                printError("Calibration failed");
                currentState = STATE_READY;
            }
            break;
        case STATE_WEIGHING:
            weighingTick(myScale, calibration);
            break;
        case STATE_ERROR:
            delay(3000);
            break;
        default:
            break;
    }
}

void handleSingleChar(char cmd) {
    const char lc = (cmd >= 'A' && cmd <= 'Z') ? static_cast<char>(cmd + 32) : cmd;

    if (currentState == STATE_WEIGHING) {
        if (lc == 't') {
            requestTare();
            printTagged("TARE", "Tare requested (T)");
            return;
        }
        if (lc == 'z') {
            clearTare();
            printTagged("ZERO", "Tare cleared / filter reset (Z)");
            return;
        }
    }

    switch (lc) {
        case 'h':
            printHelp();
            break;
        case 'r':
            printTagged("SYS", "Restarting...");
            delay(100);
            ESP.restart();
            break;
        case 'l':
            if (currentState == STATE_READY) {
                if (loadCalibration(calibration)) {
                    clearTare();
                    currentState = STATE_WEIGHING;
                    printTagged("INIT", "Calibration loaded. Starting weighing...");
                } else {
                    printError("Failed to load calibration");
                }
            }
            break;
        case 'n':
            if (currentState == STATE_READY) {
                printTagged("INIT", "Starting new calibration...");
                currentState = STATE_CALIBRATING;
            }
            break;
        case 'q':
            if (currentState == STATE_WEIGHING) {
                currentState = STATE_READY;
                printTagged("INFO", "Exited weighing mode. Back to menu.");
            }
            break;
        case 'c':
            if (currentState == STATE_WEIGHING) {
                currentState = STATE_CALIBRATING;
            }
            break;
        case 's':
            if (currentState == STATE_WEIGHING || currentState == STATE_READY) {
                printStatus(calibration);
            }
            break;
        case 'u':
            printRegisterDump();
            break;
        default:
            break;
    }
}
