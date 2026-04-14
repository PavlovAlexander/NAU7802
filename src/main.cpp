#include <Arduino.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

#include "calibration_data.h"
#include "calibration_storage.h"
#include "scale_init.h"
#include "scale_calibration.h"
#include "scale_weighing.h"
#include "ui_console.h"

// ---------------------------------------------------------------------------
// FSM state
// ---------------------------------------------------------------------------
enum AppState {
    STATE_INIT,
    STATE_READY,
    STATE_CALIBRATING,
    STATE_WEIGHING,
    STATE_ERROR
};

// ---------------------------------------------------------------------------
// Global instances
// ---------------------------------------------------------------------------
NAU7802 myScale;
CalibrationData  calibration;
AppState         currentState = STATE_INIT;

// ---------------------------------------------------------------------------
// Forward declaration
// ---------------------------------------------------------------------------
void handleCommand(char cmd);

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500); // дать Serial время подняться

    printTagged("INIT", "Booting...");
    printTagged("INIT", "Starting I2C and NAU7802...");

    if (initHardware(myScale)) {
        currentState = STATE_READY;
        
        // Инициализация компонентов точности
        printTagged("INIT", "Initializing accuracy components...");
        initAccuracyComponents();
    } else {
        currentState = STATE_ERROR;
    }
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    // Read and dispatch any incoming serial command
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }

    switch (currentState) {

        case STATE_READY: {
            static bool calCheckDone = false;
            static bool readyMsgShown = false;
            static bool hasCalibration = false;
            
            if (!calCheckDone) {
                hasCalibration = loadCalibration(calibration);
                calCheckDone = true;
                
                if (hasCalibration) {
                    // Калибровка найдена — предложить выбор
                    if (!readyMsgShown) {
                        printTagged("INIT", "Calibration found in memory");
                        printTagged("INIT", "Press 'L' to LOAD calibration and start weighing");
                        printTagged("INIT", "Press 'N' to start NEW calibration");
                        readyMsgShown = true;
                    }
                } else {
                    // Калибровки нет — сразу предложить калибровку
                    if (!readyMsgShown) {
                        printTagged("INIT", "No calibration found. Press ENTER to calibrate");
                        readyMsgShown = true;
                    }
                }
            }
            delay(100);
            break;
        }

        case STATE_CALIBRATING: {
            if (runCalibrationWizard(myScale, calibration)) {
                currentState = STATE_WEIGHING;
            }
            // if false: wizard already printed error, stay in STATE_CALIBRATING
            break;
        }

        case STATE_WEIGHING: {
            static bool helpShown = false;
            if (!helpShown) {
                printTagged("INFO", "Commands: C=calibrate | S=status | R=restart");
                helpShown = true;
            }
            weighingTick(myScale, calibration);
            // Без задержки — 16 отсчётов @ 80 SPS = ~200 мс → 5 выводов/сек
            break;
        }

        case STATE_ERROR: {
            delay(5000);
            break;
        }

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// handleCommand()
// ---------------------------------------------------------------------------
void handleCommand(char cmd) {
    switch (cmd) {
        case 'c':
        case 'C':
            currentState = STATE_CALIBRATING;
            break;

        case 'l':
        case 'L':
            if (currentState == STATE_READY) {
                // Загрузить калибровку и перейти к взвешиванию
                if (loadCalibration(calibration)) {
                    printTagged("INIT", "Calibration loaded. Starting weighing...");
                    currentState = STATE_WEIGHING;
                } else {
                    printError("Failed to load calibration");
                }
            }
            break;

        case 'n':
        case 'N':
            if (currentState == STATE_READY) {
                // Начать новую калибровку
                printTagged("INIT", "Starting new calibration...");
                currentState = STATE_CALIBRATING;
            }
            break;

        case 'r':
        case 'R':
            ESP.restart();
            break;

        case 's':
        case 'S':
            if (currentState == STATE_WEIGHING) {
                printStatus(calibration);
            }
            break;

        case '\n':
        case '\r':
            if (currentState == STATE_READY) {
                currentState = STATE_CALIBRATING;
            }
            // STATE_CALIBRATING ENTER is handled inside runCalibrationWizard
            break;

        default:
            break;
    }
}
