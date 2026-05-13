#include <Arduino.h>
#include <stdio.h>

#include "7Semi_NAU7802.h"
#include "chopper_control_7semi.h"
#include "calibration_data.h"
#include "calibration_storage.h"
#include "scale_calibration_7semi.h"
#include "scale_init_7semi.h"
#include "scale_weighing_7semi.h"
#include "ui_console_7semi.h"
#include "weigh_runtime_7semi.h"

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

namespace {

void applyWeighTuneAfterChange() {
    myScale.setSampling(weighRuntimeGetReadNumSamples(), weighRuntimeGetReadTimeoutMs());
    resetWeighingFilter();
    char buf[128];
    snprintf(buf,
             sizeof(buf),
             "N=%u T=%lu af=%.2f as=%.2f dG=%.3f db=%.4g F=%s",
             static_cast<unsigned>(weighRuntimeGetReadNumSamples()),
             static_cast<unsigned long>(weighRuntimeGetReadTimeoutMs()),
             weighRuntimeGetAlphaFast(),
             weighRuntimeGetAlphaSlow(),
             weighRuntimeGetBigDeltaG(),
             weighRuntimeGetDeadbandG(),
             weighRuntimeGetFilterName());
    printTagged("TUNE", buf);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(500);

    printTagged("INIT", "Booting...");
    printTagged("INIT", "Starting I2C and NAU7802 (7Semi)...");

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

    if (currentState == STATE_READY && lc == 'd') {
        weighRuntimeCycleCalWizardMult();
        char buf[112];
        snprintf(buf,
                 sizeof(buf),
                 "Wizard readAverage: N=%u T=%lu ms (base 50 x mult %u)",
                 static_cast<unsigned>(weighRuntimeGetCalWizardNumSamples()),
                 static_cast<unsigned long>(weighRuntimeGetCalWizardTimeoutMs()),
                 static_cast<unsigned>(weighRuntimeGetCalWizardMult()));
        printTagged("CAL", buf);
        return;
    }

    if (currentState == STATE_WEIGHING) {
        if (cmd >= '1' && cmd <= '5') {
            const PGA_gain_select prevPga = weighRuntimeGetPga();
            if (!weighRuntimeApplyPgaKey(cmd)) {
                return;
            }
            if (!applyHardwarePreset(myScale)) {
                weighRuntimeSetPga(prevPga);
                (void)applyHardwarePreset(myScale);
                printError("PGA preset: hardware apply failed");
                return;
            }
            printTagged("WARN", "Recalibrate (N) after LDO/PGA change");
            return;
        }
        if (cmd == '6' || cmd == '7' || cmd == '8' || cmd == '9' || cmd == '0') {
            const LDO_Output_Voltage prevLdo = weighRuntimeGetLdo();
            if (!weighRuntimeApplyLdoKey(cmd)) {
                return;
            }
            if (!applyHardwarePreset(myScale)) {
                weighRuntimeSetLdo(prevLdo);
                (void)applyHardwarePreset(myScale);
                printError("LDO preset: hardware apply failed");
                return;
            }
            printTagged("WARN", "Recalibrate (N) after LDO/PGA change");
            return;
        }
        if (cmd == '-') {
            weighRuntimeSetFilterQuiet();
            myScale.setSampling(weighRuntimeGetReadNumSamples(), weighRuntimeGetReadTimeoutMs());
            resetWeighingFilter();
            printTagged("FILTER", "Quiet profile (-)");
            return;
        }
        if (cmd == '=') {
            weighRuntimeSetFilterFast();
            myScale.setSampling(weighRuntimeGetReadNumSamples(), weighRuntimeGetReadTimeoutMs());
            resetWeighingFilter();
            printTagged("FILTER", "Fast profile (=)");
            return;
        }
        if (lc == 'd') {
            weighRuntimeSetFilterDefault();
            myScale.setSampling(weighRuntimeGetReadNumSamples(), weighRuntimeGetReadTimeoutMs());
            resetWeighingFilter();
            printTagged("FILTER", "Default profile (D)");
            return;
        }
        if (lc == 'a') {
            weighRuntimeCycleN();
            applyWeighTuneAfterChange();
            return;
        }
        if (lc == 'i') {
            weighRuntimeCycleReadTimeoutMs();
            applyWeighTuneAfterChange();
            return;
        }
        if (lc == 'f') {
            weighRuntimeCycleAlphaFast();
            applyWeighTuneAfterChange();
            return;
        }
        if (lc == 'w') {
            weighRuntimeCycleAlphaSlow();
            applyWeighTuneAfterChange();
            return;
        }
        if (lc == 'g') {
            weighRuntimeCycleBigDeltaG();
            applyWeighTuneAfterChange();
            return;
        }
        if (lc == 'b') {
            weighRuntimeCycleDeadbandG();
            applyWeighTuneAfterChange();
            return;
        }
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
        case 'j':
            if (cycleChopperMode(myScale)) {
                printTagged("CHOP", "Runtime chopper delay mode updated");
            }
            break;
        case 'u':
            printRegisterDump();
            break;
        default:
            break;
    }
}
