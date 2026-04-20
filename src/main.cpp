#include <Arduino.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

#include "calibration_data.h"
#include "calibration_storage.h"
#include "scale_init.h"
#include "scale_calibration.h"
#include "scale_weighing.h"
#include "ui_console.h"
#include "config/accuracy_config_manager.h"
#include "hw_char_test.h"

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
// Forward declarations
// ---------------------------------------------------------------------------
void handleSingleChar(char cmd);

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
    // Побайтовый режим: каждый полученный символ обрабатывается немедленно,
    // без накопления строки. Это позволяет управлять устройством одиночными
    // нажатиями клавиш без Enter.
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c != '\r' && c != '\n') {
            handleSingleChar(c);
        }
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
                    if (!readyMsgShown) {
                        printTagged("INIT", "Calibration found in memory");
                        printTagged("INIT", "Press 'L' to LOAD calibration and start weighing");
                        printTagged("INIT", "Press 'N' to start NEW calibration");
                        readyMsgShown = true;
                    }
                } else {
                    if (!readyMsgShown) {
                        printTagged("INIT", "No calibration found. Press 'N' to calibrate");
                        readyMsgShown = true;
                    }
                }
            }
            delay(100);
            break;
        }

        case STATE_CALIBRATING: {
            if (runCalibrationWizard(myScale, calibration)) {
                // Новая калибровка → сбрасываем tare и фильтры пользовательского уровня,
                // сразу переходим в режим взвешивания (показ текущего веса).
                clearTare();
                currentState = STATE_WEIGHING;
            }
            break;
        }

        case STATE_WEIGHING: {
            static bool helpShown = false;
            if (!helpShown) {
                printTagged("INFO", "Weighing started. Press H for key commands.");
                helpShown = true;
            }
            weighingTick(myScale, calibration);
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
// handleSingleChar() — все команды управляются одиночными нажатиями клавиш
//
// Навигация:
//   H  — показать help
//   R  — перезагрузить устройство
//
// Режим READY (ожидание после старта):
//   L  — загрузить калибровку и начать взвешивание
//   N  — новая калибровка
//
// Режим WEIGHING (взвешивание):
//   Q  — выход из режима взвешивания в READY (меню)
//   0  — tare (обнулить текущее показание)
//   C  — перекалибровать
//   S  — показать статус калибровки
//
// Компоненты точности (работают в режиме WEIGHING):
//   A  — статус всех компонентов accuracy
//   D  — переключить диагностический режим (вкл/выкл)
//   E  — экспорт конфигурации в Serial
//   I  — импорт конфигурации (интерактивный режим)
//   W  — сохранить конфигурацию в NVS (ранее 'Q')
//   X  — сбросить конфигурацию к умолчаниям
//   V  — показать статистику (выбросы, усреднение)
//   O  — переключить фильтр выбросов (вкл/выкл)
//
// Test Wizard:
//   T  — запустить / продолжить Test Wizard
//   K  — пропустить текущий тест
//   P  — показать отчёт теста
//
// HW Characterization:
//   M  — запустить тест характеризации железа (gain×SPS×chopper свип + drift + нагрузки)
// ---------------------------------------------------------------------------
void handleSingleChar(char cmd) {
    // Приводим к нижнему регистру для единообразия
    char lc = (cmd >= 'A' && cmd <= 'Z') ? cmd + 32 : cmd;

    switch (lc) {

        // ---------------------------------------------------------------
        // Общие команды (любой режим)
        // ---------------------------------------------------------------
        case 'h':
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
            printTagged("HELP", "--- Accuracy components ---");
            printTagged("HELP", "A - accuracy status");
            printTagged("HELP", "D - toggle diagnostic mode");
            printTagged("HELP", "E - export config to serial");
            printTagged("HELP", "I - import config (interactive)");
            printTagged("HELP", "W - save config to NVS (was Q)");
            printTagged("HELP", "X - reset config to defaults");
            printTagged("HELP", "V - show statistics");
            printTagged("HELP", "O - toggle outlier filter");
            printTagged("HELP", "--- Test Wizard ---");
            printTagged("HELP", "T - start / resume test wizard");
            printTagged("HELP", "K - skip current test");
            printTagged("HELP", "P - print test report");
            printTagged("HELP", "--- HW Characterization ---");
            printTagged("HELP", "M - run HW characterization test (gain x SPS x chopper sweep)");
            printTagged("HELP", "==============================");
            break;

        case 'r':
            printTagged("SYS", "Restarting...");
            delay(100);
            ESP.restart();
            break;

        // ---------------------------------------------------------------
        // Режим READY
        // ---------------------------------------------------------------
        case 'l':
            if (currentState == STATE_READY) {
                if (loadCalibration(calibration)) {
                    printTagged("INIT", "Calibration loaded. Starting weighing...");
                    clearTare();
                    currentState = STATE_WEIGHING;
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

        // ---------------------------------------------------------------
        // Режим WEIGHING — базовые команды
        // ---------------------------------------------------------------
        case 'c':
            currentState = STATE_CALIBRATING;
            break;

        case 's':
            if (currentState == STATE_WEIGHING) {
                printStatus(calibration);
            }
            break;

        // ---------------------------------------------------------------
        // Accuracy компоненты
        // ---------------------------------------------------------------
        case 'a':
            cmd_accuracy_status();
            break;

        case 'd':
            if (isDiagnosticMode()) {
                cmd_accuracy_diag_off();
            } else {
                cmd_accuracy_diag_on();
            }
            break;

        case 'e':
            cmd_accuracy_export();
            break;

        case 'i':
            cmd_accuracy_import_interactive();
            break;

        case 'q':
            // Выход из режима взвешивания обратно в READY (меню).
            if (currentState == STATE_WEIGHING) {
                currentState = STATE_READY;
                printTagged("INFO", "Exited weighing mode. Back to menu.");
                printTagged("INIT", "Press 'L' to LOAD calibration, 'N' for NEW calibration, 'H' for help");
            }
            break;

        case 'w':
            // Сохранить конфигурацию accuracy в NVS (ранее было на 'Q').
            cmd_accuracy_save();
            break;

        case '0':
            // Tare (zeroing): зафиксировать текущий сглаженный вес как ноль.
            if (currentState == STATE_WEIGHING) {
                requestTare();
                printTagged("TARE", "Tare requested (will apply on next tick)");
            }
            break;

        case 'x':
            cmd_accuracy_reset();
            break;

        case 'v':
            cmd_accuracy_stats();
            break;

        case 'o': {
            // Переключаем фильтр выбросов
            AccuracyConfigManager& cfg = getAccuracyConfig();
            bool nowEnabled = !cfg.getConfig().outlierFilterEnabled;
            cfg.enableOutlierFilter(nowEnabled);
            printTagged("ACCURACY", nowEnabled ? "Outlier filter ENABLED" : "Outlier filter DISABLED");
            break;
        }

        // ---------------------------------------------------------------
        // Test Wizard
        // ---------------------------------------------------------------
        case 't':
            cmd_test_wizard_resume(myScale, calibration);
            break;

        case 'k':
            cmd_test_wizard_skip();
            break;

        case 'p':
            cmd_test_wizard_report();
            break;

        // ---------------------------------------------------------------
        // HW Characterization Test
        // ---------------------------------------------------------------
        case 'm':
            runHwCharTest(myScale);
            break;

        default:
            break;
    }
}

