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
    printTagged("HELP", "H - эта справка");
    printTagged("HELP", "R - перезагрузка устройства");
    printTagged("HELP", "--- READY (меню) ---");
    printTagged("HELP", "L - загрузить калибровку и начать взвешивание");
    printTagged("HELP", "N - новая калибровка");
    printTagged("HELP", "D - множитель семплов калибровки 1x..4x (база 50) до старта калибровки");
    printTagged("HELP", "--- WEIGHING (взвешивание) ---");
    printTagged("HELP", "Q - выйти из взвешивания в меню");
    printTagged("HELP", "T - тара: зафиксировать ноль под текущий груз на платформе");
    printTagged("HELP", "Z - сброс тары и программного смещения (zero / clear offset)");
    printTagged("HELP", "PGA: 1=p128(def) 2=p64 3=p32 4=p16 5=p8 — после смены нужна калибровка (N)");
    printTagged("HELP", "LDO: 6=3.3V(def) 7=3.0V 8=3.6V 9=3.9V 0=4.2V — в этом режиме 0 = пресет LDO, не тара");
    printTagged("HELP", "= / - / D - пресеты фильтра fast / quiet / default");
    printTagged("HELP", "A - цикл N (8..32 шаг 2); I - цикл T (1000..5000 шаг 500 ms)");
    printTagged("HELP", "F/W - цикл af / as (0.1..0.9 шаг 0.1); G/B - цикл dG / db (шаги по плану)");
    printTagged("HELP", "C - перекалибровка (мастер)");
    printTagged("HELP", "S - статус калибровки");
    printTagged("HELP", "J - цикл режима задержки chopper ADC (REG0x15[1:0])");
    printTagged("HELP", "U - дамп регистров NAU7802 (PU_CTRL/CTRL1/CTRL2/ADC_CTRL1/...)");
    printTagged("HELP", "==============================");
}
