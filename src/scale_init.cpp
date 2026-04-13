#include "scale_init.h"
#include "ui_console.h"
#include <Wire.h>
#include <Arduino.h>

bool initHardware(NAU7802& scale) {
    printTagged("INIT", "Wire.begin...");
    Wire.begin(21, 22);
    Wire.setClock(100000);  // 100 кГц вместо 400 - надёжнее для длинных проводов

    // I2C scan
    printTagged("INIT", "Scanning I2C bus...");
    int deviceCount = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            char buf[32];
            snprintf(buf, sizeof(buf), "Found device at 0x%02X", addr);
            printTagged("INIT", buf);
            deviceCount++;
        }
    }
    if (deviceCount == 0) {
        printError("No I2C devices found! Check wiring and pullups.");
    }

    printTagged("INIT", "scale.begin...");
    if (!scale.begin(Wire)) {
        printError("NAU7802 not found at 0x2A");
        return false;
    }

    printTagged("INIT", "setGain...");
    scale.setGain(NAU7802_GAIN_128);
    printTagged("INIT", "setSampleRate...");
    scale.setSampleRate(NAU7802_SPS_80);  // 80 SPS — баланс между точностью и скоростью

    // Чоппер отключён (CLK_CHP = 0b11) — с чоппером разброс ±500 мг, без него ±2 мг

    printTagged("INIT", "NAU7802 initialized OK");
    return true;
}
