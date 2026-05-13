#include "chopper_control_7semi.h"

#include "scale_weighing_7semi.h"
#include "ui_console_7semi.h"

#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t NAU7802_ADDR = 0x2A;
constexpr uint8_t NAU_REG_PU_CTRL = 0x00;
constexpr uint8_t NAU_REG_CTRL1 = 0x01;
constexpr uint8_t NAU_REG_CTRL2 = 0x02;
constexpr uint8_t NAU_REG_ADC_CTRL1 = 0x15;
constexpr uint8_t NAU_REG_ADC_CTRL3 = 0x1B;
constexpr uint8_t NAU_REG_PWR_CTRL = 0x1C;

constexpr uint8_t REG_CHPS_MASK = 0x30;  // REG0x15[5:4], datasheet recommended 0b11.
constexpr uint8_t REG_CHP_MASK = 0x03;   // REG0x15[1:0], ADC/chopper clock delay.
constexpr uint8_t REG_CHPS_RECOMMENDED = 0x30;

uint8_t g_chopperDelayMode = 0;

void printReg(const char* name, uint8_t reg) {
    uint8_t value = 0;
    if (readNauRegister(reg, value)) {
        Serial.printf("[REG] %s(0x%02X)=0x%02X\n", name, reg, value);
    } else {
        Serial.printf("[REG] %s(0x%02X)=ERR\n", name, reg);
    }
}
}  // namespace

bool readNauRegister(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(NAU7802_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(NAU7802_ADDR, static_cast<uint8_t>(1)) != 1) {
        return false;
    }
    value = Wire.read();
    return true;
}

bool writeNauRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(NAU7802_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool enableInternalAvdd() {
    uint8_t value = 0;
    if (!readNauRegister(NAU_REG_PU_CTRL, value)) {
        printError("Failed to read PU_CTRL");
        return false;
    }

    const uint8_t nextValue = value | 0x80;  // PU_CTRL[7] AVDDS: use internal LDO for AVDD.
    if (!writeNauRegister(NAU_REG_PU_CTRL, nextValue)) {
        printError("Failed to enable AVDDS");
        return false;
    }

    delay(300);

    uint8_t readback = 0;
    if (!readNauRegister(NAU_REG_PU_CTRL, readback)) {
        printError("Failed to read PU_CTRL after AVDDS");
        return false;
    }

    Serial.printf("[REG] PU_CTRL=0x%02X (AVDDS=%u)\n",
                  readback,
                  (readback & 0x80) ? 1 : 0);
    return (readback & 0x80) != 0;
}

bool dumpAndValidatePowerState() {
    printRegisterDump();

    uint8_t puCtrl = 0;
    if (!readNauRegister(NAU_REG_PU_CTRL, puCtrl)) {
        printError("Failed to validate PU_CTRL");
        return false;
    }

    if ((puCtrl & 0x80) == 0) {
        printError("PU_CTRL AVDDS bit is not set");
        return false;
    }
    return true;
}

bool initChopperControl() {
    uint8_t value = 0;
    if (!readNauRegister(NAU_REG_ADC_CTRL1, value)) {
        printError("Failed to read ADC_CTRL1");
        return false;
    }

    // Keep the datasheet-recommended REG_CHPS bits at 0b11 and preserve current delay bits.
    const uint8_t normalized = (value | REG_CHPS_RECOMMENDED);
    if (normalized != value && !writeNauRegister(NAU_REG_ADC_CTRL1, normalized)) {
        printError("Failed to normalize ADC_CTRL1 REG_CHPS bits");
        return false;
    }

    g_chopperDelayMode = normalized & REG_CHP_MASK;
    Serial.printf("[CHOP] ADC_CTRL1=0x%02X REG_CHPS=3 REG_CHP=%u\n",
                  normalized,
                  g_chopperDelayMode);
    return true;
}

bool applyChopperDelayMode(uint8_t mode) {
    uint8_t value = 0;
    if (!readNauRegister(NAU_REG_ADC_CTRL1, value)) {
        printError("Failed to read ADC_CTRL1");
        return false;
    }

    const uint8_t nextMode = mode & REG_CHP_MASK;
    const uint8_t nextValue = (value & ~REG_CHP_MASK) | REG_CHPS_RECOMMENDED | nextMode;
    if (!writeNauRegister(NAU_REG_ADC_CTRL1, nextValue)) {
        printError("Failed to write ADC_CTRL1");
        return false;
    }

    g_chopperDelayMode = nextMode;
    Serial.printf("[CHOP] ADC_CTRL1=0x%02X REG_CHPS=3 REG_CHP=%u\n",
                  nextValue,
                  g_chopperDelayMode);
    return true;
}

bool cycleChopperMode(NAU7802_7Semi& scale) {
    const uint8_t nextMode = (g_chopperDelayMode + 1) & REG_CHP_MASK;
    if (!applyChopperDelayMode(nextMode)) {
        return false;
    }

    delay(300);
    resetWeighingFilter();

    int32_t discarded = 0;
    for (int i = 0; i < 6; i++) {
        (void)scale.readRaw(discarded, 250);
    }

    printTagged("CHOP", "Settle and sample flush complete");
    return true;
}

void printRegisterDump() {
    printReg("PU_CTRL", NAU_REG_PU_CTRL);
    printReg("CTRL1", NAU_REG_CTRL1);
    printReg("CTRL2", NAU_REG_CTRL2);
    printReg("ADC_CTRL1", NAU_REG_ADC_CTRL1);
    printReg("ADC_CTRL3", NAU_REG_ADC_CTRL3);
    printReg("PWR_CTRL", NAU_REG_PWR_CTRL);
}

uint8_t getChopperDelayMode() {
    return g_chopperDelayMode;
}
