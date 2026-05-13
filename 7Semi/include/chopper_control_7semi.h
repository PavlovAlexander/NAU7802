#pragma once

#include "7Semi_NAU7802.h"

#include <stdint.h>

// Runtime control for NAU7802 ADC register 0x15.
// Datasheet Rev 2.1:
// - REG0x15[5:4] REG_CHPS: recommended value is 0b11.
// - REG0x15[1:0] REG_CHP: delay between ADC CLKSD and ADC chopper clock CHP_CLKSD.

bool initChopperControl();
bool enableInternalAvdd();
bool readNauRegister(uint8_t reg, uint8_t& value);
bool writeNauRegister(uint8_t reg, uint8_t value);
bool dumpAndValidatePowerState();
bool applyChopperDelayMode(uint8_t mode);
bool cycleChopperMode(NAU7802_7Semi& scale);
void printRegisterDump();
uint8_t getChopperDelayMode();
