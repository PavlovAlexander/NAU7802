#pragma once

#include "7Semi_NAU7802.h"

#include <stdint.h>

/** Fixed hardware and weighing pipeline (final7semi). Enum codes: PGA=7 (p128), LDO=4 (ldo33). */
constexpr PGA_gain_select kFixedPga = PGA_128;
constexpr LDO_Output_Voltage kFixedLdo = LDO_3_3V;

constexpr uint8_t kFixedReadNumSamples = 22;
constexpr unsigned long kFixedReadTimeoutMs = 4000UL;

constexpr float kFixedAlphaFast = 0.80f;
constexpr float kFixedAlphaSlow = 0.50f;
constexpr float kFixedBigDeltaG = 0.005f;
constexpr float kFixedDeadbandG = 0.001f;

/** REG0x15[1:0] chopper delay mode applied at init. */
constexpr uint8_t kFixedChopperDelayMode = 0;

/** Avoirdupois: 1 grain = 64.79891 mg. grains = grams * (1000 / 64.79891). */
constexpr float kGramsToGrains = 1000.0f / 64.79891f;

/** Calibration wizard collectPoint (unchanged from classic 7Semi). */
constexpr uint8_t kCalWizardSamples = 50;
constexpr unsigned long kCalWizardTimeoutMs = 2000UL;
