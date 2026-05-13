#pragma once

#include "7Semi_NAU7802.h"

#include <stdint.h>

/** Boot defaults and live PGA/LDO/filter parameters for 7Semi weighing. */
void weighRuntimeInitDefaults();

LDO_Output_Voltage weighRuntimeGetLdo();
PGA_gain_select weighRuntimeGetPga();

uint8_t weighRuntimeGetReadNumSamples();
unsigned long weighRuntimeGetReadTimeoutMs();
float weighRuntimeGetAlphaFast();
float weighRuntimeGetAlphaSlow();
float weighRuntimeGetBigDeltaG();
float weighRuntimeGetDeadbandG();
const char* weighRuntimeGetFilterName();

bool weighRuntimeApplyPgaKey(char key);
bool weighRuntimeApplyLdoKey(char key);
void weighRuntimeSetPga(PGA_gain_select pga);
void weighRuntimeSetLdo(LDO_Output_Voltage ldo);
void weighRuntimeSetFilterDefault();
void weighRuntimeSetFilterQuiet();
void weighRuntimeSetFilterFast();

/** Live tuning (WEIGHING); sets filter label to "custom". */
void weighRuntimeCycleN();
void weighRuntimeCycleReadTimeoutMs();
void weighRuntimeCycleAlphaFast();
void weighRuntimeCycleAlphaSlow();
void weighRuntimeCycleBigDeltaG();
void weighRuntimeCycleDeadbandG();

/** READY only: cycle calibration wizard sample multiplier 1 -> 2 -> 3 -> 4 -> 1 (base 50 samples). */
void weighRuntimeCycleCalWizardMult();
uint8_t weighRuntimeGetCalWizardNumSamples();
unsigned long weighRuntimeGetCalWizardTimeoutMs();
uint8_t weighRuntimeGetCalWizardMult();

const char* weighRuntimePgaLabel(PGA_gain_select pga);
const char* weighRuntimeLdoLabel(LDO_Output_Voltage ldo);

/** Applies current runtime LDO/PGA: setConfig, CRS_80, setSampling, AVDD, chopper, settle, flush. */
bool applyHardwarePreset(NAU7802_7Semi& scale);
