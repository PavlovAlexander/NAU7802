#include "weigh_runtime_7semi.h"

#include "chopper_control_7semi.h"
#include "ui_console_7semi.h"

#include <Arduino.h>
#include <math.h>

namespace {

struct FilterProfile {
    uint8_t numSamples;
    unsigned long readTimeoutMs;
    float alphaFast;
    float alphaSlow;
    float bigDeltaG;
    float deadbandG;
    const char* name;
};

constexpr FilterProfile kFilterDefault = {16,
                                          4000,
                                          0.70f,
                                          0.15f,
                                          0.030f,
                                          0.002f,
                                          "default"};
constexpr FilterProfile kFilterQuiet = {28,
                                        4500,
                                        0.55f,
                                        0.10f,
                                        0.025f,
                                        0.0035f,
                                        "quiet"};
constexpr FilterProfile kFilterFast = {8,
                                       2500,
                                       0.90f,
                                       0.30f,
                                       0.020f,
                                       0.001f,
                                       "fast"};

constexpr PGA_gain_select kPgaByKey1To5[5] = {PGA_128, PGA_64, PGA_32, PGA_16, PGA_8};
constexpr LDO_Output_Voltage kLdoByKey6To0[5] = {
    LDO_3_3V, LDO_3_0V, LDO_3_6V, LDO_3_9V, LDO_4_2V};

constexpr char kCustomLabel[] = "custom";

/** Base sample count for calibration wizard (collectPoint); effective N = base * mult, mult in 1..4. */
constexpr uint8_t kCalWizardBaseSamples = 50;

LDO_Output_Voltage g_ldo = LDO_3_3V;
PGA_gain_select g_pga = PGA_128;

uint8_t g_numSamples = kFilterDefault.numSamples;
unsigned long g_readTimeoutMs = kFilterDefault.readTimeoutMs;
float g_alphaFast = kFilterDefault.alphaFast;
float g_alphaSlow = kFilterDefault.alphaSlow;
float g_bigDeltaG = kFilterDefault.bigDeltaG;
float g_deadbandG = kFilterDefault.deadbandG;
const char* g_filterName = kFilterDefault.name;

/** 1..4; effective calibration samples = kCalWizardBaseSamples * mult (fits uint8_t). */
uint8_t g_calWizardMult = 1;

void applyFilterProfile(const FilterProfile& p) {
    g_numSamples = p.numSamples;
    g_readTimeoutMs = p.readTimeoutMs;
    g_alphaFast = p.alphaFast;
    g_alphaSlow = p.alphaSlow;
    g_bigDeltaG = p.bigDeltaG;
    g_deadbandG = p.deadbandG;
    g_filterName = p.name;
}

void markCustom() {
    g_filterName = kCustomLabel;
}

int clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

}  // namespace

void weighRuntimeInitDefaults() {
    g_ldo = LDO_3_3V;
    g_pga = PGA_128;
    applyFilterProfile(kFilterDefault);
    g_calWizardMult = 1;
}

LDO_Output_Voltage weighRuntimeGetLdo() {
    return g_ldo;
}

PGA_gain_select weighRuntimeGetPga() {
    return g_pga;
}

uint8_t weighRuntimeGetReadNumSamples() {
    return g_numSamples;
}

unsigned long weighRuntimeGetReadTimeoutMs() {
    return g_readTimeoutMs;
}

float weighRuntimeGetAlphaFast() {
    return g_alphaFast;
}

float weighRuntimeGetAlphaSlow() {
    return g_alphaSlow;
}

float weighRuntimeGetBigDeltaG() {
    return g_bigDeltaG;
}

float weighRuntimeGetDeadbandG() {
    return g_deadbandG;
}

const char* weighRuntimeGetFilterName() {
    return g_filterName;
}

bool weighRuntimeApplyPgaKey(char key) {
    if (key < '1' || key > '5') {
        return false;
    }
    const unsigned idx = static_cast<unsigned>(key - '1');
    const PGA_gain_select next = kPgaByKey1To5[idx];
    if (next == g_pga) {
        return false;
    }
    g_pga = next;
    return true;
}

void weighRuntimeSetPga(PGA_gain_select pga) {
    g_pga = pga;
}

void weighRuntimeSetLdo(LDO_Output_Voltage ldo) {
    g_ldo = ldo;
}

bool weighRuntimeApplyLdoKey(char key) {
    unsigned idx = 0;
    if (key == '6') {
        idx = 0;
    } else if (key == '7') {
        idx = 1;
    } else if (key == '8') {
        idx = 2;
    } else if (key == '9') {
        idx = 3;
    } else if (key == '0') {
        idx = 4;
    } else {
        return false;
    }
    const LDO_Output_Voltage next = kLdoByKey6To0[idx];
    if (next == g_ldo) {
        return false;
    }
    g_ldo = next;
    return true;
}

void weighRuntimeSetFilterDefault() {
    applyFilterProfile(kFilterDefault);
}

void weighRuntimeSetFilterQuiet() {
    applyFilterProfile(kFilterQuiet);
}

void weighRuntimeSetFilterFast() {
    applyFilterProfile(kFilterFast);
}

void weighRuntimeCycleN() {
    int v = static_cast<int>(g_numSamples);
    if (v < 8) {
        v = 8;
    }
    if (v > 32) {
        v = 32;
    }
    if ((v & 1) != 0) {
        v--;
    }
    if (v < 8) {
        v = 8;
    }
    v += 2;
    if (v > 32) {
        v = 8;
    }
    g_numSamples = static_cast<uint8_t>(v);
    markCustom();
}

void weighRuntimeCycleReadTimeoutMs() {
    if (g_readTimeoutMs >= 5000UL) {
        g_readTimeoutMs = 1000UL;
    } else {
        g_readTimeoutMs += 500UL;
    }
    markCustom();
}

void weighRuntimeCycleAlphaFast() {
    int t = static_cast<int>(lroundf(g_alphaFast * 10.0f));
    t = clampInt(t, 1, 9);
    t++;
    if (t > 9) {
        t = 1;
    }
    g_alphaFast = static_cast<float>(t) * 0.1f;
    markCustom();
}

void weighRuntimeCycleAlphaSlow() {
    int t = static_cast<int>(lroundf(g_alphaSlow * 10.0f));
    t = clampInt(t, 1, 9);
    t++;
    if (t > 9) {
        t = 1;
    }
    g_alphaSlow = static_cast<float>(t) * 0.1f;
    markCustom();
}

void weighRuntimeCycleBigDeltaG() {
    int s = static_cast<int>(lroundf(g_bigDeltaG / 0.005f));
    if (s < 1 || s > 6) {
        s = 1;
    } else {
        s++;
        if (s > 6) {
            s = 1;
        }
    }
    g_bigDeltaG = 0.005f * static_cast<float>(s);
    markCustom();
}

void weighRuntimeCycleDeadbandG() {
    int s = static_cast<int>(lroundf(g_deadbandG / 0.0005f));
    if (s < 1 || s > 7) {
        s = 1;
    } else {
        s++;
        if (s > 7) {
            s = 1;
        }
    }
    g_deadbandG = 0.0005f * static_cast<float>(s);
    markCustom();
}

void weighRuntimeCycleCalWizardMult() {
    if (g_calWizardMult >= 4) {
        g_calWizardMult = 1;
    } else {
        g_calWizardMult = static_cast<uint8_t>(g_calWizardMult + 1);
    }
}

uint8_t weighRuntimeGetCalWizardMult() {
    return g_calWizardMult;
}

uint8_t weighRuntimeGetCalWizardNumSamples() {
    const unsigned n = static_cast<unsigned>(kCalWizardBaseSamples) * static_cast<unsigned>(g_calWizardMult);
    if (n > 255U) {
        return 255;
    }
    return static_cast<uint8_t>(n);
}

unsigned long weighRuntimeGetCalWizardTimeoutMs() {
    return 2000UL * static_cast<unsigned long>(g_calWizardMult);
}

const char* weighRuntimePgaLabel(PGA_gain_select pga) {
    switch (pga) {
        case PGA_128:
            return "p128";
        case PGA_64:
            return "p64";
        case PGA_32:
            return "p32";
        case PGA_16:
            return "p16";
        case PGA_8:
            return "p8";
        case PGA_4:
            return "p4";
        case PGA_2:
            return "p2";
        case PGA_1:
            return "p1";
        default:
            return "p?";
    }
}

const char* weighRuntimeLdoLabel(LDO_Output_Voltage ldo) {
    switch (ldo) {
        case LDO_3_3V:
            return "ldo33";
        case LDO_3_0V:
            return "ldo30";
        case LDO_3_6V:
            return "ldo36";
        case LDO_3_9V:
            return "ldo39";
        case LDO_4_2V:
            return "ldo42";
        case LDO_4_5V:
            return "ldo45";
        case LDO_2_7V:
            return "ldo27";
        case LDO_2_4V:
            return "ldo24";
        default:
            return "ldo?";
    }
}

bool applyHardwarePreset(NAU7802_7Semi& scale) {
    if (!scale.setConfig(g_ldo, g_pga)) {
        printError("setConfig (LDO/PGA) failed");
        return false;
    }
    if (!scale.setSampleRate(CRS_80)) {
        printError("Failed to set sample rate after preset");
        return false;
    }
    scale.setSampling(weighRuntimeGetReadNumSamples(), weighRuntimeGetReadTimeoutMs());
    if (!enableInternalAvdd()) {
        return false;
    }
    if (!initChopperControl()) {
        return false;
    }
    delay(280);
    int32_t discarded = 0;
    for (int i = 0; i < 6; i++) {
        (void)scale.readRaw(discarded, 250);
    }
    return true;
}
