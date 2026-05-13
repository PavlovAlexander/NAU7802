#include "scale_init_7semi.h"

#include "chopper_control_7semi.h"
#include "ui_console_7semi.h"

bool initHardware(NAU7802_7Semi& scale) {
    if (!scale.begin(21, 22, 100000)) {
        printError("NAU7802 init failed");
        return false;
    }
    if (!scale.setSampleRate(CRS_80)) {
        printError("Failed to set sample rate");
        return false;
    }
    if (!scale.setGain(PGA_128)) {
        printError("Failed to set gain");
        return false;
    }
    // Library readAverage() splits maxTimeout across numSamples (per-sample budget).
    // Keep total budget comfortably above 16 * (1/SPS) at CRS_80 plus I2C margin.
    scale.setSampling(16, 4000);
    if (!enableInternalAvdd()) {
        return false;
    }
    if (!initChopperControl()) {
        return false;
    }
    dumpAndValidatePowerState();

    int32_t discarded = 0;
    for (int i = 0; i < 6; i++) {
        (void)scale.readRaw(discarded, 250);
    }
    printTagged("INIT", "Initial sample flush complete");
    printTagged("INIT", "NAU7802 (7Semi) initialized");
    return true;
}
