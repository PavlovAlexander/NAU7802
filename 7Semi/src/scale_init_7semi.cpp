#include "scale_init_7semi.h"

#include "chopper_control_7semi.h"
#include "ui_console_7semi.h"

bool initHardware(NAU7802_7Semi& scale) {
    if (!scale.begin()) {
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
    scale.setSampling(16, 1200);
    if (!initChopperControl()) {
        return false;
    }
    printTagged("INIT", "NAU7802 (7Semi) initialized");
    return true;
}
