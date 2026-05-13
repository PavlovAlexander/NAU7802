#include "scale_init_7semi.h"

#include "chopper_control_7semi.h"
#include "ui_console_7semi.h"
#include "weigh_runtime_7semi.h"

bool initHardware(NAU7802_7Semi& scale) {
    if (!scale.begin(21, 22, 100000)) {
        printError("NAU7802 init failed");
        return false;
    }
    weighRuntimeInitDefaults();
    if (!applyHardwarePreset(scale)) {
        return false;
    }
    dumpAndValidatePowerState();

    printTagged("INIT", "Initial sample flush complete");
    printTagged("INIT", "NAU7802 (7Semi) initialized");
    return true;
}
