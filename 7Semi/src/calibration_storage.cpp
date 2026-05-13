#include "calibration_storage.h"

#include <Preferences.h>

namespace {
constexpr int32_t CALIBRATION_MIN_RAW_SPAN = 10000;

int32_t rawSpanAbs(const CalibrationData& cal) {
    const int32_t span = cal.points[3].rawADC - cal.points[0].rawADC;
    return (span < 0) ? -span : span;
}
}  // namespace

void saveCalibration(const CalibrationData& cal) {
    Preferences prefs;
    prefs.begin("scale_cal", false);

    prefs.putFloat("k", cal.k);
    prefs.putFloat("b", cal.b);
    prefs.putFloat("r2", cal.r2);
    prefs.putUInt("timestamp", cal.timestamp);
    prefs.putUChar("method", static_cast<uint8_t>(cal.method));

    for (int i = 0; i < 4; i++) {
        char keyRaw[8];
        char keyW[10];
        snprintf(keyRaw, sizeof(keyRaw), "raw_%d", i);
        snprintf(keyW, sizeof(keyW), "weight_%d", i);
        prefs.putInt(keyRaw, cal.points[i].rawADC);
        prefs.putFloat(keyW, cal.points[i].weight);
    }

    prefs.end();
}

bool loadCalibration(CalibrationData& cal) {
    Preferences prefs;
    prefs.begin("scale_cal", true);

    cal.k = prefs.getFloat("k", 0.0f);
    cal.b = prefs.getFloat("b", 0.0f);
    cal.r2 = prefs.getFloat("r2", 0.0f);
    cal.timestamp = prefs.getUInt("timestamp", 0);
    cal.method = static_cast<CalibrationMethod>(prefs.getUChar("method", CAL_PIECEWISE_LINEAR));

    for (int i = 0; i < 4; i++) {
        char keyRaw[8];
        char keyW[10];
        snprintf(keyRaw, sizeof(keyRaw), "raw_%d", i);
        snprintf(keyW, sizeof(keyW), "weight_%d", i);
        cal.points[i].rawADC = prefs.getInt(keyRaw, 0);
        cal.points[i].weight = prefs.getFloat(keyW, 0.0f);
    }

    prefs.end();
    return cal.timestamp != 0 && rawSpanAbs(cal) >= CALIBRATION_MIN_RAW_SPAN;
}
