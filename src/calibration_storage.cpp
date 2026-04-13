#include "calibration_storage.h"
#include <Preferences.h>

void saveCalibration(const CalibrationData& cal) {
    Preferences prefs;
    prefs.begin("scale_cal", false);

    prefs.putFloat("k",         cal.k);
    prefs.putFloat("b",         cal.b);
    prefs.putFloat("r2",        cal.r2);
    prefs.putUInt("timestamp",  cal.timestamp);

    prefs.putInt("raw_0",    cal.points[0].rawADC);
    prefs.putFloat("weight_0", cal.points[0].weight);
    prefs.putInt("raw_1",    cal.points[1].rawADC);
    prefs.putFloat("weight_1", cal.points[1].weight);
    prefs.putInt("raw_2",    cal.points[2].rawADC);
    prefs.putFloat("weight_2", cal.points[2].weight);
    prefs.putInt("raw_3",    cal.points[3].rawADC);
    prefs.putFloat("weight_3", cal.points[3].weight);

    prefs.end();
}

bool loadCalibration(CalibrationData& cal) {
    Preferences prefs;
    prefs.begin("scale_cal", true);

    cal.k         = prefs.getFloat("k",         0.0f);
    cal.b         = prefs.getFloat("b",         0.0f);
    cal.r2        = prefs.getFloat("r2",        0.0f);
    cal.timestamp = prefs.getUInt("timestamp",  0);

    cal.points[0].rawADC = prefs.getInt("raw_0",    0);
    cal.points[0].weight = prefs.getFloat("weight_0", 0.0f);
    cal.points[1].rawADC = prefs.getInt("raw_1",    0);
    cal.points[1].weight = prefs.getFloat("weight_1", 0.0f);
    cal.points[2].rawADC = prefs.getInt("raw_2",    0);
    cal.points[2].weight = prefs.getFloat("weight_2", 0.0f);
    cal.points[3].rawADC = prefs.getInt("raw_3",    0);
    cal.points[3].weight = prefs.getFloat("weight_3", 0.0f);

    prefs.end();

    return cal.k != 0.0f;
}
