#pragma once

#include "calibration_data.h"

void printTagged(const char* tag, const char* msg);
void printError(const char* msg);
void printStatus(const CalibrationData& cal);
void printHelp();
