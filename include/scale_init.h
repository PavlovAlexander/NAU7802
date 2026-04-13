#pragma once
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

// Инициализирует Wire (GPIO 21/22, 400 кГц) и NAU7802.
// Возвращает true при успехе, false при ошибке обнаружения.
bool initHardware(NAU7802& scale);
