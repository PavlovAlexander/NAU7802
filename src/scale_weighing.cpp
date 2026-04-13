#include "scale_weighing.h"
#include "ui_console.h"

// Вычисляет вес по линейной модели: weight = k * rawADC + b
float computeWeight(float k, float b, int32_t rawADC) {
    return k * (float)rawADC + b;
}

// Один цикл взвешивания
void weighingTick(NAU7802& scale,
                  const CalibrationData& cal) {
    // 16 отсчётов @ 80 SPS = ~200 мс → 5 выводов/сек
    int64_t sum = 0;
    const int numSamples = 16;
    
    for (int i = 0; i < numSamples; i++) {
        while (!scale.available()) {
            delay(1);
        }
        sum += scale.getReading();
    }
    
    int32_t rawADC = (int32_t)(sum / numSamples);
    float calibrated = computeWeight(cal.k, cal.b, rawADC);
    printWeighLine(rawADC, calibrated);
}
