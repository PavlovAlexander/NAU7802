#include "measurement_logger.h"
#include <Arduino.h>

MeasurementLogger::MeasurementLogger() {
    logIndex_ = 0;
    logCount_ = 0;
}

void MeasurementLogger::log(int32_t rawADC, int32_t filteredADC, float weight, float temperature, bool outlierRejected) {
    // Добавляем запись в кольцевой буфер
    logs_[logIndex_].timestamp = millis();
    logs_[logIndex_].rawADC = rawADC;
    logs_[logIndex_].filteredADC = filteredADC;
    logs_[logIndex_].weight = weight;
    logs_[logIndex_].temperature = temperature;
    logs_[logIndex_].outlierRejected = outlierRejected;
    
    logIndex_ = (logIndex_ + 1) % MAX_LOGS;
    if (logCount_ < MAX_LOGS) {
        logCount_++;
    }
}

void MeasurementLogger::exportCSV() {
    Serial.println("=== Measurement Log (CSV) ===");
    Serial.println("Timestamp,RawADC,FilteredADC,Weight,Temperature,OutlierRejected");
    
    // Определяем начальный индекс (если буфер заполнен, начинаем со старейшей записи)
    int startIndex = (logCount_ < MAX_LOGS) ? 0 : logIndex_;
    
    for (int i = 0; i < logCount_; i++) {
        int idx = (startIndex + i) % MAX_LOGS;
        const MeasurementLog& entry = logs_[idx];
        
        Serial.printf("%lu,%ld,%ld,%.3f,%.2f,%d\n",
                      entry.timestamp,
                      (long)entry.rawADC,
                      (long)entry.filteredADC,
                      entry.weight,
                      entry.temperature,
                      entry.outlierRejected ? 1 : 0);
    }
    
    Serial.println("=============================");
}

void MeasurementLogger::clear() {
    logIndex_ = 0;
    logCount_ = 0;
    Serial.println("[LOGGER] Log cleared");
}

int MeasurementLogger::getCount() const {
    return logCount_;
}
