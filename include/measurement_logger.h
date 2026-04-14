#pragma once
#include <stdint.h>

// Структура записи лога измерения
struct MeasurementLog {
    uint32_t timestamp;      // Время измерения (millis)
    int32_t rawADC;          // Сырое значение ADC
    int32_t filteredADC;     // Отфильтрованное значение ADC
    float weight;            // Вычисленный вес (г)
    float temperature;       // Температура (°C)
    bool outlierRejected;    // Был ли отброшен выброс
};

// Класс для логирования измерений
class MeasurementLogger {
public:
    MeasurementLogger();
    
    // Добавить запись в лог
    void log(int32_t rawADC, int32_t filteredADC, float weight, float temperature, bool outlierRejected);
    
    // Экспорт лога в CSV формат
    void exportCSV();
    
    // Очистить лог
    void clear();
    
    // Получить количество записей
    int getCount() const;
    
private:
    static const int MAX_LOGS = 100;
    MeasurementLog logs_[MAX_LOGS];
    int logIndex_;
    int logCount_;
};
