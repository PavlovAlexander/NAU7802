#pragma once
#include <stdint.h>
#include "config/accuracy_config.h"

// Конфигурация фильтра выбросов
struct OutlierFilterConfig {
    OutlierFilterMethod method;
    int windowSize;        // Размер окна для медианного (3, 5, 7)
    int sigmaWindow;       // Размер окна для σ вычисления (10-20)
    float sigmaThreshold;  // Порог в σ (обычно 3.0)
};

// Фильтр выбросов
// Поддерживает медианный фильтр и статистический 3σ фильтр
class OutlierFilter {
public:
    OutlierFilter();
    
    // Настройка метода
    void setMethod(OutlierFilterMethod method);
    void setWindowSize(int size);
    void setSigmaWindow(int size);
    void setSigmaThreshold(float threshold);
    
    OutlierFilterMethod getMethod() const;
    int getWindowSize() const;
    float getSigmaThreshold() const;
    
    // Обработка значения
    // Возвращает отфильтрованное значение
    int32_t filter(int32_t rawADC);
    
    // Статистика
    uint32_t getOutlierCount() const;
    uint32_t getTotalCount() const;
    void resetStatistics();
    
    // Загрузка/сохранение конфигурации
    void loadFrom(const OutlierFilterConfig& config);
    void saveTo(OutlierFilterConfig& config) const;
    
private:
    OutlierFilterConfig config_;
    
    // Буферы для фильтров
    int32_t medianBuffer_[7];    // Максимальный размер окна
    int32_t sigmaBuffer_[20];    // Максимальный размер окна
    int bufferIndex_;
    int bufferFilled_;
    
    int32_t lastValidValue_;     // Последнее валидное значение
    uint32_t outlierCount_;      // Счётчик выбросов
    uint32_t totalCount_;        // Общий счётчик измерений
    
    // Внутренние методы
    int32_t medianFilter(int32_t value);
    int32_t sigmaFilter(int32_t value);
    int32_t computeMedian(int32_t* buffer, int size);
    float computeMean(int32_t* buffer, int size);
    float computeStdDev(int32_t* buffer, int size, float mean);
};
