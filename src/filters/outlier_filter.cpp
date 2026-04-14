#include "filters/outlier_filter.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

OutlierFilter::OutlierFilter() {
    // Инициализация с дефолтными значениями
    config_.method = OUTLIER_NONE;
    config_.windowSize = 5;
    config_.sigmaWindow = 15;
    config_.sigmaThreshold = 3.0f;
    
    bufferIndex_ = 0;
    bufferFilled_ = 0;
    lastValidValue_ = 0;
    outlierCount_ = 0;
    totalCount_ = 0;
    
    memset(medianBuffer_, 0, sizeof(medianBuffer_));
    memset(sigmaBuffer_, 0, sizeof(sigmaBuffer_));
}

void OutlierFilter::setMethod(OutlierFilterMethod method) {
    config_.method = method;
    // Сбрасываем буферы при смене метода
    bufferIndex_ = 0;
    bufferFilled_ = 0;
}

void OutlierFilter::setWindowSize(int size) {
    if (size == 3 || size == 5 || size == 7) {
        config_.windowSize = size;
        // Сбрасываем буфер
        bufferIndex_ = 0;
        bufferFilled_ = 0;
    }
}

void OutlierFilter::setSigmaWindow(int size) {
    if (size >= 10 && size <= 20) {
        config_.sigmaWindow = size;
        // Сбрасываем буфер
        bufferIndex_ = 0;
        bufferFilled_ = 0;
    }
}

void OutlierFilter::setSigmaThreshold(float threshold) {
    if (threshold > 0.0f) {
        config_.sigmaThreshold = threshold;
    }
}

OutlierFilterMethod OutlierFilter::getMethod() const {
    return config_.method;
}

int OutlierFilter::getWindowSize() const {
    return config_.windowSize;
}

float OutlierFilter::getSigmaThreshold() const {
    return config_.sigmaThreshold;
}

uint32_t OutlierFilter::getOutlierCount() const {
    return outlierCount_;
}

uint32_t OutlierFilter::getTotalCount() const {
    return totalCount_;
}

void OutlierFilter::resetStatistics() {
    outlierCount_ = 0;
    totalCount_ = 0;
}

void OutlierFilter::loadFrom(const OutlierFilterConfig& config) {
    config_ = config;
    // Сбрасываем буферы
    bufferIndex_ = 0;
    bufferFilled_ = 0;
}

void OutlierFilter::saveTo(OutlierFilterConfig& config) const {
    config = config_;
}

// ============================================================================
// Основной метод фильтрации
// ============================================================================

int32_t OutlierFilter::filter(int32_t rawADC) {
    totalCount_++;
    
    switch (config_.method) {
        case OUTLIER_NONE:
            lastValidValue_ = rawADC;
            return rawADC;
            
        case OUTLIER_MEDIAN:
            return medianFilter(rawADC);
            
        case OUTLIER_SIGMA:
            return sigmaFilter(rawADC);
            
        default:
            return rawADC;
    }
}

// ============================================================================
// Медианный фильтр
// ============================================================================

int32_t OutlierFilter::medianFilter(int32_t value) {
    // Добавляем значение в кольцевой буфер
    medianBuffer_[bufferIndex_] = value;
    bufferIndex_ = (bufferIndex_ + 1) % config_.windowSize;
    
    if (bufferFilled_ < config_.windowSize) {
        bufferFilled_++;
    }
    
    // Если буфер ещё не заполнен, возвращаем текущее значение
    if (bufferFilled_ < config_.windowSize) {
        lastValidValue_ = value;
        return value;
    }
    
    // Вычисляем медиану
    int32_t median = computeMedian(medianBuffer_, config_.windowSize);
    lastValidValue_ = median;
    return median;
}

int32_t OutlierFilter::computeMedian(int32_t* buffer, int size) {
    // Копируем буфер во временный массив для сортировки
    int32_t temp[7];  // Максимальный размер окна
    memcpy(temp, buffer, size * sizeof(int32_t));
    
    // Сортировка пузырьком (для малых размеров это быстро)
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                int32_t swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    
    // Возвращаем элемент в середине
    return temp[size / 2];
}

// ============================================================================
// Статистический 3σ фильтр
// ============================================================================

int32_t OutlierFilter::sigmaFilter(int32_t value) {
    // Если буфер ещё не заполнен, просто добавляем значение
    if (bufferFilled_ < config_.sigmaWindow) {
        sigmaBuffer_[bufferIndex_] = value;
        bufferIndex_ = (bufferIndex_ + 1) % config_.sigmaWindow;
        bufferFilled_++;
        lastValidValue_ = value;
        return value;
    }
    
    // Вычисляем среднее и стандартное отклонение
    float mean = computeMean(sigmaBuffer_, config_.sigmaWindow);
    float stddev = computeStdDev(sigmaBuffer_, config_.sigmaWindow, mean);
    
    // Проверяем, является ли новое значение выбросом
    float deviation = fabs((float)value - mean);
    float threshold = config_.sigmaThreshold * stddev;
    
    if (deviation > threshold) {
        // Выброс обнаружен
        outlierCount_++;
        // Возвращаем последнее валидное значение
        return lastValidValue_;
    } else {
        // Значение валидное, добавляем в буфер
        sigmaBuffer_[bufferIndex_] = value;
        bufferIndex_ = (bufferIndex_ + 1) % config_.sigmaWindow;
        lastValidValue_ = value;
        return value;
    }
}

float OutlierFilter::computeMean(int32_t* buffer, int size) {
    int64_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return (float)sum / (float)size;
}

float OutlierFilter::computeStdDev(int32_t* buffer, int size, float mean) {
    float variance = 0.0f;
    for (int i = 0; i < size; i++) {
        float diff = (float)buffer[i] - mean;
        variance += diff * diff;
    }
    variance /= (float)size;
    return sqrtf(variance);
}
