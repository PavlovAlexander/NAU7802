#include "filters/adaptive_averager.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

AdaptiveAverager::AdaptiveAverager() {
    // Инициализация с дефолтными значениями
    config_.enabled = false;
    config_.stabilityThreshold = 5.0f;  // 5 мг
    config_.minSamples = 10;
    config_.maxSamples = 50;
    config_.transitionSteps = 5;
    
    currentSampleCount_ = config_.minSamples;
    currentStdDev_ = 0.0f;
    historyIndex_ = 0;
    historyFilled_ = 0;
    
    memset(historyBuffer_, 0, sizeof(historyBuffer_));
}

void AdaptiveAverager::setEnabled(bool enabled) {
    config_.enabled = enabled;
}

void AdaptiveAverager::setStabilityThreshold(float threshold) {
    if (threshold > 0.0f) {
        config_.stabilityThreshold = threshold;
    }
}

void AdaptiveAverager::setSampleRange(int minSamples, int maxSamples) {
    if (minSamples > 0 && maxSamples >= minSamples) {
        config_.minSamples = minSamples;
        config_.maxSamples = maxSamples;
    }
}

void AdaptiveAverager::setTransitionSteps(int steps) {
    if (steps > 0) {
        config_.transitionSteps = steps;
    }
}

bool AdaptiveAverager::isEnabled() const {
    return config_.enabled;
}

float AdaptiveAverager::getStabilityThreshold() const {
    return config_.stabilityThreshold;
}

int AdaptiveAverager::getMinSamples() const {
    return config_.minSamples;
}

int AdaptiveAverager::getMaxSamples() const {
    return config_.maxSamples;
}

int AdaptiveAverager::getCurrentSampleCount() const {
    return currentSampleCount_;
}

float AdaptiveAverager::getCurrentStdDev() const {
    return currentStdDev_;
}

void AdaptiveAverager::loadFrom(const AdaptiveAveragerConfig& config) {
    config_ = config;
}

void AdaptiveAverager::saveTo(AdaptiveAveragerConfig& config) const {
    config = config_;
}

// ============================================================================
// Обновление истории измерений
// ============================================================================

void AdaptiveAverager::updateHistory(float weight) {
    // Добавляем вес в кольцевой буфер
    historyBuffer_[historyIndex_] = weight;
    historyIndex_ = (historyIndex_ + 1) % 10;
    
    if (historyFilled_ < 10) {
        historyFilled_++;
    }
}

// ============================================================================
// Вычисление количества отсчётов на основе стабильности
// ============================================================================

int AdaptiveAverager::computeSampleCount() {
    if (!config_.enabled) {
        return config_.minSamples;
    }
    
    // Если буфер ещё не заполнен, используем минимальное количество
    if (historyFilled_ < 10) {
        return config_.minSamples;
    }
    
    // Вычисляем стандартное отклонение последних измерений
    currentStdDev_ = computeStdDev(historyBuffer_, historyFilled_);
    
    // Быстрая реакция на резкие изменения (Требование 4.8):
    // при sigma >= 2 * threshold немедленно переключаемся на minSamples
    if (currentStdDev_ >= config_.stabilityThreshold * 2.0f) {
        currentSampleCount_ = config_.minSamples;
        return currentSampleCount_;
    }
    
    // Определяем целевое количество отсчётов
    int targetSampleCount;
    if (currentStdDev_ < config_.stabilityThreshold) {
        // Стабильное состояние → используем максимум отсчётов для точности
        targetSampleCount = config_.maxSamples;
    } else {
        // Нестабильное состояние → используем минимум отсчётов для скорости
        targetSampleCount = config_.minSamples;
    }
    
    // Плавный переход к целевому значению
    int delta = targetSampleCount - currentSampleCount_;
    int step = delta / config_.transitionSteps;
    
    // Минимальный шаг = 1 (чтобы не застрять)
    if (step == 0 && delta != 0) {
        step = (delta > 0) ? 1 : -1;
    }
    
    currentSampleCount_ += step;
    
    // Ограничиваем диапазон
    if (currentSampleCount_ < config_.minSamples) {
        currentSampleCount_ = config_.minSamples;
    }
    if (currentSampleCount_ > config_.maxSamples) {
        currentSampleCount_ = config_.maxSamples;
    }
    
    return currentSampleCount_;
}

// ============================================================================
// Вспомогательные методы
// ============================================================================

float AdaptiveAverager::computeMean(const float* values, int count) {
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += values[i];
    }
    return sum / (float)count;
}

float AdaptiveAverager::computeStdDev(const float* values, int count) {
    if (count < 2) return 0.0f;
    
    float mean = computeMean(values, count);
    
    float variance = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = values[i] - mean;
        variance += diff * diff;
    }
    variance /= (float)count;
    
    return sqrtf(variance);
}
