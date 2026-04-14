#pragma once
#include <stdint.h>

// Конфигурация адаптивного усреднения
struct AdaptiveAveragerConfig {
    bool enabled;
    float stabilityThreshold;  // Порог σ для определения стабильности, мг
    int minSamples;            // Минимальное количество отсчётов (быстрый режим)
    int maxSamples;            // Максимальное количество отсчётов (точный режим)
    int transitionSteps;       // Количество шагов для плавного перехода
};

// Модуль адаптивного усреднения
// Автоматически балансирует между точностью и скоростью отклика
class AdaptiveAverager {
public:
    AdaptiveAverager();
    
    // Настройка
    void setEnabled(bool enabled);
    void setStabilityThreshold(float threshold);
    void setSampleRange(int minSamples, int maxSamples);
    void setTransitionSteps(int steps);
    
    bool isEnabled() const;
    float getStabilityThreshold() const;
    int getMinSamples() const;
    int getMaxSamples() const;
    
    // Обновить историю измерений
    // Вызывается после каждого измерения веса
    void updateHistory(float weight);
    
    // Вычислить количество отсчётов на основе стабильности
    // Возвращает текущее рекомендуемое количество отсчётов
    int computeSampleCount();
    
    // Получить текущее количество отсчётов
    int getCurrentSampleCount() const;
    
    // Статистика
    float getCurrentStdDev() const;
    
    // Загрузка/сохранение
    void loadFrom(const AdaptiveAveragerConfig& config);
    void saveTo(AdaptiveAveragerConfig& config) const;
    
private:
    AdaptiveAveragerConfig config_;
    int currentSampleCount_;
    float currentStdDev_;
    
    // Буфер последних измерений для вычисления σ
    float historyBuffer_[10];
    int historyIndex_;
    int historyFilled_;
    
    float computeStdDev(const float* values, int count);
    float computeMean(const float* values, int count);
};
