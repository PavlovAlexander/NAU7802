#pragma once
#include "config/accuracy_config.h"
#include <Preferences.h>

// Менеджер конфигурации компонентов точности
// Управляет сохранением/загрузкой конфигурации в NVS
class AccuracyConfigManager {
public:
    AccuracyConfigManager();
    
    // Инициализация (загрузка из NVS)
    bool init();
    
    // Управление компонентами
    void enableOutlierFilter(bool enabled);
    void enableTemperatureComp(bool enabled);
    void enableAdaptiveAveraging(bool enabled);
    
    bool isOutlierFilterEnabled() const;
    bool isTemperatureCompEnabled() const;
    bool isAdaptiveAveragingEnabled() const;
    
    // Настройка OutlierFilter
    void setOutlierMethod(OutlierFilterMethod method);
    void setOutlierWindowSize(uint8_t size);
    void setOutlierSigmaWindow(uint8_t window);
    void setOutlierSigmaThreshold(float threshold);
    
    OutlierFilterMethod getOutlierMethod() const;
    uint8_t getOutlierWindowSize() const;
    float getOutlierSigmaThreshold() const;
    
    // Настройка TemperatureCompensator
    void setTempCoefficients(float alpha, float beta);
    void getTempCoefficients(float& alpha, float& beta) const;
    
    // Настройка AdaptiveAverager
    void setAdaptiveParams(float threshold, uint8_t minSamples, uint8_t maxSamples, uint8_t transitionSteps);
    void getAdaptiveParams(float& threshold, uint8_t& minSamples, uint8_t& maxSamples) const;
    
    // Сохранение/загрузка
    bool save();
    bool load();
    void reset();  // Сброс к DEFAULT_ACCURACY_CONFIG
    
    // Экспорт/импорт конфигурации
    void exportConfig(char* buffer, size_t bufferSize);
    bool importConfig(const char* configText);
    
    // Диагностика
    void printStatus();
    
    // Прямой доступ к конфигурации
    const AccuracyConfig& getConfig() const;
    void setConfig(const AccuracyConfig& config);
    
private:
    AccuracyConfig config_;
    Preferences prefs_;
    
    // Валидация
    bool validateConfig(const AccuracyConfig& config);
    
    // CRC
    uint16_t computeCRC16(const uint8_t* data, size_t length);
};
