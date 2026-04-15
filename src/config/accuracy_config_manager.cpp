#include "config/accuracy_config_manager.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

// Namespace и ключ для NVS
static const char* NVS_NAMESPACE = "scale_cfg";
static const char* NVS_KEY = "acc_cfg";

AccuracyConfigManager::AccuracyConfigManager() {
    // Инициализация с дефолтными значениями
    config_ = DEFAULT_ACCURACY_CONFIG;
}

// ============================================================================
// Инициализация
// ============================================================================

bool AccuracyConfigManager::init() {
    // Пытаемся загрузить конфигурацию из NVS
    if (load()) {
        Serial.println("[AccuracyConfig] Loaded from NVS");
        return true;
    } else {
        Serial.println("[AccuracyConfig] Using default config");
        config_ = DEFAULT_ACCURACY_CONFIG;
        return false;
    }
}

// ============================================================================
// Управление компонентами
// ============================================================================

void AccuracyConfigManager::enableOutlierFilter(bool enabled) {
    config_.outlierFilterEnabled = enabled;
}

void AccuracyConfigManager::enableTemperatureComp(bool enabled) {
    config_.temperatureCompEnabled = enabled;
}

void AccuracyConfigManager::enableAdaptiveAveraging(bool enabled) {
    config_.adaptiveAvgEnabled = enabled;
}

bool AccuracyConfigManager::isOutlierFilterEnabled() const {
    return config_.outlierFilterEnabled;
}

bool AccuracyConfigManager::isTemperatureCompEnabled() const {
    return config_.temperatureCompEnabled;
}

bool AccuracyConfigManager::isAdaptiveAveragingEnabled() const {
    return config_.adaptiveAvgEnabled;
}

// ============================================================================
// Настройка OutlierFilter
// ============================================================================

void AccuracyConfigManager::setOutlierMethod(OutlierFilterMethod method) {
    config_.outlierMethod = (uint8_t)method;
}

void AccuracyConfigManager::setOutlierWindowSize(uint8_t size) {
    if (size == 3 || size == 5 || size == 7) {
        config_.outlierWindowSize = size;
    }
}

void AccuracyConfigManager::setOutlierSigmaWindow(uint8_t window) {
    if (window >= 10 && window <= 20) {
        config_.outlierSigmaWindow = window;
    }
}

void AccuracyConfigManager::setOutlierSigmaThreshold(float threshold) {
    if (threshold > 0.0f) {
        config_.outlierSigmaThreshold = threshold;
    }
}

OutlierFilterMethod AccuracyConfigManager::getOutlierMethod() const {
    return (OutlierFilterMethod)config_.outlierMethod;
}

uint8_t AccuracyConfigManager::getOutlierWindowSize() const {
    return config_.outlierWindowSize;
}

float AccuracyConfigManager::getOutlierSigmaThreshold() const {
    return config_.outlierSigmaThreshold;
}

// ============================================================================
// Настройка TemperatureCompensator
// ============================================================================

void AccuracyConfigManager::setTempCoefficients(float alpha, float beta) {
    config_.tempAlpha = alpha;
    config_.tempBeta = beta;
}

void AccuracyConfigManager::getTempCoefficients(float& alpha, float& beta) const {
    alpha = config_.tempAlpha;
    beta = config_.tempBeta;
}

// ============================================================================
// Настройка AdaptiveAverager
// ============================================================================

void AccuracyConfigManager::setAdaptiveParams(float threshold, uint8_t minSamples, uint8_t maxSamples, uint8_t transitionSteps) {
    if (threshold > 0.0f && minSamples > 0 && maxSamples >= minSamples && transitionSteps > 0) {
        config_.adaptiveStabilityThreshold = threshold;
        config_.adaptiveMinSamples = minSamples;
        config_.adaptiveMaxSamples = maxSamples;
        config_.adaptiveTransitionSteps = transitionSteps;
    }
}

void AccuracyConfigManager::getAdaptiveParams(float& threshold, uint8_t& minSamples, uint8_t& maxSamples) const {
    threshold = config_.adaptiveStabilityThreshold;
    minSamples = config_.adaptiveMinSamples;
    maxSamples = config_.adaptiveMaxSamples;
}

// ============================================================================
// Сохранение/загрузка
// ============================================================================

bool AccuracyConfigManager::save() {
    // Вычисляем CRC перед сохранением
    config_.crc16 = computeCRC16((const uint8_t*)&config_, sizeof(AccuracyConfig) - sizeof(uint16_t));
    
    // Открываем NVS
    if (!prefs_.begin(NVS_NAMESPACE, false)) {
        Serial.println("[AccuracyConfig] Failed to open NVS for writing");
        return false;
    }
    
    // Сохраняем конфигурацию
    size_t written = prefs_.putBytes(NVS_KEY, &config_, sizeof(AccuracyConfig));
    prefs_.end();
    
    if (written != sizeof(AccuracyConfig)) {
        Serial.println("[AccuracyConfig] Failed to write config to NVS");
        return false;
    }
    
    Serial.println("[AccuracyConfig] Saved to NVS");
    return true;
}

bool AccuracyConfigManager::load() {
    // Открываем NVS
    if (!prefs_.begin(NVS_NAMESPACE, true)) {
        Serial.println("[AccuracyConfig] Failed to open NVS for reading");
        return false;
    }
    
    // Загружаем конфигурацию
    AccuracyConfig loadedConfig;
    size_t read = prefs_.getBytes(NVS_KEY, &loadedConfig, sizeof(AccuracyConfig));
    prefs_.end();
    
    if (read != sizeof(AccuracyConfig)) {
        Serial.println("[AccuracyConfig] Config not found in NVS");
        return false;
    }
    
    // Проверяем CRC
    uint16_t expectedCRC = computeCRC16((const uint8_t*)&loadedConfig, sizeof(AccuracyConfig) - sizeof(uint16_t));
    if (loadedConfig.crc16 != expectedCRC) {
        Serial.printf("[AccuracyConfig] CRC mismatch: expected 0x%04X, got 0x%04X\n", expectedCRC, loadedConfig.crc16);
        return false;
    }
    
    // Валидация
    if (!validateConfig(loadedConfig)) {
        Serial.println("[AccuracyConfig] Loaded config is invalid");
        return false;
    }
    
    // Применяем загруженную конфигурацию
    config_ = loadedConfig;
    return true;
}

void AccuracyConfigManager::reset() {
    config_ = DEFAULT_ACCURACY_CONFIG;
    Serial.println("[AccuracyConfig] Reset to default");
}

// ============================================================================
// Экспорт/импорт конфигурации
// ============================================================================

void AccuracyConfigManager::exportConfig(char* buffer, size_t bufferSize) {
    snprintf(buffer, bufferSize,
        "version=%u\n"
        "outlierFilterEnabled=%d\n"
        "temperatureCompEnabled=%d\n"
        "adaptiveAvgEnabled=%d\n"
        "outlierMethod=%u\n"
        "outlierWindowSize=%u\n"
        "outlierSigmaWindow=%u\n"
        "outlierSigmaThreshold=%.9g\n"
        "tempAlpha=%.9g\n"
        "tempBeta=%.9g\n"
        "adaptiveStabilityThreshold=%.9g\n"
        "adaptiveMinSamples=%u\n"
        "adaptiveMaxSamples=%u\n"
        "adaptiveTransitionSteps=%u\n",
        config_.version,
        config_.outlierFilterEnabled ? 1 : 0,
        config_.temperatureCompEnabled ? 1 : 0,
        config_.adaptiveAvgEnabled ? 1 : 0,
        config_.outlierMethod,
        config_.outlierWindowSize,
        config_.outlierSigmaWindow,
        config_.outlierSigmaThreshold,
        config_.tempAlpha,
        config_.tempBeta,
        config_.adaptiveStabilityThreshold,
        config_.adaptiveMinSamples,
        config_.adaptiveMaxSamples,
        config_.adaptiveTransitionSteps
    );
}

bool AccuracyConfigManager::importConfig(const char* configText) {
    AccuracyConfig newConfig = config_;  // Начинаем с текущей конфигурации
    
    // Простой парсер key=value
    char line[128];
    const char* ptr = configText;
    
    while (*ptr) {
        // Читаем строку
        size_t i = 0;
        while (*ptr && *ptr != '\n' && i < sizeof(line) - 1) {
            line[i++] = *ptr++;
        }
        line[i] = '\0';
        if (*ptr == '\n') ptr++;
        
        // Пропускаем пустые строки
        if (line[0] == '\0') continue;
        
        // Парсим key=value
        char* eq = strchr(line, '=');
        if (!eq) continue;
        
        *eq = '\0';
        const char* key = line;
        const char* value = eq + 1;
        
        // Применяем значения
        if (strcmp(key, "version") == 0) {
            newConfig.version = atoi(value);
        } else if (strcmp(key, "outlierFilterEnabled") == 0) {
            newConfig.outlierFilterEnabled = atoi(value) != 0;
        } else if (strcmp(key, "temperatureCompEnabled") == 0) {
            newConfig.temperatureCompEnabled = atoi(value) != 0;
        } else if (strcmp(key, "adaptiveAvgEnabled") == 0) {
            newConfig.adaptiveAvgEnabled = atoi(value) != 0;
        } else if (strcmp(key, "outlierMethod") == 0) {
            newConfig.outlierMethod = atoi(value);
        } else if (strcmp(key, "outlierWindowSize") == 0) {
            newConfig.outlierWindowSize = atoi(value);
        } else if (strcmp(key, "outlierSigmaWindow") == 0) {
            newConfig.outlierSigmaWindow = atoi(value);
        } else if (strcmp(key, "outlierSigmaThreshold") == 0) {
            newConfig.outlierSigmaThreshold = atof(value);
        } else if (strcmp(key, "tempAlpha") == 0) {
            newConfig.tempAlpha = atof(value);
        } else if (strcmp(key, "tempBeta") == 0) {
            newConfig.tempBeta = atof(value);
        } else if (strcmp(key, "adaptiveStabilityThreshold") == 0) {
            newConfig.adaptiveStabilityThreshold = atof(value);
        } else if (strcmp(key, "adaptiveMinSamples") == 0) {
            newConfig.adaptiveMinSamples = atoi(value);
        } else if (strcmp(key, "adaptiveMaxSamples") == 0) {
            newConfig.adaptiveMaxSamples = atoi(value);
        } else if (strcmp(key, "adaptiveTransitionSteps") == 0) {
            newConfig.adaptiveTransitionSteps = atoi(value);
        }
    }
    
    // Валидация
    if (!validateConfig(newConfig)) {
        Serial.println("[AccuracyConfig] Imported config is invalid");
        return false;
    }
    
    // Применяем
    config_ = newConfig;
    Serial.println("[AccuracyConfig] Config imported successfully");
    return true;
}

// ============================================================================
// Диагностика
// ============================================================================

void AccuracyConfigManager::printStatus() {
    Serial.println("=== Accuracy Configuration ===");
    Serial.printf("Version: %u\n", config_.version);
    Serial.println();
    
    Serial.println("Components:");
    Serial.printf("  Outlier Filter:        %s\n", config_.outlierFilterEnabled ? "ENABLED" : "DISABLED");
    Serial.printf("  Temperature Comp:      %s\n", config_.temperatureCompEnabled ? "ENABLED" : "DISABLED");
    Serial.printf("  Adaptive Averaging:    %s\n", config_.adaptiveAvgEnabled ? "ENABLED" : "DISABLED");
    Serial.println();
    
    Serial.println("Outlier Filter:");
    const char* methodName = "NONE";
    if (config_.outlierMethod == OUTLIER_MEDIAN) methodName = "MEDIAN";
    else if (config_.outlierMethod == OUTLIER_SIGMA) methodName = "SIGMA";
    Serial.printf("  Method:                %s\n", methodName);
    Serial.printf("  Window Size:           %u\n", config_.outlierWindowSize);
    Serial.printf("  Sigma Window:          %u\n", config_.outlierSigmaWindow);
    Serial.printf("  Sigma Threshold:       %.2f\n", config_.outlierSigmaThreshold);
    Serial.println();
    
    Serial.println("Temperature Compensation:");
    Serial.printf("  Alpha (ppm/°C):        %.6f\n", config_.tempAlpha);
    Serial.printf("  Beta (mg/°C):          %.2f\n", config_.tempBeta);
    Serial.println();
    
    Serial.println("Adaptive Averaging:");
    Serial.printf("  Stability Threshold:   %.2f mg\n", config_.adaptiveStabilityThreshold);
    Serial.printf("  Min Samples:           %u\n", config_.adaptiveMinSamples);
    Serial.printf("  Max Samples:           %u\n", config_.adaptiveMaxSamples);
    Serial.printf("  Transition Steps:      %u\n", config_.adaptiveTransitionSteps);
    Serial.println();
    
    Serial.printf("CRC16: 0x%04X\n", config_.crc16);
    Serial.println("==============================");
}

// ============================================================================
// Прямой доступ
// ============================================================================

const AccuracyConfig& AccuracyConfigManager::getConfig() const {
    return config_;
}

void AccuracyConfigManager::setConfig(const AccuracyConfig& config) {
    if (validateConfig(config)) {
        config_ = config;
    }
}

// ============================================================================
// Валидация
// ============================================================================

bool AccuracyConfigManager::validateConfig(const AccuracyConfig& config) {
    // Проверка версии
    if (config.version == 0 || config.version > 100) {
        Serial.printf("[AccuracyConfig] Invalid version: %u\n", config.version);
        return false;
    }
    
    // Проверка outlier filter
    if (config.outlierMethod > OUTLIER_SIGMA) {
        Serial.printf("[AccuracyConfig] Invalid outlier method: %u\n", config.outlierMethod);
        return false;
    }
    
    if (config.outlierWindowSize != 3 && config.outlierWindowSize != 5 && config.outlierWindowSize != 7) {
        Serial.printf("[AccuracyConfig] Invalid outlier window size: %u\n", config.outlierWindowSize);
        return false;
    }
    
    if (config.outlierSigmaWindow < 10 || config.outlierSigmaWindow > 20) {
        Serial.printf("[AccuracyConfig] Invalid sigma window: %u\n", config.outlierSigmaWindow);
        return false;
    }
    
    if (config.outlierSigmaThreshold <= 0.0f || config.outlierSigmaThreshold > 10.0f) {
        Serial.printf("[AccuracyConfig] Invalid sigma threshold: %.2f\n", config.outlierSigmaThreshold);
        return false;
    }
    
    // Проверка adaptive averaging
    if (config.adaptiveStabilityThreshold <= 0.0f) {
        Serial.printf("[AccuracyConfig] Invalid stability threshold: %.2f\n", config.adaptiveStabilityThreshold);
        return false;
    }
    
    if (config.adaptiveMinSamples == 0 || config.adaptiveMinSamples > config.adaptiveMaxSamples) {
        Serial.printf("[AccuracyConfig] Invalid sample range: %u-%u\n", config.adaptiveMinSamples, config.adaptiveMaxSamples);
        return false;
    }
    
    if (config.adaptiveTransitionSteps == 0) {
        Serial.printf("[AccuracyConfig] Invalid transition steps: %u\n", config.adaptiveTransitionSteps);
        return false;
    }
    
    return true;
}

// ============================================================================
// CRC16
// ============================================================================

uint16_t AccuracyConfigManager::computeCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}
