#include "calibration/temperature_compensation.h"
#include <Arduino.h>
#include <math.h>

TemperatureCompensator::TemperatureCompensator() {
    // Инициализация с дефолтными значениями
    data_.enabled = false;
    data_.T_cal = 25.0f;
    data_.alpha = 0.00002f;  // 20 ppm/°C
    data_.beta = 0.5f;       // 0.5 мг/°C
    data_.T1 = 0.0f;
    data_.T2 = 0.0f;
    data_.k1 = 0.0f;
    data_.b1 = 0.0f;
    data_.k2 = 0.0f;
    data_.b2 = 0.0f;
}

void TemperatureCompensator::setEnabled(bool enabled) {
    data_.enabled = enabled;
}

bool TemperatureCompensator::isEnabled() const {
    return data_.enabled;
}

void TemperatureCompensator::setCalibrationTemp(float T_cal) {
    data_.T_cal = T_cal;
}

void TemperatureCompensator::setCoefficients(float alpha, float beta) {
    data_.alpha = alpha;
    data_.beta = beta;
}

float TemperatureCompensator::getAlpha() const {
    return data_.alpha;
}

float TemperatureCompensator::getBeta() const {
    return data_.beta;
}

float TemperatureCompensator::getCalibrationTemp() const {
    return data_.T_cal;
}

void TemperatureCompensator::loadFrom(const TemperatureCompensationData& data) {
    data_ = data;
}

void TemperatureCompensator::saveTo(TemperatureCompensationData& data) const {
    data = data_;
}

// ============================================================================
// Двухточечная калибровка
// ============================================================================

void TemperatureCompensator::calibrateTwoPoint(float T1, float k1, float b1,
                                               float T2, float k2, float b2) {
    // Сохраняем данные калибровки
    data_.T1 = T1;
    data_.T2 = T2;
    data_.k1 = k1;
    data_.b1 = b1;
    data_.k2 = k2;
    data_.b2 = b2;
    
    // Проверка на вырожденный случай
    if (fabs(T2 - T1) < 0.1f) {
        // Температуры слишком близки, используем дефолтные коэффициенты
        data_.alpha = 0.00002f;
        data_.beta = 0.5f;
        data_.T_cal = T1;
        return;
    }
    
    // Вычисляем температурные коэффициенты
    // α = (k₂/k₁ - 1) / (T₂ - T₁)
    // β = (b₂ - b₁) / (T₂ - T₁)
    
    if (fabs(k1) > 1e-10f) {
        data_.alpha = (k2 / k1 - 1.0f) / (T2 - T1);
    } else {
        data_.alpha = 0.0f;
    }
    
    data_.beta = (b2 - b1) / (T2 - T1);
    
    // Используем первую температуру как базовую
    data_.T_cal = T1;
}

// ============================================================================
// Применение компенсации
// ============================================================================

void TemperatureCompensator::compensate(float T_current, float& k, float& b) const {
    if (!data_.enabled) {
        return;  // Компенсация отключена
    }
    
    // Проверка на валидность температуры
    if (isnan(T_current)) {
        return;  // Некорректная температура, не применяем компенсацию
    }
    
    // Вычисляем разницу температур
    float deltaT = T_current - data_.T_cal;
    
    // Применяем формулы коррекции:
    // k_corrected = k × (1 + α × ΔT)
    // b_corrected = b + β × ΔT
    
    k = k * (1.0f + data_.alpha * deltaT);
    b = b + data_.beta * deltaT;
}

// ============================================================================
// Чтение температуры из NAU7802
// ============================================================================

float TemperatureCompensator::readTemperature(NAU7802& scale) const {
    // NAU7802 имеет встроенный температурный сенсор
    // Для чтения температуры нужно:
    // 1. Переключить ADC на температурный канал
    // 2. Дождаться готовности данных
    // 3. Прочитать значение
    // 4. Преобразовать в °C
    
    // ВАЖНО: Библиотека SparkFun NAU7802 не предоставляет прямого API
    // для чтения температуры. Нужно работать с регистрами напрямую.
    
    // Сохраняем текущую конфигурацию
    // (в реальной реализации нужно сохранить регистры ADC_CTRL)
    
    // Для упрощения MVP используем приблизительную формулу
    // на основе внутреннего температурного сенсора ESP32
    
    // TODO: Реализовать чтение температуры из NAU7802 через регистры
    // Пока возвращаем комнатную температуру как заглушку
    
    // Попытка чтения через низкоуровневый доступ к регистрам
    // Регистр 0x15 (ADC_CTRL) бит 6 = TEMP_SENSOR
    
    uint8_t adcCtrl = scale.getRegister(NAU7802_CTRL1);
    
    // Включаем температурный сенсор (бит 6)
    scale.setRegister(NAU7802_CTRL1, adcCtrl | 0x40);
    
    // Ждём готовности данных (максимум 100 мс)
    unsigned long startTime = millis();
    while (!scale.available()) {
        if (millis() - startTime > 100) {
            // Таймаут, восстанавливаем конфигурацию
            scale.setRegister(NAU7802_CTRL1, adcCtrl);
            return NAN;
        }
        delay(1);
    }
    
    // Читаем значение
    int32_t tempRaw = scale.getReading();
    
    // Восстанавливаем конфигурацию
    scale.setRegister(NAU7802_CTRL1, adcCtrl);
    
    // Преобразуем в °C
    // Формула из даташита NAU7802:
    // T(°C) = (ADC_value / 2^23) * Vref / (0.00385 * R0) - 273.15
    // Для упрощения используем линейную аппроксимацию
    // T(°C) ≈ 25.0 + (tempRaw - baseline) * scale_factor
    
    // Базовое значение при 25°C (примерно)
    const int32_t baseline = 8388608;  // 2^23 / 2
    const float scale_factor = 0.0001f;  // Примерный коэффициент
    
    float temperature = 25.0f + (float)(tempRaw - baseline) * scale_factor;
    
    // Проверка на разумность (диапазон -10°C до +60°C)
    if (temperature < -10.0f || temperature > 60.0f) {
        return NAN;
    }
    
    return temperature;
}
