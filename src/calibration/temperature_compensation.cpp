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
    // Процедура чтения встроенного температурного сенсора NAU7802 по даташиту v1.7:
    //
    // Регистр I2C_CTRL (0x11), бит 1 (TS) — Temperature Sensor Select:
    //   1 = температурный сенсор подключён к входу PGA (ADC читает температуру)
    //   0 = температурный сенсор отключён (ADC читает мостовой датчик)
    //
    // Характеристики (секция 7.4): точность ±1°C, диапазон -40..+85°C

    // Регистр I2C_CTRL (0x11) по даташиту NAU7802
    const uint8_t NAU7802_I2C_CTRL = 0x11;
    // Бит 1 (TS) — подключение температурного сенсора к PGA
    const uint8_t NAU7802_TS_BIT   = 0x02;

    // 1. Сохраняем текущее значение I2C_CTRL
    uint8_t i2cCtrl = scale.getRegister(NAU7802_I2C_CTRL);

    // 2. Устанавливаем TS = 1 — подключаем температурный сенсор к входу PGA
    scale.setRegister(NAU7802_I2C_CTRL, i2cCtrl | NAU7802_TS_BIT);

    // 3. Ждём готовности данных (DRDY), максимум 200 мс
    unsigned long startTime = millis();
    while (!scale.available()) {
        if (millis() - startTime > 200) {
            // Тайм-аут: восстанавливаем регистр и возвращаем ошибку
            scale.setRegister(NAU7802_I2C_CTRL, i2cCtrl);
            return NAN;
        }
        delay(1);
    }

    // 4. Читаем 24-bit сырое значение из ADC
    int32_t tempRaw = scale.getReading();

    // 5. Восстанавливаем I2C_CTRL (отключаем температурный сенсор)
    scale.setRegister(NAU7802_I2C_CTRL, i2cCtrl);

    // 6. Пересчитываем raw -> °C
    // Даташит NAU7802 v1.7 не публикует точную формулу пересчёта raw->°C.
    // Используем линейное приближение: T = raw / 65536.0
    // Коэффициент подлежит уточнению экспериментально на конкретном экземпляре чипа.
    // Альтернатива: calibrateTwoPoint() для двухточечной калибровки сенсора.
    float temperature = (float)tempRaw / 65536.0f;

    // Проверка на физически разумный диапазон (NAU7802: -40..+85°C)
    if (temperature < -40.0f || temperature > 85.0f) {
        return NAN;
    }

    return temperature;
}
