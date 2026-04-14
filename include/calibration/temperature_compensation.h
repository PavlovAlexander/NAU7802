#pragma once
#include <stdint.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

// Данные температурной компенсации
struct TemperatureCompensationData {
    bool enabled;
    
    float T_cal;        // Температура калибровки, °C
    float alpha;        // Температурный коэффициент для k (ppm/°C)
    float beta;         // Температурный коэффициент для b (мг/°C)
    
    // Для расширенной калибровки (двухточечная)
    float T1, T2;       // Две температуры калибровки
    float k1, b1;       // Коэффициенты при T1
    float k2, b2;       // Коэффициенты при T2
};

// Модуль температурной компенсации
// Использует встроенный температурный сенсор NAU7802
class TemperatureCompensator {
public:
    TemperatureCompensator();
    
    // Включить/выключить компенсацию
    void setEnabled(bool enabled);
    bool isEnabled() const;
    
    // Установить базовую температуру и коэффициенты
    void setCalibrationTemp(float T_cal);
    void setCoefficients(float alpha, float beta);
    
    // Расширенная калибровка (вычисляет alpha, beta из двух точек)
    void calibrateTwoPoint(float T1, float k1, float b1,
                           float T2, float k2, float b2);
    
    // Применить компенсацию к коэффициентам
    // Модифицирует k и b на основе текущей температуры
    void compensate(float T_current, float& k, float& b) const;
    
    // Прочитать температуру из NAU7802
    // Возвращает температуру в °C или NAN при ошибке
    float readTemperature(NAU7802& scale) const;
    
    // Загрузка/сохранение
    void loadFrom(const TemperatureCompensationData& data);
    void saveTo(TemperatureCompensationData& data) const;
    
    // Получить текущие параметры
    float getAlpha() const;
    float getBeta() const;
    float getCalibrationTemp() const;
    
private:
    TemperatureCompensationData data_;
};
