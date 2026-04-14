#pragma once
#include <stdint.h>
#include "calibration_data.h"

// Модуль нелинейной калибровки
// Поддерживает 4 метода: линейная, кусочно-линейная, полиномиальная 2/3 степени
class CalibrationModule {
public:
    CalibrationModule();
    
    // Установить метод калибровки
    void setMethod(CalibrationMethod method);
    CalibrationMethod getMethod() const;
    
    // Калибровка (заполняет points[], вычисляет коэффициенты)
    // Возвращает true если успешно (R² >= 0.99)
    bool calibrate(const CalibrationPoint* points, int numPoints);
    
    // Преобразование raw ADC → граммы
    float rawToWeight(int32_t rawADC) const;
    
    // Получить R² (коэффициент детерминации)
    float getR2() const;
    
    // Получить коэффициенты k и b (для линейной)
    float getK() const;
    float getB() const;
    
    // Загрузка/сохранение из CalibrationData
    void loadFrom(const CalibrationData& cal);
    void saveTo(CalibrationData& cal) const;
    
private:
    CalibrationData data_;
    
    // Внутренние методы вычисления коэффициентов
    void computeLinear();
    void computePiecewiseLinear();
    void computePolynomial(int degree);
    
    // Методы интерполяции/вычисления
    float interpolatePiecewise(int32_t rawADC) const;
    float evaluatePolynomial(int32_t rawADC) const;
    
    // Вычисление R² для оценки качества
    float computeR2() const;
};
