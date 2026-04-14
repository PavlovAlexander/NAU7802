#include "calibration/nonlinear_calibration.h"
#include <Arduino.h>
#include <math.h>

CalibrationModule::CalibrationModule() {
    // Инициализация с дефолтными значениями
    data_ = DEFAULT_CALIBRATION_EXTENSION;
}

void CalibrationModule::setMethod(CalibrationMethod method) {
    data_.method = method;
}

CalibrationMethod CalibrationModule::getMethod() const {
    return data_.method;
}

float CalibrationModule::getK() const {
    return data_.k;
}

float CalibrationModule::getB() const {
    return data_.b;
}

float CalibrationModule::getR2() const {
    return data_.r2;
}

void CalibrationModule::loadFrom(const CalibrationData& cal) {
    data_ = cal;
}

void CalibrationModule::saveTo(CalibrationData& cal) const {
    cal = data_;
}

// ============================================================================
// Калибровка
// ============================================================================

bool CalibrationModule::calibrate(const CalibrationPoint* points, int numPoints) {
    if (numPoints != 4) {
        return false;  // Поддерживаем только 4 точки
    }
    
    // Копируем точки
    for (int i = 0; i < 4; i++) {
        data_.points[i] = points[i];
    }
    
    // Вычисляем коэффициенты в зависимости от метода
    switch (data_.method) {
        case CAL_LINEAR:
            computeLinear();
            break;
        case CAL_PIECEWISE_LINEAR:
            computePiecewiseLinear();
            break;
        case CAL_POLYNOMIAL_2:
            computePolynomial(2);
            break;
        case CAL_POLYNOMIAL_3:
            computePolynomial(3);
            break;
    }
    
    // Вычисляем R²
    data_.r2 = computeR2();
    data_.timestamp = millis();
    
    // Проверяем качество калибровки
    return data_.r2 >= 0.99f;
}

// ============================================================================
// Линейная регрессия: y = kx + b
// ============================================================================

void CalibrationModule::computeLinear() {
    // Метод наименьших квадратов
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int n = 4;
    
    for (int i = 0; i < n; i++) {
        float x = (float)data_.points[i].rawADC;
        float y = data_.points[i].weight;
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }
    
    // k = (n*Σxy - Σx*Σy) / (n*Σx² - (Σx)²)
    float denominator = n * sum_x2 - sum_x * sum_x;
    if (fabs(denominator) < 1e-10f) {
        // Вырожденный случай
        data_.k = 0.0f;
        data_.b = sum_y / n;
        return;
    }
    
    data_.k = (n * sum_xy - sum_x * sum_y) / denominator;
    data_.b = (sum_y - data_.k * sum_x) / n;
}

// ============================================================================
// Кусочно-линейная интерполяция
// ============================================================================

void CalibrationModule::computePiecewiseLinear() {
    // Для кусочно-линейной интерполяции коэффициенты k и b не используются
    // Интерполяция выполняется в interpolatePiecewise()
    data_.k = 0.0f;
    data_.b = 0.0f;
}

float CalibrationModule::interpolatePiecewise(int32_t rawADC) const {
    // Находим сегмент бинарным поиском
    int segment = -1;
    for (int i = 0; i < 3; i++) {
        if (rawADC >= data_.points[i].rawADC && rawADC <= data_.points[i+1].rawADC) {
            segment = i;
            break;
        }
    }
    
    // Если вне диапазона, используем крайние сегменты
    if (segment == -1) {
        if (rawADC < data_.points[0].rawADC) {
            segment = 0;
        } else {
            segment = 2;
        }
    }
    
    // Линейная интерполяция в сегменте
    int32_t x0 = data_.points[segment].rawADC;
    int32_t x1 = data_.points[segment + 1].rawADC;
    float y0 = data_.points[segment].weight;
    float y1 = data_.points[segment + 1].weight;
    
    if (x1 == x0) {
        return y0;  // Вырожденный случай
    }
    
    // y = y0 + (y1 - y0) / (x1 - x0) * (x - x0)
    float k = (y1 - y0) / (float)(x1 - x0);
    return y0 + k * (float)(rawADC - x0);
}

// ============================================================================
// Полиномиальная регрессия: y = c0 + c1*x + c2*x² + c3*x³
// ============================================================================

void CalibrationModule::computePolynomial(int degree) {
    // Для упрощения используем метод наименьших квадратов
    // Решаем систему нормальных уравнений
    
    int n = 4;  // Количество точек
    int m = degree + 1;  // Количество коэффициентов
    
    // Обнуляем коэффициенты
    for (int i = 0; i < 4; i++) {
        data_.coeffs[i] = 0.0f;
    }
    
    // Для степени 2 и 3 используем прямое решение
    if (degree == 2) {
        // Полином 2-й степени: y = c0 + c1*x + c2*x²
        // Система 3x3
        float S[6] = {0};  // S[0]=Σx⁰, S[1]=Σx¹, ..., S[5]=Σx⁵
        float T[3] = {0};  // T[0]=Σy, T[1]=Σxy, T[2]=Σx²y
        
        for (int i = 0; i < n; i++) {
            float x = (float)data_.points[i].rawADC;
            float y = data_.points[i].weight;
            float xp = 1.0f;
            
            for (int j = 0; j <= 4; j++) {
                S[j] += xp;
                xp *= x;
            }
            
            T[0] += y;
            T[1] += x * y;
            T[2] += x * x * y;
        }
        
        // Решаем систему методом Крамера (для 3x3 это быстрее Гаусса)
        // [S0  S1  S2] [c0]   [T0]
        // [S1  S2  S3] [c1] = [T1]
        // [S2  S3  S4] [c2]   [T2]
        
        float det = S[0]*(S[2]*S[4] - S[3]*S[3]) - S[1]*(S[1]*S[4] - S[2]*S[3]) + S[2]*(S[1]*S[3] - S[2]*S[2]);
        
        if (fabs(det) < 1e-10f) {
            // Вырожденная система, fallback на линейную
            computeLinear();
            return;
        }
        
        data_.coeffs[0] = (T[0]*(S[2]*S[4] - S[3]*S[3]) - S[1]*(T[1]*S[4] - T[2]*S[3]) + S[2]*(T[1]*S[3] - T[2]*S[2])) / det;
        data_.coeffs[1] = (S[0]*(T[1]*S[4] - T[2]*S[3]) - T[0]*(S[1]*S[4] - S[2]*S[3]) + S[2]*(S[1]*T[2] - S[2]*T[1])) / det;
        data_.coeffs[2] = (S[0]*(S[2]*T[2] - S[3]*T[1]) - S[1]*(S[1]*T[2] - S[2]*T[1]) + T[0]*(S[1]*S[3] - S[2]*S[2])) / det;
        
    } else if (degree == 3) {
        // Полином 3-й степени: y = c0 + c1*x + c2*x² + c3*x³
        // Система 4x4 - используем метод Гаусса
        
        float S[7] = {0};  // S[0]=Σx⁰, S[1]=Σx¹, ..., S[6]=Σx⁶
        float T[4] = {0};  // T[0]=Σy, T[1]=Σxy, T[2]=Σx²y, T[3]=Σx³y
        
        for (int i = 0; i < n; i++) {
            float x = (float)data_.points[i].rawADC;
            float y = data_.points[i].weight;
            float xp = 1.0f;
            
            for (int j = 0; j <= 6; j++) {
                S[j] += xp;
                xp *= x;
            }
            
            T[0] += y;
            T[1] += x * y;
            T[2] += x * x * y;
            T[3] += x * x * x * y;
        }
        
        // Матрица A и вектор b
        float A[4][4];
        float b[4];
        
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                A[i][j] = S[i + j];
            }
            b[i] = T[i];
        }
        
        // Метод Гаусса с частичным выбором главного элемента
        for (int k = 0; k < 4; k++) {
            // Поиск главного элемента
            int maxRow = k;
            for (int i = k + 1; i < 4; i++) {
                if (fabs(A[i][k]) > fabs(A[maxRow][k])) {
                    maxRow = i;
                }
            }
            
            // Обмен строк
            if (maxRow != k) {
                for (int j = 0; j < 4; j++) {
                    float tmp = A[k][j];
                    A[k][j] = A[maxRow][j];
                    A[maxRow][j] = tmp;
                }
                float tmp = b[k];
                b[k] = b[maxRow];
                b[maxRow] = tmp;
            }
            
            // Проверка на вырожденность
            if (fabs(A[k][k]) < 1e-10f) {
                // Вырожденная система, fallback на линейную
                computeLinear();
                return;
            }
            
            // Прямой ход
            for (int i = k + 1; i < 4; i++) {
                float factor = A[i][k] / A[k][k];
                for (int j = k; j < 4; j++) {
                    A[i][j] -= factor * A[k][j];
                }
                b[i] -= factor * b[k];
            }
        }
        
        // Обратный ход
        for (int i = 3; i >= 0; i--) {
            data_.coeffs[i] = b[i];
            for (int j = i + 1; j < 4; j++) {
                data_.coeffs[i] -= A[i][j] * data_.coeffs[j];
            }
            data_.coeffs[i] /= A[i][i];
        }
    }
}

float CalibrationModule::evaluatePolynomial(int32_t rawADC) const {
    float x = (float)rawADC;
    float result = 0.0f;
    float xp = 1.0f;
    
    int degree = (data_.method == CAL_POLYNOMIAL_2) ? 2 : 3;
    
    for (int i = 0; i <= degree; i++) {
        result += data_.coeffs[i] * xp;
        xp *= x;
    }
    
    return result;
}

// ============================================================================
// Преобразование raw ADC → граммы
// ============================================================================

float CalibrationModule::rawToWeight(int32_t rawADC) const {
    switch (data_.method) {
        case CAL_LINEAR:
            return data_.k * (float)rawADC + data_.b;
            
        case CAL_PIECEWISE_LINEAR:
            return interpolatePiecewise(rawADC);
            
        case CAL_POLYNOMIAL_2:
        case CAL_POLYNOMIAL_3:
            return evaluatePolynomial(rawADC);
            
        default:
            return 0.0f;
    }
}

// ============================================================================
// Вычисление R² (коэффициент детерминации)
// ============================================================================

float CalibrationModule::computeR2() const {
    // R² = 1 - (SS_res / SS_tot)
    // SS_res = Σ(y_i - ŷ_i)²  // сумма квадратов остатков
    // SS_tot = Σ(y_i - ȳ)²    // общая сумма квадратов
    
    int n = 4;
    
    // Вычисляем среднее значение y
    float mean_y = 0.0f;
    for (int i = 0; i < n; i++) {
        mean_y += data_.points[i].weight;
    }
    mean_y /= n;
    
    // Вычисляем SS_res и SS_tot
    float ss_res = 0.0f;
    float ss_tot = 0.0f;
    
    for (int i = 0; i < n; i++) {
        float y_actual = data_.points[i].weight;
        float y_predicted = rawToWeight(data_.points[i].rawADC);
        
        float residual = y_actual - y_predicted;
        float deviation = y_actual - mean_y;
        
        ss_res += residual * residual;
        ss_tot += deviation * deviation;
    }
    
    // Проверка на вырожденный случай
    if (ss_tot < 1e-10f) {
        return 1.0f;  // Все y одинаковые
    }
    
    float r2 = 1.0f - (ss_res / ss_tot);
    
    // R² должен быть в диапазоне [0, 1]
    if (r2 < 0.0f) r2 = 0.0f;
    if (r2 > 1.0f) r2 = 1.0f;
    
    return r2;
}
