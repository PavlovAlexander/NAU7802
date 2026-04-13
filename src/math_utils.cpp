#include "math_utils.h"
#include <math.h>

// Вычисляет коэффициенты k, b методом МНК по n точкам.
// x = rawADC (int32_t → float), y = weight (float)
// k = (Σ(x·y) − n·mean_x·mean_y) / (Σ(x²) − n·mean_x²)
// b = mean_y − k·mean_x
void linearRegression(const CalibrationPoint* points, int n,
                      float& k, float& b) {
    float sum_x  = 0.0f;
    float sum_y  = 0.0f;
    float sum_xy = 0.0f;
    float sum_x2 = 0.0f;

    for (int i = 0; i < n; i++) {
        float x = (float)points[i].rawADC;
        float y = points[i].weight;
        sum_x  += x;
        sum_y  += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    float mean_x = sum_x / (float)n;
    float mean_y = sum_y / (float)n;

    float denom = sum_x2 - (float)n * mean_x * mean_x;
    if (fabsf(denom) < 1e-10f) {
        // Все x одинаковы — регрессия не определена, возвращаем нули
        k = 0.0f;
        b = mean_y;
        return;
    }

    k = (sum_xy - (float)n * mean_x * mean_y) / denom;
    b = mean_y - k * mean_x;
}

// Вычисляет R² = 1 − SS_res / SS_tot
// SS_res = Σ(y_i − (k·x_i + b))²
// SS_tot = Σ(y_i − mean_y)²
float computeR2(const CalibrationPoint* points, int n,
                float k, float b) {
    float sum_y = 0.0f;
    for (int i = 0; i < n; i++) {
        sum_y += points[i].weight;
    }
    float mean_y = sum_y / (float)n;

    float ss_res = 0.0f;
    float ss_tot = 0.0f;
    for (int i = 0; i < n; i++) {
        float x    = (float)points[i].rawADC;
        float y    = points[i].weight;
        float pred = k * x + b;
        float dy   = y - mean_y;
        ss_res += (y - pred) * (y - pred);
        ss_tot += dy * dy;
    }

    if (fabsf(ss_tot) < 1e-10f) {
        // Все y одинаковы — R² не определён, возвращаем 1.0 (идеальное совпадение)
        return 1.0f;
    }

    return 1.0f - ss_res / ss_tot;
}
