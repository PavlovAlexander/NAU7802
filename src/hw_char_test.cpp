/**
 * @file hw_char_test.cpp
 * @brief Тест характеризации железа: NAU7802 × тензодатчик. v1.2
 *
 * Изменения v1.2:
 *   - Калибровка расширена до 5 точек (0/5/10/20/30 г) + return-to-zero
 *   - 4 модели калибровки параллельно: LINEAR, PIECEWISE, POLY2, POLY3
 *   - Полиномиальная регрессия на double с нормализацией X (устойчивость)
 *   - Фаза 1: только gain=128, chopper=OFF, 4 SPS — убраны gain=64 и все чопперы
 *   - Фаза 3: вывод ошибки по всем 4 моделям + итоговая сравнительная таблица
 *
 * Все строки машинного лога — ASCII, тег [TAG] key=value, парсимы на Python/awk.
 */

#include "hw_char_test.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdarg.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <esp_task_wdt.h>

// ============================================================================
// Регистры NAU7802 (адреса по даташиту v1.7)
// ============================================================================

static constexpr uint8_t REG_PU_CTRL = 0x00;
static constexpr uint8_t REG_CTRL1   = 0x01;
static constexpr uint8_t REG_CTRL2   = 0x02;
static constexpr uint8_t REG_ADC_REG = 0x15;
static constexpr uint8_t REG_PGA     = 0x1B;
static constexpr uint8_t REG_POWER   = 0x1C;

static constexpr uint8_t MASK_CHP  = 0b00110000;
static constexpr uint8_t CHP_OFF   = 0b11;
static constexpr uint8_t CHP_CLK16 = 0b00;
static constexpr uint8_t CHP_CLK8  = 0b01;
static constexpr uint8_t CHP_CLK4  = 0b10;

static constexpr uint8_t GAIN_128 = NAU7802_GAIN_128;
static constexpr uint8_t GAIN_64  = NAU7802_GAIN_64;

static constexpr uint8_t SPS_10  = NAU7802_SPS_10;
static constexpr uint8_t SPS_40  = NAU7802_SPS_40;
static constexpr uint8_t SPS_80  = NAU7802_SPS_80;
static constexpr uint8_t SPS_320 = NAU7802_SPS_320;

// ============================================================================
// Константы теста
// ============================================================================

// Победитель из v1.1 — фиксируем
static constexpr uint8_t  REF_GAIN = GAIN_128;
static constexpr uint8_t  REF_SPS  = SPS_80;   // для settle/empty-check
static constexpr uint8_t  REF_CHP  = CHP_OFF;

// Точки калибровки (5 точек + return-to-zero)
static constexpr uint8_t  N_CAL_PTS = 5;  // 0, 5, 10, 20, 30 г
static constexpr float    CAL_LOADS_G[N_CAL_PTS] = {0.0f, 5.0f, 10.0f, 20.0f, 30.0f};

// Точки свипа нагрузок Фазы 3 (те же + return-to-zero)
static constexpr uint8_t  N_SWEEP_PTS = 6;  // 0a, 5, 10, 20, 30, 0b
static constexpr float    SWEEP_LOADS_G[N_SWEEP_PTS] = {0.0f, 5.0f, 10.0f, 20.0f, 30.0f, 0.0f};

// Кол-во отсчётов
static constexpr uint16_t N_CAL     = 200;  // калибровка
static constexpr uint16_t N_SWEEP   = 200;  // Фаза 1
static constexpr uint16_t N_LOAD    = 300;  // Фаза 3
static constexpr uint16_t N_DISCARD = 10;

// Паузы
static constexpr uint32_t SETTLE_MS     = 5000UL;  // settle после груза
static constexpr uint32_t CFG_SETTLE_MS = 300UL;   // settle после смены конфига

// Drift-тест
static constexpr uint32_t DRIFT_DURATION_MS = 120000UL;
static constexpr uint32_t DRIFT_WINDOW_MS   = 10000UL;

// Прочее
static constexpr uint32_t READ_TIMEOUT_MS        = 500UL;
static constexpr float    EMPTY_CHECK_SIGMA_MULT = 5.0f;

// ============================================================================
// Таймстамп и единый вывод
// ============================================================================

static uint32_t g_test_start_ms = 0;

/**
 * @brief Вывод с таймстампом [MM:SS] от старта теста.
 */
static void logPrintf(const char* fmt, ...) {
    uint32_t elapsed = millis() - g_test_start_ms;
    uint32_t mm = elapsed / 60000UL;
    uint32_t ss = (elapsed % 60000UL) / 1000UL;
    Serial.printf("[%02u:%02u] ", (unsigned)mm, (unsigned)ss);
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

// ============================================================================
// Структуры
// ============================================================================

struct SampleStats {
    double   mean;
    double   sigma;
    double   minVal;
    double   maxVal;
    double   p2p;
    double   slope;     // LSB/с
    uint16_t i2c_err;
    uint16_t n;
};

/**
 * @brief Результат 5-точечной калибровки со сравнением 4 моделей.
 *
 * Индексы моделей (CAL_MODEL_*):
 *   0 = LINEAR, 1 = PIECEWISE, 2 = POLY2, 3 = POLY3
 *
 * Для внутренних вычислений все X нормализованы:
 *   x_norm = (raw - x_mean) / x_scale
 * Это обеспечивает устойчивость полиномиальной регрессии при raw ~10⁶.
 */
static constexpr uint8_t CAL_MODEL_LINEAR    = 0;
static constexpr uint8_t CAL_MODEL_PIECEWISE = 1;
static constexpr uint8_t CAL_MODEL_POLY2     = 2;
static constexpr uint8_t CAL_MODEL_POLY3     = 3;
static constexpr uint8_t N_CAL_MODELS        = 4;

static const char* const CAL_MODEL_NAMES[N_CAL_MODELS] = {
    "LINEAR", "PIECEWISE", "POLY2", "POLY3"
};

struct CalResult5 {
    // Сырые данные 5 калибровочных точек + return-to-zero
    double raw_means[N_CAL_PTS + 1]; // [0..4] = точки, [5] = 0g_final
    double raw_sigma[N_CAL_PTS + 1];

    // Нормализация: raw_norm = (raw - x_mean) / x_scale
    double x_mean;   // среднее raw по 5 точкам
    double x_scale;  // полудиапазон (max-min)/2

    // Коэффициенты LINEAR: mg = k_lin * raw + b_lin
    double k_lin;
    double b_lin;

    // Коэффициенты POLY2: mg = p2[0] + p2[1]*x_n + p2[2]*x_n²
    double p2[3];

    // Коэффициенты POLY3: mg = p3[0] + p3[1]*x_n + p3[2]*x_n² + p3[3]*x_n³
    double p3[4];

    // Качество каждой модели (по 5 точкам)
    double r2[N_CAL_MODELS];
    double max_resid_mg[N_CAL_MODELS];
    double rms_resid_mg[N_CAL_MODELS];

    // Из линейной модели — для перевода LSB → мг в Фазах 1 и 2
    double k_lsb_per_mg;  // = 1 / k_lin (если k_lin в г, делим иначе)

    // Гистерезис (из return-to-zero)
    double hysteresis_mg;
};

struct SweepCombo {
    uint8_t gain_code;
    uint8_t sps_code;
    uint8_t chp_value;
};

// ============================================================================
// Вспомогательные функции — управление NAU7802
// ============================================================================

static void setChopper(NAU7802& scale, uint8_t chp_value) {
    uint8_t reg = scale.getRegister(REG_ADC_REG);
    reg = (reg & ~MASK_CHP) | ((chp_value & 0x03) << 4);
    scale.setRegister(REG_ADC_REG, reg);
}

static const char* chopperName(uint8_t chp_value) {
    switch (chp_value) {
        case CHP_OFF:   return "OFF";
        case CHP_CLK16: return "CLK16";
        case CHP_CLK8:  return "CLK8";
        case CHP_CLK4:  return "CLK4";
        default:        return "UNK";
    }
}

static int spsValue(uint8_t sps_code) {
    switch (sps_code) {
        case SPS_10:  return 10;
        case SPS_40:  return 40;
        case SPS_80:  return 80;
        case SPS_320: return 320;
        default:      return -1;
    }
}

static int gainValue(uint8_t gain_code) {
    switch (gain_code) {
        case GAIN_64:  return 64;
        case GAIN_128: return 128;
        default:       return -1;
    }
}

static void applyConfig(NAU7802& scale, uint8_t gain_code, uint8_t sps_code, uint8_t chp_value) {
    scale.setGain(gain_code);
    scale.setSampleRate(sps_code);
    setChopper(scale, chp_value);
    delay(CFG_SETTLE_MS);
}

// ============================================================================
// Сбор отсчётов
// ============================================================================

static bool waitReading(NAU7802& scale, uint32_t timeout_ms = READ_TIMEOUT_MS) {
    unsigned long t0 = millis();
    while (!scale.available()) {
        if (millis() - t0 > timeout_ms) return false;
        delay(1);
    }
    return true;
}

/**
 * @brief Собрать N отсчётов со статистикой.
 * @param label  Тег для прогресс-лога (nullptr — без прогресса).
 */
static SampleStats collectStats(NAU7802& scale, uint16_t n, const char* label = nullptr) {
    SampleStats s{};
    s.n       = 0;
    s.i2c_err = 0;

    static int32_t  buf[320];
    static uint32_t tbuf[320];

    uint16_t maxN = (n <= 320) ? n : 320;

    s.minVal = 1e18;
    s.maxVal = -1e18;
    double sum = 0.0, sum2 = 0.0;
    uint32_t t0 = millis();
    uint32_t lastProgress = t0;

    if (label) logPrintf("[COLLECT] %s: started n=%u\n", label, maxN);

    for (uint16_t i = 0; i < maxN; i++) {
        esp_task_wdt_reset();
        if (!waitReading(scale)) {
            s.i2c_err++;
            continue;
        }
        int32_t raw = scale.getReading();
        buf[s.n]    = raw;
        tbuf[s.n]   = millis() - t0;

        double v = (double)raw;
        sum  += v;
        sum2 += v * v;
        if (v < s.minVal) s.minVal = v;
        if (v > s.maxVal) s.maxVal = v;
        s.n++;

        if (label && (millis() - lastProgress >= 5000UL)) {
            uint32_t el = (millis() - t0) / 1000UL;
            logPrintf("[COLLECT] %s: %u/%u elapsed=%us\n", label, s.n, maxN, el);
            lastProgress = millis();
        }
    }

    if (label) {
        uint32_t el = (millis() - t0) / 1000UL;
        logPrintf("[COLLECT] %s: done n=%u/%u elapsed=%us i2c_err=%u\n",
                  label, s.n, maxN, el, s.i2c_err);
    }

    if (s.n < 2) {
        s.mean = s.sigma = s.slope = s.p2p = 0.0;
        return s;
    }

    s.mean = sum / s.n;
    double var = (sum2 - (double)s.n * s.mean * s.mean) / (s.n - 1);
    s.sigma    = (var > 0.0) ? sqrt(var) : 0.0;
    s.p2p      = s.maxVal - s.minVal;

    double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
    for (uint16_t i = 0; i < s.n; i++) {
        double t = tbuf[i] / 1000.0;
        double y = (double)buf[i];
        Sx  += t; Sy  += y;
        Sxx += t * t; Sxy += t * y;
    }
    double denom = (double)s.n * Sxx - Sx * Sx;
    s.slope = (denom != 0.0) ? ((double)s.n * Sxy - Sx * Sy) / denom : 0.0;

    return s;
}

static void discardReadings(NAU7802& scale, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        esp_task_wdt_reset();
        if (!waitReading(scale, READ_TIMEOUT_MS)) break;
        scale.getReading();
    }
}

// ============================================================================
// Ввод пользователя
// ============================================================================

static void waitAnyKey() {
    while (Serial.available()) Serial.read();
    while (!Serial.available()) { esp_task_wdt_reset(); delay(20); }
    char c = (char)Serial.read();
    while (Serial.available()) Serial.read();
    logPrintf("[KEY] '%c' received, continuing...\n", (c >= 32 && c < 127) ? c : '?');
}

static char waitChoice(const char* allowed) {
    while (true) {
        esp_task_wdt_reset();
        while (!Serial.available()) { esp_task_wdt_reset(); delay(20); }
        char c = (char)Serial.read();
        if (c >= 'A' && c <= 'Z') c += 32;
        while (Serial.available()) Serial.read();
        for (const char* p = allowed; *p; p++) {
            if (c == *p) {
                logPrintf("[KEY] '%c' selected\n", c);
                return c;
            }
        }
        logPrintf("[KEY] unknown '%c', expected one of: %s\n",
                  (c >= 32 && c < 127) ? c : '?', allowed);
    }
}

// ============================================================================
// Математика калибровки (double, нормализованный X)
// ============================================================================

/**
 * @brief Перевод raw LSB → мг по модели.
 *
 * x_norm = (raw - cal.x_mean) / cal.x_scale
 * LINEAR:    mg = k_lin * raw + b_lin
 * PIECEWISE: кусочно-линейная по 5 узлам
 * POLY2:     mg = p2[0] + p2[1]*x_n + p2[2]*x_n²
 * POLY3:     mg = p3[0] + p3[1]*x_n + p3[2]*x_n² + p3[3]*x_n³
 */
static double rawToMg(const CalResult5& cal, double raw, uint8_t model) {
    switch (model) {
        case CAL_MODEL_LINEAR:
            return cal.k_lin * raw + cal.b_lin;

        case CAL_MODEL_PIECEWISE: {
            // 5 узлов: CAL_LOADS_G × raw_means[0..4]
            // Найти сегмент
            int seg = -1;
            for (int i = 0; i < (int)N_CAL_PTS - 1; i++) {
                if (raw >= cal.raw_means[i] && raw <= cal.raw_means[i + 1]) {
                    seg = i;
                    break;
                }
            }
            if (seg == -1) {
                // Вне диапазона — экстраполяция крайним сегментом
                seg = (raw < cal.raw_means[0]) ? 0 : (N_CAL_PTS - 2);
            }
            double x0 = cal.raw_means[seg];
            double x1 = cal.raw_means[seg + 1];
            double y0 = (double)CAL_LOADS_G[seg] * 1000.0;
            double y1 = (double)CAL_LOADS_G[seg + 1] * 1000.0;
            if (x1 == x0) return y0;
            return y0 + (y1 - y0) / (x1 - x0) * (raw - x0);
        }

        case CAL_MODEL_POLY2: {
            double xn = (raw - cal.x_mean) / cal.x_scale;
            return cal.p2[0] + cal.p2[1] * xn + cal.p2[2] * xn * xn;
        }

        case CAL_MODEL_POLY3: {
            double xn = (raw - cal.x_mean) / cal.x_scale;
            return cal.p3[0] + cal.p3[1] * xn + cal.p3[2] * xn * xn + cal.p3[3] * xn * xn * xn;
        }

        default:
            return 0.0;
    }
}

/**
 * @brief Решить систему 3×3 методом Крамера.
 * A·x = b  →  x сохраняется в res[3].
 * @return false если матрица вырождена.
 */
static bool solve3x3(double A[3][3], double b[3], double res[3]) {
    // Определитель A
    double det = A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
               - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
               + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    if (fabs(det) < 1e-14) return false;

    for (int c = 0; c < 3; c++) {
        double M[3][3];
        for (int r = 0; r < 3; r++)
            for (int cc = 0; cc < 3; cc++)
                M[r][cc] = (cc == c) ? b[r] : A[r][cc];
        double d = M[0][0] * (M[1][1]*M[2][2] - M[1][2]*M[2][1])
                 - M[0][1] * (M[1][0]*M[2][2] - M[1][2]*M[2][0])
                 + M[0][2] * (M[1][0]*M[2][1] - M[1][1]*M[2][0]);
        res[c] = d / det;
    }
    return true;
}

/**
 * @brief Решить систему 4×4 методом Гаусса с частичным выбором главного элемента.
 * @return false если вырождена.
 */
static bool solve4x4(double A[4][4], double b[4], double res[4]) {
    double M[4][5];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) M[i][j] = A[i][j];
        M[i][4] = b[i];
    }
    for (int k = 0; k < 4; k++) {
        // Поиск главного элемента
        int pivot = k;
        for (int i = k + 1; i < 4; i++)
            if (fabs(M[i][k]) > fabs(M[pivot][k])) pivot = i;
        if (pivot != k)
            for (int j = 0; j <= 4; j++) { double t = M[k][j]; M[k][j] = M[pivot][j]; M[pivot][j] = t; }
        if (fabs(M[k][k]) < 1e-14) return false;
        for (int i = k + 1; i < 4; i++) {
            double f = M[i][k] / M[k][k];
            for (int j = k; j <= 4; j++) M[i][j] -= f * M[k][j];
        }
    }
    for (int i = 3; i >= 0; i--) {
        res[i] = M[i][4];
        for (int j = i + 1; j < 4; j++) res[i] -= M[i][j] * res[j];
        res[i] /= M[i][i];
    }
    return true;
}

/**
 * @brief Построить 4 модели по 5 калибровочным точкам.
 *
 * Все полиномы строятся на нормализованных X:
 *   x_n = (raw - x_mean) / x_scale
 * чтобы матрица Грама была хорошо обусловлена.
 */
static void fitAllModels(CalResult5& cal) {
    // --- Нормализация ---
    double x_min = cal.raw_means[0];
    double x_max = cal.raw_means[0];
    for (uint8_t i = 1; i < N_CAL_PTS; i++) {
        if (cal.raw_means[i] < x_min) x_min = cal.raw_means[i];
        if (cal.raw_means[i] > x_max) x_max = cal.raw_means[i];
    }
    cal.x_mean  = (x_min + x_max) * 0.5;
    cal.x_scale = (x_max - x_min) * 0.5;
    if (cal.x_scale < 1.0) cal.x_scale = 1.0; // защита от деления на 0

    double xn[N_CAL_PTS], y_mg[N_CAL_PTS];
    for (uint8_t i = 0; i < N_CAL_PTS; i++) {
        xn[i]   = (cal.raw_means[i] - cal.x_mean) / cal.x_scale;
        y_mg[i] = (double)CAL_LOADS_G[i] * 1000.0; // г → мг
    }
    int N = (int)N_CAL_PTS;

    // ---- LINEAR (МНК по нормализованным X, потом денормализуем) ----
    {
        double Sx=0, Sy=0, Sxx=0, Sxy=0;
        for (int i=0; i<N; i++) {
            Sx  += xn[i]; Sy  += y_mg[i];
            Sxx += xn[i]*xn[i]; Sxy += xn[i]*y_mg[i];
        }
        double denom = N*Sxx - Sx*Sx;
        double k_n, b_n; // коэффициенты по нормализованному X
        if (fabs(denom) > 1e-14) {
            k_n = (N*Sxy - Sx*Sy) / denom;
            b_n = (Sy - k_n*Sx) / N;
        } else {
            k_n = 0.0; b_n = Sy / N;
        }
        // Денормализация: mg = k_n * (raw - x_mean)/x_scale + b_n
        //                    = (k_n/x_scale)*raw + (b_n - k_n*x_mean/x_scale)
        cal.k_lin = k_n / cal.x_scale;
        cal.b_lin = b_n - k_n * cal.x_mean / cal.x_scale;
        // k_lsb_per_mg = 1 / k_lin (LSB на 1 мг), только если k_lin > 0
        cal.k_lsb_per_mg = (fabs(cal.k_lin) > 1e-30) ? (1.0 / cal.k_lin) : 1.0;
    }

    // ---- POLY2 (на нормализованных X) ----
    // МНК 5 точек → 3 коэффициента
    {
        double S[5] = {};  // S[k] = Σ xn^k
        double T[3] = {};  // T[k] = Σ xn^k * y
        for (int i=0; i<N; i++) {
            double xp = 1.0;
            for (int k=0; k<=4; k++) { S[k] += xp; xp *= xn[i]; }
            T[0] += y_mg[i];
            T[1] += xn[i]*y_mg[i];
            T[2] += xn[i]*xn[i]*y_mg[i];
        }
        double A[3][3] = {
            {S[0], S[1], S[2]},
            {S[1], S[2], S[3]},
            {S[2], S[3], S[4]}
        };
        double b[3] = {T[0], T[1], T[2]};
        if (!solve3x3(A, b, cal.p2)) {
            // fallback — копируем линейные
            cal.p2[0] = cal.b_lin;  // при x_n=0 mg=b
            cal.p2[1] = cal.k_lin * cal.x_scale; // денормализованный
            cal.p2[2] = 0.0;
        }
    }

    // ---- POLY3 (на нормализованных X) ----
    // МНК 5 точек → 4 коэффициента
    {
        double S[7] = {};
        double T[4] = {};
        for (int i=0; i<N; i++) {
            double xp = 1.0;
            for (int k=0; k<=6; k++) { S[k] += xp; xp *= xn[i]; }
            T[0] += y_mg[i];
            T[1] += xn[i]*y_mg[i];
            T[2] += xn[i]*xn[i]*y_mg[i];
            T[3] += xn[i]*xn[i]*xn[i]*y_mg[i];
        }
        double A[4][4] = {
            {S[0],S[1],S[2],S[3]},
            {S[1],S[2],S[3],S[4]},
            {S[2],S[3],S[4],S[5]},
            {S[3],S[4],S[5],S[6]}
        };
        double b[4] = {T[0],T[1],T[2],T[3]};
        if (!solve4x4(A, b, cal.p3)) {
            cal.p3[0]=cal.b_lin; cal.p3[1]=cal.k_lin*cal.x_scale;
            cal.p3[2]=0.0;       cal.p3[3]=0.0;
        }
    }

    // ---- Качество каждой модели ----
    for (uint8_t m = 0; m < N_CAL_MODELS; m++) {
        double ss_res = 0.0, ss_tot = 0.0;
        double mean_y = 0.0;
        for (int i=0; i<N; i++) mean_y += y_mg[i];
        mean_y /= N;

        double max_r = 0.0, sum_r2 = 0.0;
        for (int i=0; i<N; i++) {
            double pred = rawToMg(cal, cal.raw_means[i], m);
            double res  = y_mg[i] - pred;
            double dev  = y_mg[i] - mean_y;
            ss_res  += res * res;
            ss_tot  += dev * dev;
            double ar = fabs(res);
            if (ar > max_r) max_r = ar;
            sum_r2 += res * res;
        }
        cal.r2[m]           = (ss_tot > 0.0) ? (1.0 - ss_res/ss_tot) : 1.0;
        cal.max_resid_mg[m] = max_r;
        cal.rms_resid_mg[m] = sqrt(sum_r2 / N);
    }

    // ---- Гистерезис ----
    double hysteresis_lsb = cal.raw_means[5] - cal.raw_means[0]; // 0g_final - 0g
    cal.hysteresis_mg = hysteresis_lsb * cal.k_lin; // mg = lsb * (mg/lsb)
}

// ============================================================================
// Фаза 0.5 — 5-точечная калибровка с retry
// ============================================================================

static bool runPhase05(NAU7802& scale, CalResult5& cal) {
    logPrintf("[P05-START] 5-point calibration: 0/5/10/20/30g + return-to-zero\n");
    applyConfig(scale, REF_GAIN, REF_SPS, REF_CHP);

    static const char* prompts[N_CAL_PTS + 1] = {
        ">>> PUT LOAD: platform empty (0g), press any key",
        ">>> PUT LOAD: put 5g (F2), press any key",
        ">>> PUT LOAD: put 10g (F2), press any key",
        ">>> PUT LOAD: put 20g (F2), press any key",
        ">>> PUT LOAD: put 30g (F2), press any key",
        ">>> PUT LOAD: remove all load (0g return), press any key"
    };
    static const char* point_tag[N_CAL_PTS + 1] = {
        "0g", "5g", "10g", "20g", "30g", "0g_final"
    };

    for (uint8_t pt = 0; pt <= N_CAL_PTS; pt++) {
        logPrintf("[P05] %s\n", prompts[pt]);
        Serial.println(prompts[pt]);
        waitAnyKey();

        logPrintf("[P05] calib %s started (settle %us + %u samples @80SPS)\n",
                  point_tag[pt], (unsigned)(SETTLE_MS/1000), (unsigned)N_CAL);
        delay(SETTLE_MS);
        discardReadings(scale, N_DISCARD);

        char label[24];
        snprintf(label, sizeof(label), "P05:%s", point_tag[pt]);
        SampleStats s = collectStats(scale, N_CAL, label);

        cal.raw_means[pt] = s.mean;
        cal.raw_sigma[pt] = s.sigma;

        logPrintf("[P05] point=%s n=%u mean=%.9g sigma_lsb=%.3f i2c_err=%u\n",
                  point_tag[pt], s.n, s.mean, s.sigma, s.i2c_err);
    }

    // Построить все 4 модели
    fitAllModels(cal);

    // Вывести результаты
    logPrintf("[P05-LINEAR] k=%.9g b=%.9g r2=%.9g max_resid=%.3fmg rms=%.3fmg\n",
              cal.k_lin, cal.b_lin,
              cal.r2[CAL_MODEL_LINEAR],
              cal.max_resid_mg[CAL_MODEL_LINEAR],
              cal.rms_resid_mg[CAL_MODEL_LINEAR]);
    logPrintf("[P05-POLY2]  p=[%.6g, %.6g, %.6g] r2=%.9g max_resid=%.3fmg rms=%.3fmg\n",
              cal.p2[0], cal.p2[1], cal.p2[2],
              cal.r2[CAL_MODEL_POLY2],
              cal.max_resid_mg[CAL_MODEL_POLY2],
              cal.rms_resid_mg[CAL_MODEL_POLY2]);
    logPrintf("[P05-POLY3]  p=[%.6g, %.6g, %.6g, %.6g] r2=%.9g max_resid=%.3fmg rms=%.3fmg\n",
              cal.p3[0], cal.p3[1], cal.p3[2], cal.p3[3],
              cal.r2[CAL_MODEL_POLY3],
              cal.max_resid_mg[CAL_MODEL_POLY3],
              cal.rms_resid_mg[CAL_MODEL_POLY3]);
    logPrintf("[P05-PIECEWISE] r2=%.9g max_resid=%.3fmg rms=%.3fmg\n",
              cal.r2[CAL_MODEL_PIECEWISE],
              cal.max_resid_mg[CAL_MODEL_PIECEWISE],
              cal.rms_resid_mg[CAL_MODEL_PIECEWISE]);
    logPrintf("[P05-HYS] hysteresis_0g=%+.3fmg\n", cal.hysteresis_mg);

    // Таблица сравнения
    logPrintf("[P05-COMPARE] %-10s  %8s  %12s  %10s\n",
              "model", "r2", "max_resid_mg", "rms_mg");
    for (uint8_t m = 0; m < N_CAL_MODELS; m++) {
        logPrintf("[P05-COMPARE] %-10s  %8.6f  %12.3f  %10.3f\n",
                  CAL_MODEL_NAMES[m],
                  cal.r2[m],
                  cal.max_resid_mg[m],
                  cal.rms_resid_mg[m]);
    }

    // Подсказка
    double best_rms = cal.rms_resid_mg[0];
    uint8_t best_m  = 0;
    for (uint8_t m = 1; m < N_CAL_MODELS; m++) {
        if (cal.rms_resid_mg[m] < best_rms) {
            best_rms = cal.rms_resid_mg[m];
            best_m   = m;
        }
    }
    const char* hint = (cal.rms_resid_mg[CAL_MODEL_LINEAR] < 5.0) ? "good" :
                       (cal.rms_resid_mg[CAL_MODEL_LINEAR] < 50.0) ? "acceptable" : "poor, retry";
    logPrintf("[P05-HINT] linear=%s best_model=%s(rms=%.3fmg)\n",
              hint, CAL_MODEL_NAMES[best_m], best_rms);

    Serial.println(">>> DECISION: press A to accept, R to retry calibration");
    char choice = waitChoice("ar");
    logPrintf("[P05-DECISION] accepted=%s\n", (choice == 'a') ? "true" : "false");
    return (choice == 'a');
}

// ============================================================================
// Pre-flight check
// ============================================================================

static void ensureEmpty(NAU7802& scale, const CalResult5& cal) {
    logPrintf("[EMPTY-CHECK] verifying platform is empty...\n");
    applyConfig(scale, REF_GAIN, REF_SPS, REF_CHP);
    discardReadings(scale, 3);

    double raw0   = cal.raw_means[5]; // raw_0_final
    double sigma0 = cal.raw_sigma[5];

    while (true) {
        esp_task_wdt_reset();
        if (!waitReading(scale)) continue;
        double v = (double)scale.getReading();

        double threshold_sigma = EMPTY_CHECK_SIGMA_MULT * sigma0;
        // Минимальный порог — 500 мг в LSB (пропускает гистерезис, ловит реальные грузы ≥5 г)
        double threshold_lsb = (fabs(cal.k_lin) > 1e-30) ? (500.0 / fabs(cal.k_lin)) : 500.0;
        double threshold = (threshold_sigma > threshold_lsb) ? threshold_sigma : threshold_lsb;

        double delta_mg = fabs(v - raw0) * fabs(cal.k_lin);

        if (fabs(v - raw0) <= threshold) {
            logPrintf("[EMPTY-CHECK] OK (delta=%.1fmg threshold=%.1fmg)\n",
                      delta_mg, threshold * fabs(cal.k_lin));
            break;
        }

        logPrintf("[EMPTY-CHECK] FAIL delta=%.1fmg threshold=%.1fmg\n",
                  delta_mg, threshold * fabs(cal.k_lin));
        Serial.println(">>> WAIT: platform must be empty, remove load and press any key");
        Serial.println(">>>       (or press S to skip)");

        while (Serial.available()) Serial.read();
        while (!Serial.available()) { esp_task_wdt_reset(); delay(20); }
        char c = (char)Serial.read();
        if (c >= 'A' && c <= 'Z') c += 32;
        while (Serial.available()) Serial.read();

        if (c == 's') { logPrintf("[EMPTY-CHECK] skipped by user\n"); break; }
        logPrintf("[KEY] '%c' received, retrying...\n", (c >= 32 && c < 127) ? c : '?');
        discardReadings(scale, 5);
    }
}

// ============================================================================
// Фаза 1 — параметрический свип (только gain=128, chopper=OFF)
// ============================================================================

static void runPhase1(NAU7802& scale, const CalResult5& cal,
                      SweepCombo& bestCombo, double& bestSigmaMg) {
    logPrintf("[P1-START] SPS sweep: gain=128 chop=OFF (4 combos)\n");

    static const uint8_t spsCodes[] = {SPS_10, SPS_40, SPS_80, SPS_320};
    const uint8_t N_COMBOS = 4;

    bestSigmaMg = 1e18;
    int bestIdx = 0;

    for (uint8_t si = 0; si < N_COMBOS; si++) {
        esp_task_wdt_reset();
        uint8_t s = spsCodes[si];

        logPrintf("[P1] combo %d/%d gain=128 sps=%d chop=OFF started\n",
                  si + 1, N_COMBOS, spsValue(s));

        applyConfig(scale, GAIN_128, s, CHP_OFF);
        discardReadings(scale, N_DISCARD);

        char label[32];
        snprintf(label, sizeof(label), "P1:%d/%d sps%d", si+1, N_COMBOS, spsValue(s));
        SampleStats stats = collectStats(scale, N_SWEEP, label);

        double sigma_mg   = stats.sigma * fabs(cal.k_lin);
        double p2p_mg     = stats.p2p   * fabs(cal.k_lin);
        double slope_mg_s = stats.slope * cal.k_lin;

        logPrintf("[P1] idx=%-2d gain=128 sps=%-3d chop=OFF n=%u "
                  "mean=%.9g sigma_lsb=%.3f sigma_mg=%.3f "
                  "p2p_mg=%.3f slope_mg_s=%.4f i2c_err=%u\n",
                  si, spsValue(s), stats.n,
                  stats.mean, stats.sigma, sigma_mg,
                  p2p_mg, slope_mg_s, stats.i2c_err);

        if (sigma_mg < bestSigmaMg) {
            bestSigmaMg = sigma_mg;
            bestCombo   = {GAIN_128, s, CHP_OFF};
            bestIdx     = si;
        }
    }

    logPrintf("[P1-BEST] idx=%d gain=128 sps=%d chop=OFF sigma_mg=%.3f\n",
              bestIdx, spsValue(bestCombo.sps_code), bestSigmaMg);
    logPrintf("[P1-DONE] sweep complete\n");

    ensureEmpty(scale, cal);
}

// ============================================================================
// Фаза 2 — долговременный drift
// ============================================================================

static void runPhase2(NAU7802& scale, const CalResult5& cal) {
    logPrintf("[P2-START] long-term zero drift test (4 configs x 2min)\n");

    struct DriftCfg {
        const char* id;
        uint8_t gain; uint8_t sps; uint8_t chp;
        const char* desc;
    };
    static const DriftCfg cfgs[4] = {
        {"A", GAIN_128, SPS_80,  CHP_OFF,   "baseline_no_chop_80sps"},
        {"B", GAIN_128, SPS_10,  CHP_OFF,   "no_chop_10sps_50Hz_reject"},
        {"C", GAIN_128, SPS_80,  CHP_CLK4,  "chop_clk4_80sps"},
        {"D", GAIN_128, SPS_10,  CHP_CLK4,  "chop_clk4_10sps"},
    };

    for (uint8_t ci = 0; ci < 4; ci++) {
        const DriftCfg& cfg = cfgs[ci];

        applyConfig(scale, cfg.gain, cfg.sps, cfg.chp);
        delay(2000);
        discardReadings(scale, N_DISCARD);

        logPrintf("[P2-CFG] cfg=%s gain=%d sps=%d chop=%s desc=%s (2min)\n",
                  cfg.id, gainValue(cfg.gain), spsValue(cfg.sps),
                  chopperName(cfg.chp), cfg.desc);

        uint32_t phaseStart  = millis();
        uint32_t windowStart = phaseStart;
        uint32_t progressT   = phaseStart;

        double fSx=0,fSy=0,fSxx=0,fSxy=0;
        uint32_t fN = 0;
        double peakDev = 0.0, firstMean = 0.0;
        bool firstWindow = true;

        while (millis() - phaseStart < DRIFT_DURATION_MS) {
            esp_task_wdt_reset();

            if (millis() - progressT >= 5000UL) {
                uint32_t el = (millis() - phaseStart) / 1000;
                logPrintf("[PROGRESS] phase=2 cfg=%s elapsed=%us/%us\n",
                          cfg.id, el, (unsigned)(DRIFT_DURATION_MS/1000));
                progressT = millis();
            }

            if (millis() - windowStart >= DRIFT_WINDOW_MS) {
                uint16_t nWindow = (spsValue(cfg.sps) >= 80) ? 40 : 5;
                SampleStats ws   = collectStats(scale, nWindow);

                if (firstWindow) { firstMean = ws.mean; firstWindow = false; }

                double t_s      = (millis() - phaseStart) / 1000.0;
                double sigma_mg = ws.sigma * fabs(cal.k_lin);
                double mean_mg  = (ws.mean - cal.raw_means[0]) * cal.k_lin;

                logPrintf("[P2][cfg=%s] t=%.0fs mean_mg=%.3f sigma_mg=%.3f i2c_err=%u\n",
                          cfg.id, t_s, mean_mg, sigma_mg, ws.i2c_err);

                fSx  += t_s; fSy  += ws.mean;
                fSxx += t_s*t_s; fSxy += t_s*ws.mean;
                fN++;

                double dev = fabs(ws.mean - firstMean) * fabs(cal.k_lin);
                if (dev > peakDev) peakDev = dev;
                windowStart = millis();
            }
            delay(10);
        }

        double slope_lsb_s = 0.0;
        if (fN >= 2) {
            double denom = (double)fN * fSxx - fSx * fSx;
            if (denom != 0.0) slope_lsb_s = ((double)fN * fSxy - fSx * fSy) / denom;
        }
        double slope_mg_min = slope_lsb_s * 60.0 * cal.k_lin;

        logPrintf("[P2-DONE][cfg=%s] slope_mg_min=%+.4f peak_dev_mg=%.3f windows=%u\n",
                  cfg.id, slope_mg_min, peakDev, (unsigned)fN);
    }
    logPrintf("[P2-DONE] all drift configs complete\n");
}

// ============================================================================
// Фаза 3 — свип нагрузок с выводом по 4 моделям
// ============================================================================

static void runPhase3(NAU7802& scale, const CalResult5& cal,
                      const SweepCombo& bestCombo, double bestSigmaMg) {
    logPrintf("[P3-START] load sweep in best config: gain=%d sps=%d chop=%s sigma_mg=%.3f\n",
              gainValue(bestCombo.gain_code), spsValue(bestCombo.sps_code),
              chopperName(bestCombo.chp_value), bestSigmaMg);

    applyConfig(scale, bestCombo.gain_code, bestCombo.sps_code, bestCombo.chp_value);

    static const char* load_tag[N_SWEEP_PTS] = {"0a","5","10","20","30","0b"};
    static const char* prompts[N_SWEEP_PTS]  = {
        ">>> PUT LOAD: platform empty (0g), press any key",
        ">>> PUT LOAD: put 5g, press any key",
        ">>> PUT LOAD: put 10g, press any key",
        ">>> PUT LOAD: put 20g, press any key",
        ">>> PUT LOAD: put 30g, press any key",
        ">>> PUT LOAD: remove all load (0g return), press any key"
    };

    double means[N_SWEEP_PTS] = {};
    double sigma_mg_arr[N_SWEEP_PTS] = {};

    for (uint8_t pt = 0; pt < N_SWEEP_PTS; pt++) {
        esp_task_wdt_reset();
        logPrintf("[P3] %s\n", prompts[pt]);
        Serial.println(prompts[pt]);
        waitAnyKey();

        logPrintf("[P3] load=%s started (settle %us + %u samples)\n",
                  load_tag[pt], (unsigned)(SETTLE_MS/1000), (unsigned)N_LOAD);
        delay(SETTLE_MS);
        discardReadings(scale, N_DISCARD);

        char label[24];
        snprintf(label, sizeof(label), "P3:%s", load_tag[pt]);
        SampleStats s = collectStats(scale, N_LOAD, label);
        means[pt] = s.mean;

        double sm   = s.sigma * fabs(cal.k_lin);
        double p2pm = s.p2p   * fabs(cal.k_lin);
        double slm  = s.slope * cal.k_lin;
        sigma_mg_arr[pt] = sm;

        logPrintf("[P3][load=%s] n=%u raw_mean=%.9g sigma_mg=%.3f p2p_mg=%.3f "
                  "slope_mg_s=%.4f i2c_err=%u\n",
                  load_tag[pt], s.n, s.mean, sm, p2pm, slm, s.i2c_err);

        // Вывод по 4 моделям
        double nominal_mg = (double)SWEEP_LOADS_G[pt] * 1000.0;
        for (uint8_t m = 0; m < N_CAL_MODELS; m++) {
            double weight = rawToMg(cal, s.mean, m);
            double err    = weight - nominal_mg;
            logPrintf("[P3-MODEL][load=%s] %-10s weight=%.3fmg err=%+.3fmg\n",
                      load_tag[pt], CAL_MODEL_NAMES[m], weight, err);
        }
    }

    // ---- Итоговая сравнительная таблица ----
    logPrintf("[P3-COMPARE] model      max_err_mg  rms_err_mg  hys_mg\n");

    double hysteresis_mg = (means[5] - means[0]) * cal.k_lin;

    for (uint8_t m = 0; m < N_CAL_MODELS; m++) {
        double max_err = 0.0, sum_err2 = 0.0;
        int cnt = 0;
        for (uint8_t pt = 0; pt < N_SWEEP_PTS; pt++) {
            double nominal_mg = (double)SWEEP_LOADS_G[pt] * 1000.0;
            // return-to-zero (pt=5) — особый случай, сравниваем с 0 г
            double weight = rawToMg(cal, means[pt], m);
            double err    = fabs(weight - nominal_mg);
            if (err > max_err) max_err = err;
            sum_err2 += err * err;
            cnt++;
        }
        double rms_err = sqrt(sum_err2 / cnt);
        logPrintf("[P3-COMPARE] %-10s %10.3f %10.3f  %+8.3f\n",
                  CAL_MODEL_NAMES[m], max_err, rms_err, hysteresis_mg);
    }

    logPrintf("[P3-HYS] hysteresis_0g=%+.3fmg\n", hysteresis_mg);
    logPrintf("[P3-DONE] load sweep complete\n");
}

// ============================================================================
// runHwCharTest — точка входа
// ============================================================================

void runHwCharTest(NAU7802& scale) {

    g_test_start_ms = millis();

    Serial.println();
    Serial.println("============================================================");
    Serial.println("   HW-CHAR v1.2 — hardware characterization test");
    Serial.println("   4-model calibration: LINEAR / PIECEWISE / POLY2 / POLY3");
    Serial.println("============================================================");

    uint8_t reg_pu  = scale.getRegister(REG_PU_CTRL);
    uint8_t reg_c1  = scale.getRegister(REG_CTRL1);
    uint8_t reg_c2  = scale.getRegister(REG_CTRL2);
    uint8_t reg_adc = scale.getRegister(REG_ADC_REG);
    uint8_t reg_pga = scale.getRegister(REG_PGA);
    uint8_t reg_pwr = scale.getRegister(REG_POWER);

    Serial.println(">>> SELECT STAND: press S for shielded, O for open");
    char standChoice = waitChoice("so");
    const char* standName = (standChoice == 's') ? "shielded" : "open";

    logPrintf("[META] fw=HW-CHAR-v1.2 build=%s stand=%s\n", __DATE__, standName);
    logPrintf("[REG] PU_CTRL=0x%02X CTRL1=0x%02X CTRL2=0x%02X ADC=0x%02X PGA=0x%02X POWER=0x%02X\n",
              reg_pu, reg_c1, reg_c2, reg_adc, reg_pga, reg_pwr);

    uint8_t save_c1  = reg_c1;
    uint8_t save_c2  = reg_c2;
    uint8_t save_adc = reg_adc;

    // -----------------------------------------------------------------------
    // Фаза 0.5: калибровка
    // -----------------------------------------------------------------------
    CalResult5 cal{};
    int calAttempts = 0;
    while (true) {
        calAttempts++;
        logPrintf("[P05-ATTEMPT] n=%d\n", calAttempts);
        if (runPhase05(scale, cal)) break;
    }
    logPrintf("[META-CAL] cal_attempts=%d k_lsb_per_mg=%.6f\n",
              calAttempts, cal.k_lsb_per_mg);

    // -----------------------------------------------------------------------
    // Фаза 1: SPS-свип (gain=128, chop=OFF)
    // -----------------------------------------------------------------------
    logPrintf("[PHASE] starting Phase 1: SPS sweep (gain=128 chop=OFF)\n");
    ensureEmpty(scale, cal);

    SweepCombo bestCombo = {REF_GAIN, SPS_10, CHP_OFF};
    double bestSigmaMg = 1e18;
    runPhase1(scale, cal, bestCombo, bestSigmaMg);

    // -----------------------------------------------------------------------
    // Фаза 2: drift
    // -----------------------------------------------------------------------
    logPrintf("[PHASE] starting Phase 2: drift test (8min total)\n");
    ensureEmpty(scale, cal);
    runPhase2(scale, cal);

    // -----------------------------------------------------------------------
    // Фаза 3: свип нагрузок
    // -----------------------------------------------------------------------
    logPrintf("[PHASE] starting Phase 3: load sweep with 4-model comparison\n");
    runPhase3(scale, cal, bestCombo, bestSigmaMg);

    // -----------------------------------------------------------------------
    // Фаза 4: восстановление регистров и DONE
    // -----------------------------------------------------------------------
    scale.setRegister(REG_CTRL1,   save_c1);
    scale.setRegister(REG_CTRL2,   save_c2);
    scale.setRegister(REG_ADC_REG, save_adc);
    delay(500);

    uint32_t total_ms = millis() - g_test_start_ms;
    uint32_t tot_m    = total_ms / 60000UL;
    uint32_t tot_s    = (total_ms % 60000UL) / 1000UL;

    Serial.println();
    Serial.println("============================================================");
    Serial.printf("   TEST COMPLETE — stand=%s  total=%02u:%02u\n", standName, tot_m, tot_s);
    Serial.println("   Return to main menu. Press 'h' for help.");
    Serial.println("============================================================");
    logPrintf("[META] DONE stand=%s total_s=%lu\n",
              standName, (unsigned long)(total_ms/1000));
}
