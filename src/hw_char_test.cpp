/**
 * @file hw_char_test.cpp
 * @brief Тест характеризации железа: NAU7802 × тензодатчик.
 *
 * Параметрический свип gain×SPS×chopper, drift-тест и свип нагрузок.
 * Все строки машинного лога — ASCII, тег [TAG] key=value, парсимы на Python/awk.
 * Строки >>> WAIT / >>> DECISION — для пользователя, тоже ASCII.
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

// Маска битов REG_CHP [5:4] в регистре ADC
static constexpr uint8_t MASK_CHP    = 0b00110000;

// Значения REG_CHP (биты [5:4])
static constexpr uint8_t CHP_OFF     = 0b11; // чоппер отключён (сдвинуть << 4)
static constexpr uint8_t CHP_CLK16   = 0b00; // CLK/16
static constexpr uint8_t CHP_CLK8    = 0b01; // CLK/8
static constexpr uint8_t CHP_CLK4    = 0b10; // CLK/4

// Gain-коды для SparkFun lib
static constexpr uint8_t GAIN_128 = NAU7802_GAIN_128;
static constexpr uint8_t GAIN_64  = NAU7802_GAIN_64;

// SPS-коды для SparkFun lib
static constexpr uint8_t SPS_10  = NAU7802_SPS_10;
static constexpr uint8_t SPS_40  = NAU7802_SPS_40;
static constexpr uint8_t SPS_80  = NAU7802_SPS_80;
static constexpr uint8_t SPS_320 = NAU7802_SPS_320;

// ============================================================================
// Константы теста
// ============================================================================

// Референсная конфигурация для калибровки и всех settle-пауз
static constexpr uint8_t  REF_GAIN    = GAIN_128;
static constexpr uint8_t  REF_SPS     = SPS_80;
static constexpr uint8_t  REF_CHP     = CHP_OFF;

// Количество отсчётов на точку
static constexpr uint16_t N_CAL       = 200; // калибровка
static constexpr uint16_t N_SWEEP     = 200; // свип фазы 1
static constexpr uint16_t N_LOAD      = 300; // свип нагрузок фазы 3
static constexpr uint16_t N_DISCARD   = 10;  // отбрасываем после смены конфигурации

// Settle-пауза после смены конфигурации (до N_DISCARD)
static constexpr uint32_t SETTLE_MS   = 5000UL; // 5 с для грузов
static constexpr uint32_t CFG_SETTLE_MS = 300UL; // 300 мс после смены gain/SPS/chop

// Drift-тест: длительность одной конфигурации
static constexpr uint32_t DRIFT_DURATION_MS = 120000UL; // 2 мин
static constexpr uint32_t DRIFT_WINDOW_MS   = 10000UL;  // окно 10 с

// Таймаут ожидания отсчёта (при I2C-ошибках)
static constexpr uint32_t READ_TIMEOUT_MS = 500UL;

// Pre-flight: порог срабатывания «груз не снят» в sigma-множителях
static constexpr float EMPTY_CHECK_SIGMA_MULT = 5.0f;

// Прогресс-лог каждые N мс
static constexpr uint32_t PROGRESS_INTERVAL_MS = 10000UL;

// ============================================================================
// Таймстамп и единый вывод
// ============================================================================

/** Время старта runHwCharTest() в мс (от начала работы ESP). */
static uint32_t g_test_start_ms = 0;

/**
 * @brief Вывод строки лога с таймстампом [MM:SS] от старта теста.
 *
 * Использует va_list для поддержки printf-форматирования.
 * Пример: [03:47] [P1] idx=0 ...
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
    double  mean;
    double  sigma;
    double  minVal;
    double  maxVal;
    double  p2p;
    double  slope;    // LSB/с (линейный тренд по времени)
    uint16_t i2c_err;
    uint16_t n;
};

struct CalResult {
    double k_lsb_per_mg;  // LSB на 1 мг (при gain=128)
    double offset_lsb;    // смещение нуля (LSB)
    double r2;
    double resid_mg[4];   // residuals по точкам 0/10/20/30 г
    double hysteresis_mg;
    double raw_means[5];  // 0, 10, 20, 30, 0_final
    double raw_sigma[5];
};

struct SweepCombo {
    uint8_t gain_code;
    uint8_t sps_code;
    uint8_t chp_value; // значение для битов [5:4]
};

// ============================================================================
// Вспомогательные функции — управление NAU7802
// ============================================================================

/**
 * @brief Установить значение чоппера (биты [5:4] REG_ADC).
 * @param chp_value  0b00=CLK/16, 0b01=CLK/8, 0b10=CLK/4, 0b11=OFF
 */
static void setChopper(NAU7802& scale, uint8_t chp_value) {
    uint8_t reg = scale.getRegister(REG_ADC_REG);
    reg = (reg & ~MASK_CHP) | ((chp_value & 0x03) << 4);
    scale.setRegister(REG_ADC_REG, reg);
}

/** @brief Имя режима чоппера для лога. */
static const char* chopperName(uint8_t chp_value) {
    switch (chp_value) {
        case CHP_OFF:   return "OFF";
        case CHP_CLK16: return "CLK16";
        case CHP_CLK8:  return "CLK8";
        case CHP_CLK4:  return "CLK4";
        default:        return "UNK";
    }
}

/** @brief Имя SPS для лога. */
static int spsValue(uint8_t sps_code) {
    switch (sps_code) {
        case SPS_10:  return 10;
        case SPS_40:  return 40;
        case SPS_80:  return 80;
        case SPS_320: return 320;
        default:      return -1;
    }
}

/** @brief Имя gain для лога. */
static int gainValue(uint8_t gain_code) {
    switch (gain_code) {
        case GAIN_64:  return 64;
        case GAIN_128: return 128;
        default:       return -1;
    }
}

/**
 * @brief Применить конфигурацию gain/SPS/chopper и подождать CFG_SETTLE_MS.
 */
static void applyConfig(NAU7802& scale, uint8_t gain_code, uint8_t sps_code, uint8_t chp_value) {
    scale.setGain(gain_code);
    scale.setSampleRate(sps_code);
    setChopper(scale, chp_value);
    delay(CFG_SETTLE_MS);
}

// ============================================================================
// Вспомогательные функции — сбор отсчётов и статистика
// ============================================================================

/**
 * @brief Дождаться одного отсчёта с таймаутом.
 * @return true — отсчёт получен, false — таймаут/I2C ошибка.
 */
static bool waitReading(NAU7802& scale, uint32_t timeout_ms = READ_TIMEOUT_MS) {
    unsigned long t0 = millis();
    while (!scale.available()) {
        if (millis() - t0 > timeout_ms) return false;
        delay(1);
    }
    return true;
}

/**
 * @brief Собрать N отсчётов, вычислить статистику.
 *
 * Если получить отсчёт не удаётся за READ_TIMEOUT_MS — увеличивает i2c_err,
 * пропускает позицию (N_actual может быть < n).
 * slope рассчитывается методом МНК по (t_i, raw_i).
 * @param label  Тег для прогресс-лога (например "P05:0g", "P1:idx=3", "P3:10g").
 *               Передать nullptr для отключения прогресса.
 */
static SampleStats collectStats(NAU7802& scale, uint16_t n, const char* label = nullptr) {
    SampleStats s{};
    s.n      = 0;
    s.i2c_err = 0;

    // Буфер на стеке разумного размера; MAX_N = 320
    static int32_t buf[320];
    static uint32_t tbuf[320]; // время в мс от первого отсчёта

    uint16_t maxN = (n <= 320) ? n : 320;

    s.minVal = 1e18;
    s.maxVal = -1e18;
    double sum = 0.0, sum2 = 0.0;
    uint32_t t0 = millis();
    uint32_t lastProgress = t0;

    if (label) {
        logPrintf("[COLLECT] %s: started n=%u\n", label, maxN);
    }

    for (uint16_t i = 0; i < maxN; i++) {
        esp_task_wdt_reset();
        if (!waitReading(scale)) {
            s.i2c_err++;
            continue;
        }
        int32_t raw = scale.getReading();
        buf[s.n]  = raw;
        tbuf[s.n] = millis() - t0;

        double v = (double)raw;
        sum  += v;
        sum2 += v * v;
        if (v < s.minVal) s.minVal = v;
        if (v > s.maxVal) s.maxVal = v;
        s.n++;

        // Прогресс каждые 5 с
        if (label && (millis() - lastProgress >= 5000UL)) {
            uint32_t elapsed_s = (millis() - t0) / 1000UL;
            logPrintf("[COLLECT] %s: %u/%u elapsed=%us\n", label, s.n, maxN, elapsed_s);
            lastProgress = millis();
        }
    }

    if (label) {
        uint32_t elapsed_s = (millis() - t0) / 1000UL;
        logPrintf("[COLLECT] %s: done n=%u/%u elapsed=%us i2c_err=%u\n",
                  label, s.n, maxN, elapsed_s, s.i2c_err);
    }

    if (s.n < 2) {
        // Недостаточно данных
        s.mean = s.sigma = s.slope = 0.0;
        s.p2p  = 0.0;
        return s;
    }

    s.mean   = sum / s.n;
    double var = (sum2 - (double)s.n * s.mean * s.mean) / (s.n - 1);
    s.sigma  = (var > 0.0) ? sqrt(var) : 0.0;
    s.p2p    = s.maxVal - s.minVal;

    // МНК slope (LSB/с)
    double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
    for (uint16_t i = 0; i < s.n; i++) {
        double t = tbuf[i] / 1000.0; // с
        double y = (double)buf[i];
        Sx  += t;
        Sy  += y;
        Sxx += t * t;
        Sxy += t * y;
    }
    double denom = (double)s.n * Sxx - Sx * Sx;
    s.slope = (denom != 0.0) ? ((double)s.n * Sxy - Sx * Sy) / denom : 0.0;

    return s;
}

/**
 * @brief Отбросить первые n отсчётов (settling после смены конфигурации).
 */
static void discardReadings(NAU7802& scale, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        esp_task_wdt_reset();
        if (!waitReading(scale, READ_TIMEOUT_MS)) break;
        scale.getReading();
    }
}

// ============================================================================
// Вспомогательные функции — ввод пользователя
// ============================================================================

/**
 * @brief Ждать одиночного нажатия любой клавиши с эхом.
 */
static void waitAnyKey() {
    while (Serial.available()) Serial.read();
    while (!Serial.available()) {
        esp_task_wdt_reset();
        delay(20);
    }
    char c = (char)Serial.read();
    while (Serial.available()) Serial.read();
    logPrintf("[KEY] '%c' received, continuing...\n", (c >= 32 && c < 127) ? c : '?');
}

/**
 * @brief Ждать нажатия конкретной клавиши из набора allowed (без учёта регистра).
 * @return нажатая буква в нижнем регистре.
 */
static char waitChoice(const char* allowed) {
    while (true) {
        esp_task_wdt_reset();
        while (!Serial.available()) {
            esp_task_wdt_reset();
            delay(20);
        }
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
// Фаза 0.5 — калибровка (внутренняя функция)
// ============================================================================

/**
 * @brief Провести 4-точечную калибровку. Возвращает true при успехе (A).
 * @param[out] cal  Результат калибровки.
 * @param[out] raw0 mean ФИНАЛЬНОЙ точки 0 г (raw_0_final, для empty-check фазы 1).
 * @param[out] sigma0_lsb sigma финальной точки 0 г.
 */
static bool runPhase05(NAU7802& scale, CalResult& cal, double& raw0_baseline, double& sigma0_baseline) {
    logPrintf("[P05-START] 4-point calibration (0/10/20/30g + return 0g)\n");

    // Референсная конфигурация для калибровки
    applyConfig(scale, REF_GAIN, REF_SPS, REF_CHP);

    static const float load_g[4]  = {0.0f, 10.0f, 20.0f, 30.0f};
    static const char* point_tag[5] = {"0g", "10g", "20g", "30g", "0g_final"};
    static const char* prompts[5] = {
        ">>> PUT LOAD: platform empty (0g), press any key when ready",
        ">>> PUT LOAD: put 10g (F2 class), press any key when ready",
        ">>> PUT LOAD: put 20g (F2 class), press any key when ready",
        ">>> PUT LOAD: put 30g (F2 class), press any key when ready",
        ">>> PUT LOAD: remove all load (0g), press any key when ready"
    };

    for (uint8_t pt = 0; pt < 5; pt++) {
        logPrintf("[P05] waiting for user: %s\n", prompts[pt]);
        Serial.println(prompts[pt]);
        waitAnyKey();

        logPrintf("[P05] calib %s started (settling %us + %u samples @80SPS ~%us)\n",
                  point_tag[pt],
                  (unsigned)(SETTLE_MS / 1000),
                  (unsigned)N_CAL,
                  (unsigned)(SETTLE_MS / 1000 + N_CAL / 80));

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

    // МНК через 4 точки (0, 10, 20, 30 г)
    // raw = k_lsb_per_mg * mass_mg + offset_lsb
    // mass_mg = {0, 10000, 20000, 30000}
    double Sm  = 0.0, Sr  = 0.0, Smm = 0.0, Smr = 0.0;
    for (uint8_t i = 0; i < 4; i++) {
        double m = (double)load_g[i] * 1000.0; // г → мг
        double r = cal.raw_means[i];
        Sm  += m;
        Sr  += r;
        Smm += m * m;
        Smr += m * r;
    }
    double denom = 4.0 * Smm - Sm * Sm;
    if (denom == 0.0) {
        Serial.println("[P05-ERROR] degenerate regression");
        return false;
    }
    cal.k_lsb_per_mg = (4.0 * Smr - Sm * Sr) / denom;
    cal.offset_lsb   = (Sr - cal.k_lsb_per_mg * Sm) / 4.0;

    // R²
    double mean_r = Sr / 4.0;
    double ss_tot = 0.0, ss_res = 0.0;
    for (uint8_t i = 0; i < 4; i++) {
        double m     = (double)load_g[i] * 1000.0;
        double r_hat = cal.k_lsb_per_mg * m + cal.offset_lsb;
        double diff  = cal.raw_means[i] - mean_r;
        double res   = cal.raw_means[i] - r_hat;
        ss_tot += diff * diff;
        ss_res += res  * res;
        cal.resid_mg[i] = (cal.k_lsb_per_mg != 0.0) ? (res / cal.k_lsb_per_mg) : 0.0;
    }
    cal.r2 = (ss_tot > 0.0) ? (1.0 - ss_res / ss_tot) : 1.0;

    // Гистерезис (raw_0_final vs raw_0)
    cal.hysteresis_mg = (cal.k_lsb_per_mg != 0.0)
        ? ((cal.raw_means[4] - cal.raw_means[0]) / cal.k_lsb_per_mg)
        : 0.0;

    logPrintf("[P05-CAL] k_lsb_per_mg=%.6f offset_lsb=%.3f r2=%.9g\n",
              cal.k_lsb_per_mg, cal.offset_lsb, cal.r2);
    logPrintf("[P05-RES] resid_0=%+.3fmg resid_10=%+.3fmg resid_20=%+.3fmg resid_30=%+.3fmg\n",
              cal.resid_mg[0], cal.resid_mg[1], cal.resid_mg[2], cal.resid_mg[3]);
    logPrintf("[P05-HYS] hysteresis_0g=%+.3fmg\n", cal.hysteresis_mg);

    // Подсказка качества
    double maxResid = 0.0;
    for (uint8_t i = 0; i < 4; i++) {
        double a = fabs(cal.resid_mg[i]);
        if (a > maxResid) maxResid = a;
    }
    double hysAbs = fabs(cal.hysteresis_mg);
    const char* hint;
    if (cal.r2 >= 0.99999 && maxResid < 0.5 && hysAbs < 0.5) {
        hint = "excellent";
    } else if (cal.r2 >= 0.9999 && maxResid < 2.0 && hysAbs < 2.0) {
        hint = "good";
    } else {
        hint = "poor, consider retry";
    }
    logPrintf("[P05-HINT] quality=%s r2=%.9g max_resid=%.3fmg hys=%.3fmg\n",
              hint, cal.r2, maxResid, hysAbs);

    // Решение пользователя
    Serial.println(">>> DECISION: press A to accept, R to retry calibration");
    char choice = waitChoice("ar");
    logPrintf("[P05-DECISION] accepted=%s\n", (choice == 'a') ? "true" : "false");

    if (choice == 'a') {
        // ВАЖНО: используем raw_0_FINAL (после гистерезиса), а не raw_0_initial
        // иначе empty-check будет ловить гистерезис как "груз на платформе"
        raw0_baseline   = cal.raw_means[4];
        sigma0_baseline = cal.raw_sigma[4];
        return true;
    }
    return false;
}

// ============================================================================
// Pre-flight check: убедиться что платформа пуста
// ============================================================================

static void ensureEmpty(NAU7802& scale, double raw0, double sigma0, double k_lsb_per_mg) {
    logPrintf("[EMPTY-CHECK] verifying platform is empty...\n");
    applyConfig(scale, REF_GAIN, REF_SPS, REF_CHP);
    discardReadings(scale, 3);

    while (true) {
        esp_task_wdt_reset();
        if (!waitReading(scale)) continue;
        double v = (double)scale.getReading();

        // Порог: минимум 5×σ или 500 мг (что больше)
        double threshold_sigma = EMPTY_CHECK_SIGMA_MULT * sigma0;
        double threshold_mg    = 500.0;
        double threshold_lsb   = (k_lsb_per_mg > 0.0) ? (threshold_mg * k_lsb_per_mg) : 500.0;
        double threshold       = (threshold_sigma > threshold_lsb) ? threshold_sigma : threshold_lsb;

        double delta_mg = (k_lsb_per_mg > 0.0) ? (fabs(v - raw0) / k_lsb_per_mg) : fabs(v - raw0);

        if (fabs(v - raw0) <= threshold) {
            logPrintf("[EMPTY-CHECK] OK (delta=%.1fmg threshold=%.1fmg)\n",
                      delta_mg, (k_lsb_per_mg > 0.0) ? threshold / k_lsb_per_mg : threshold);
            break;
        }

        logPrintf("[EMPTY-CHECK] FAIL delta=%.1fmg threshold=%.1fmg\n",
                  delta_mg, (k_lsb_per_mg > 0.0) ? threshold / k_lsb_per_mg : threshold);
        Serial.println(">>> WAIT: platform must be empty, remove load and press any key");
        Serial.println(">>>       (or press S to skip this check)");

        while (Serial.available()) Serial.read();
        while (!Serial.available()) {
            esp_task_wdt_reset();
            delay(20);
        }
        char c = (char)Serial.read();
        if (c >= 'A' && c <= 'Z') c += 32;
        while (Serial.available()) Serial.read();

        if (c == 's') {
            logPrintf("[EMPTY-CHECK] skipped by user\n");
            break;
        }
        logPrintf("[KEY] '%c' received, retrying check...\n", (c >= 32 && c < 127) ? c : '?');
        discardReadings(scale, 5);
    }
}

// ============================================================================
// Фаза 1 — параметрический свип
// ============================================================================

static void runPhase1(NAU7802& scale, const CalResult& cal,
                      double raw0, double sigma0,
                      SweepCombo& bestCombo, double& bestSigmaMg) {
    logPrintf("[P1-START] gain x sps x chopper sweep (32 combos)\n");

    static const uint8_t gains[]    = {GAIN_64,  GAIN_128};
    static const uint8_t spsCodes[] = {SPS_10, SPS_40, SPS_80, SPS_320};
    static const uint8_t chpVals[]  = {CHP_OFF, CHP_CLK16, CHP_CLK8, CHP_CLK4};

    bestSigmaMg = 1e18;
    int bestIdx = 0;
    int idx = 0;

    for (uint8_t gi = 0; gi < 2; gi++) {
        for (uint8_t si = 0; si < 4; si++) {
            for (uint8_t ci = 0; ci < 4; ci++) {

                esp_task_wdt_reset();

                uint8_t g = gains[gi];
                uint8_t s = spsCodes[si];
                uint8_t c = chpVals[ci];

                logPrintf("[P1] combo %d/32 gain=%d sps=%d chop=%s started\n",
                          idx + 1, gainValue(g), spsValue(s), chopperName(c));

                applyConfig(scale, g, s, c);
                discardReadings(scale, N_DISCARD);

                char label[32];
                snprintf(label, sizeof(label), "P1:%d/32 g%d s%d %s",
                         idx + 1, gainValue(g), spsValue(s), chopperName(c));
                SampleStats stats = collectStats(scale, N_SWEEP, label);

                // Перевод sigma в мг с поправкой на gain
                double gainFactor = (g == GAIN_128) ? 1.0 : 0.5;
                double kEff       = (cal.k_lsb_per_mg != 0.0) ? (cal.k_lsb_per_mg * gainFactor) : 1.0;
                double sigma_mg   = stats.sigma / kEff;
                double p2p_mg     = stats.p2p   / kEff;
                double slope_mg_s = stats.slope  / kEff;

                // SKIPPED если слишком много I2C-ошибок (>10% от N_SWEEP)
                bool skipped = (stats.i2c_err > N_SWEEP / 10);

                if (skipped) {
                    logPrintf("[P1] idx=%-2d gain=%-3d sps=%-3d chop=%-5s SKIPPED i2c_err=%u\n",
                              idx, gainValue(g), spsValue(s), chopperName(c), stats.i2c_err);
                } else {
                    logPrintf("[P1] idx=%-2d gain=%-3d sps=%-3d chop=%-5s n=%u "
                              "mean=%.9g sigma_lsb=%.3f sigma_mg=%.3f "
                              "p2p_mg=%.3f slope_mg_s=%.4f i2c_err=%u\n",
                              idx, gainValue(g), spsValue(s), chopperName(c), stats.n,
                              stats.mean, stats.sigma, sigma_mg,
                              p2p_mg, slope_mg_s, stats.i2c_err);

                    if (sigma_mg < bestSigmaMg) {
                        bestSigmaMg = sigma_mg;
                        bestCombo = {g, s, c};
                        bestIdx   = idx;
                    }
                }

                idx++;
            }
        }
    }

    logPrintf("[P1-BEST] idx=%d gain=%d sps=%d chop=%s sigma_mg=%.3f\n",
              bestIdx,
              gainValue(bestCombo.gain_code),
              spsValue(bestCombo.sps_code),
              chopperName(bestCombo.chp_value),
              bestSigmaMg);
    logPrintf("[P1-DONE] sweep complete\n");

    ensureEmpty(scale, raw0, sigma0, cal.k_lsb_per_mg);
}

// ============================================================================
// Фаза 2 — долговременный drift
// ============================================================================

static void runPhase2(NAU7802& scale, const CalResult& cal) {
    logPrintf("[P2-START] long-term zero drift test (4 configs x 2min)\n");

    struct DriftCfg {
        const char* id;
        uint8_t gain;
        uint8_t sps;
        uint8_t chp;
        const char* desc;
    };

    static const DriftCfg cfgs[4] = {
        {"A", GAIN_128, SPS_80,  CHP_OFF,   "baseline_no_chop_80sps"},
        {"B", GAIN_128, SPS_10,  CHP_OFF,   "no_chop_10sps_50Hz_reject"},
        {"C", GAIN_128, SPS_80,  CHP_CLK4,  "chop_clk4_80sps"},
        {"D", GAIN_128, SPS_10,  CHP_CLK4,  "chop_clk4_10sps_best_expected"},
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

        double fSx = 0.0, fSy = 0.0, fSxx = 0.0, fSxy = 0.0;
        uint32_t fN = 0;
        double peakDev = 0.0;
        double firstMean = 0.0;
        bool firstWindow = true;

        while (millis() - phaseStart < DRIFT_DURATION_MS) {
            esp_task_wdt_reset();

            // Прогресс каждые 5 с
            if (millis() - progressT >= 5000UL) {
                uint32_t elapsed = (millis() - phaseStart) / 1000;
                logPrintf("[PROGRESS] phase=2 cfg=%s elapsed=%us/%us\n",
                          cfg.id, elapsed, (unsigned)(DRIFT_DURATION_MS / 1000));
                progressT = millis();
            }

            // Окно каждые 10 с
            if (millis() - windowStart >= DRIFT_WINDOW_MS) {
                uint16_t nWindow = (spsValue(cfg.sps) >= 80) ? 40 : 5;
                SampleStats ws = collectStats(scale, nWindow); // без label — быстрые замеры

                if (firstWindow) {
                    firstMean  = ws.mean;
                    firstWindow = false;
                }

                double t_s  = (millis() - phaseStart) / 1000.0;
                double kEff = (cal.k_lsb_per_mg != 0.0) ? cal.k_lsb_per_mg : 1.0;
                double sigma_mg = ws.sigma / kEff;
                double mean_mg  = (ws.mean - cal.offset_lsb) / kEff;

                logPrintf("[P2][cfg=%s] t=%.0fs mean_mg=%.3f sigma_mg=%.3f i2c_err=%u\n",
                          cfg.id, t_s, mean_mg, sigma_mg, ws.i2c_err);

                fSx  += t_s;
                fSy  += ws.mean;
                fSxx += t_s * t_s;
                fSxy += t_s * ws.mean;
                fN++;

                double dev = fabs(ws.mean - firstMean) / kEff;
                if (dev > peakDev) peakDev = dev;

                windowStart = millis();
            }

            delay(10);
        }

        double slope_lsb_s = 0.0;
        if (fN >= 2) {
            double denom = (double)fN * fSxx - fSx * fSx;
            if (denom != 0.0) {
                slope_lsb_s = ((double)fN * fSxy - fSx * fSy) / denom;
            }
        }

        double kEff         = (cal.k_lsb_per_mg != 0.0) ? cal.k_lsb_per_mg : 1.0;
        double slope_mg_min = slope_lsb_s * 60.0 / kEff;

        logPrintf("[P2-DONE][cfg=%s] slope_mg_min=%+.4f peak_dev_mg=%.3f windows=%u\n",
                  cfg.id, slope_mg_min, peakDev, (unsigned)fN);
    }

    logPrintf("[P2-DONE] all drift configs complete\n");
}

// ============================================================================
// Фаза 3 — свип нагрузок
// ============================================================================

static void runPhase3(NAU7802& scale, const CalResult& cal,
                      const SweepCombo& bestCombo, double bestSigmaMg) {
    logPrintf("[P3-START] load sweep in best config: gain=%d sps=%d chop=%s sigma_mg=%.3f\n",
              gainValue(bestCombo.gain_code),
              spsValue(bestCombo.sps_code),
              chopperName(bestCombo.chp_value),
              bestSigmaMg);

    applyConfig(scale, bestCombo.gain_code, bestCombo.sps_code, bestCombo.chp_value);

    static const float load_g[5]   = {0.0f, 10.0f, 20.0f, 30.0f, 0.0f};
    static const char* load_tag[5] = {"0a", "10", "20", "30", "0b"};
    static const char* prompts[5]  = {
        ">>> PUT LOAD: platform empty (0g), press any key when ready",
        ">>> PUT LOAD: put 10g, press any key when ready",
        ">>> PUT LOAD: put 20g, press any key when ready",
        ">>> PUT LOAD: put 30g, press any key when ready",
        ">>> PUT LOAD: remove load (0g again), press any key when ready"
    };

    double means[5] = {};
    double kEff = (cal.k_lsb_per_mg != 0.0) ? cal.k_lsb_per_mg : 1.0;
    if (bestCombo.gain_code == GAIN_64) kEff *= 0.5;

    for (uint8_t pt = 0; pt < 5; pt++) {
        esp_task_wdt_reset();
        logPrintf("[P3] waiting for user: %s\n", prompts[pt]);
        Serial.println(prompts[pt]);
        waitAnyKey();

        logPrintf("[P3] load=%s started (settling %us + %u samples)\n",
                  load_tag[pt], (unsigned)(SETTLE_MS / 1000), (unsigned)N_LOAD);
        delay(SETTLE_MS);
        discardReadings(scale, N_DISCARD);

        char label[24];
        snprintf(label, sizeof(label), "P3:%s", load_tag[pt]);
        SampleStats s = collectStats(scale, N_LOAD, label);
        means[pt] = s.mean;

        double sigma_mg   = s.sigma / kEff;
        double p2p_mg     = s.p2p   / kEff;
        double slope_mg_s = s.slope / kEff;

        logPrintf("[P3][load=%s] n=%u mean=%.9g sigma_mg=%.3f p2p_mg=%.3f "
                  "slope_mg_s=%.4f i2c_err=%u\n",
                  load_tag[pt], s.n, s.mean, sigma_mg, p2p_mg,
                  slope_mg_s, s.i2c_err);
    }

    // Линейность: 4 точки (0a, 10, 20, 30) vs номинальные значения
    double Sm = 0.0, Sr = 0.0, Smm = 0.0, Smr = 0.0;
    for (uint8_t i = 0; i < 4; i++) {
        double m = (double)load_g[i] * 1000.0;
        double r = means[i];
        Sm  += m;
        Sr  += r;
        Smm += m * m;
        Smr += m * r;
    }
    double denom = 4.0 * Smm - Sm * Sm;
    double r2_lin = 0.0;
    if (denom != 0.0) {
        double k2 = (4.0 * Smr - Sm * Sr) / denom;
        double b2 = (Sr - k2 * Sm) / 4.0;
        double meanR = Sr / 4.0;
        double ssTot = 0.0, ssRes = 0.0;
        for (uint8_t i = 0; i < 4; i++) {
            double m = (double)load_g[i] * 1000.0;
            double diff = means[i] - meanR;
            double res  = means[i] - (k2 * m + b2);
            ssTot += diff * diff;
            ssRes += res  * res;
        }
        r2_lin = (ssTot > 0.0) ? (1.0 - ssRes / ssTot) : 1.0;
    }

    double hysteresis_mg = (means[4] - means[0]) / kEff;

    logPrintf("[P3-LINEARITY] r2=%.9g hysteresis_0g=%+.3fmg\n",
              r2_lin, hysteresis_mg);
    logPrintf("[P3-DONE] load sweep complete\n");
}

// ============================================================================
// runHwCharTest — точка входа
// ============================================================================

void runHwCharTest(NAU7802& scale) {

    g_test_start_ms = millis();

    // -----------------------------------------------------------------------
    // Фаза 0: заголовок, регистры, выбор стенда
    // -----------------------------------------------------------------------
    Serial.println();
    Serial.println("============================================================");
    Serial.println("   HW-CHAR v1.1 — hardware characterization test");
    Serial.println("============================================================");

    uint8_t reg_pu   = scale.getRegister(REG_PU_CTRL);
    uint8_t reg_c1   = scale.getRegister(REG_CTRL1);
    uint8_t reg_c2   = scale.getRegister(REG_CTRL2);
    uint8_t reg_adc  = scale.getRegister(REG_ADC_REG);
    uint8_t reg_pga  = scale.getRegister(REG_PGA);
    uint8_t reg_pwr  = scale.getRegister(REG_POWER);

    Serial.println(">>> SELECT STAND: press S for shielded, O for open");
    char standChoice = waitChoice("so");
    const char* standName = (standChoice == 's') ? "shielded" : "open";

    logPrintf("[META] fw=HW-CHAR-v1.1 build=%s stand=%s\n", __DATE__, standName);
    logPrintf("[REG] PU_CTRL=0x%02X CTRL1=0x%02X CTRL2=0x%02X ADC=0x%02X PGA=0x%02X POWER=0x%02X\n",
              reg_pu, reg_c1, reg_c2, reg_adc, reg_pga, reg_pwr);

    uint8_t save_c1  = reg_c1;
    uint8_t save_c2  = reg_c2;
    uint8_t save_adc = reg_adc;

    // -----------------------------------------------------------------------
    // Фаза 0.5: калибровка с циклом retry
    // -----------------------------------------------------------------------
    CalResult cal{};
    double raw0_baseline   = 0.0;
    double sigma0_baseline = 10.0;

    int calAttempts = 0;
    while (true) {
        calAttempts++;
        logPrintf("[P05-ATTEMPT] n=%d\n", calAttempts);
        if (runPhase05(scale, cal, raw0_baseline, sigma0_baseline)) break;
    }
    logPrintf("[META-CAL] cal_attempts=%d k_lsb_per_mg=%.6f\n",
              calAttempts, cal.k_lsb_per_mg);

    // -----------------------------------------------------------------------
    // Фаза 1: параметрический свип
    // -----------------------------------------------------------------------
    logPrintf("[PHASE] starting Phase 1: parametric sweep\n");
    ensureEmpty(scale, raw0_baseline, sigma0_baseline, cal.k_lsb_per_mg);

    SweepCombo bestCombo = {REF_GAIN, REF_SPS, REF_CHP};
    double bestSigmaMg = 1e18;
    runPhase1(scale, cal, raw0_baseline, sigma0_baseline, bestCombo, bestSigmaMg);

    // -----------------------------------------------------------------------
    // Фаза 2: drift-тест
    // -----------------------------------------------------------------------
    logPrintf("[PHASE] starting Phase 2: drift test (8min total)\n");
    ensureEmpty(scale, raw0_baseline, sigma0_baseline, cal.k_lsb_per_mg);
    runPhase2(scale, cal);

    // -----------------------------------------------------------------------
    // Фаза 3: свип нагрузок
    // -----------------------------------------------------------------------
    logPrintf("[PHASE] starting Phase 3: load sweep\n");
    runPhase3(scale, cal, bestCombo, bestSigmaMg);

    // -----------------------------------------------------------------------
    // Фаза 4: восстановление и DONE
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
              standName, (unsigned long)(total_ms / 1000));
}
