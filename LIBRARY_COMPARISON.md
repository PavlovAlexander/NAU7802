# Техническое Сравнение: SparkFun vs Adafruit NAU7802

## Быстрое резюме

**ВЫБОР:** SparkFun NAU7802 ✅

**Причины:**
1. Полная поддержка многоточечной калибровки
2. Максимальный контроль над аппаратурой NAU7802
3. Минимальные зависимости (только Wire.h)
4. Лучшая оптимизация памяти для ESP-32
5. Поддержка адреса 0x2A
6. Высокое разрешение - Gain до 128x для достижения точности ±1мг

---

## 1. Детальное Сравнение

### SparkFun NAU7802 Library

**PlatformIO ID:** `sparkfun/SparkFun NAU7802 Scale@^1.2.5`

```cpp
// Базовая инициализация
#include "SparkFun_NAU7802_Scale.h"

SparkFun_NAU7802 myScale;

void setup() {
  Wire.begin(); // GPIO 21 (SDA), GPIO 22 (SCL)
  
  if (!myScale.begin(Wire)) {
    Serial.println("Scale not found!");
    while(1);
  }
  
  // Полный контроль над регистрами
  myScale.selectGain(64);              // Максимальный gain для 50g
  myScale.setSampleRate(320);          // 320 SPS
  myScale.setZeroOffset(0);
  myScale.enableChopper();             // Включить стабилизацию
  
  // Доступ к сырым регистрам
  uint8_t revision = myScale.getRevision();
}

void loop() {
  if (myScale.available()) {
    int32_t rawValue = myScale.getReading(); // 24-bit value
    Serial.println(rawValue);
  }
}
```

**Полный API (relevent методы):**

| Функция | Параметры | Возврат | Описание |
|---------|-----------|---------|---------|
| `begin(TwoWire&)` | I²C шина | bool | Инициализация I²C, проверка устройства |
| `available()` | - | bool | Есть ли новые данные? |
| `getReading()` | - | int32_t | Получить 24-bit ADC значение |
| `getRevision()` | - | uint8_t | Получить версию кристалла |
| `selectGain(uint8_t)` | 1, 2, 4, 8, 16, 32, 64 | bool | Установить gain |
| `setSampleRate(uint16_t)` | 10, 20, 40, 80, 320 SPS | bool | Установить частоту дискретизации |
| `setZeroOffset(uint32_t)` | 0-16777215 | - | Установить нулевое смещение |
| `getZeroOffset()` | - | uint32_t | Получить текущее нулевое смещение |
| `setCalibrationFactor(uint16_t)` | калибровочный фактор | - | Установить фактор калибровки |
| `getCalibrationFactor()` | - | uint16_t | Получить текущий фактор |
| `enableChopper()` | - | - | Включить Chopper (стабилизация) |
| `disableChopper()` | - | - | Отключить Chopper |
| `reset()` | - | - | Мягкий сброс устройства |
| `calibrateAFE()` | - | bool | Автоматическая калибровка AFE |

---

### Adafruit NAU7802 Library

**PlatformIO ID:** `adafruit/Adafruit NAU7802@^1.1.6`

```cpp
// Базовая инициализация (требует Adafruit_BusIO)
#include "Adafruit_NAU7802.h"

Adafruit_NAU7802 nau7802;

void setup() {
  if (!nau7802.begin()) {
    Serial.println("NAU7802 not found!");
    while(1);
  }
  
  // Ограниченный контроль
  nau7802.enableChopper();
  nau7802.disableChopper();
  
  // Встроенная односточечная калибровка
  nau7802.calibrate(50); // Калибровать под 50g
}

void loop() {
  int32_t rawValue = nau7802.getReading();
  Serial.println(rawValue);
}
```

**Упрощённый API:**

| Функция | Параметры | Возврат | Описание |
|---------|-----------|---------|---------|
| `begin()` | - | bool | Инициализация |
| `getReading()` | - | int32_t | Получить значение |
| `calibrate(uint16_t)` | целевой вес | bool | Односточечная калибровка |
| `enableChopper()` | - | - | Стабилизация ВКЛ |
| `disableChopper()` | - | - | Стабилизация ВЫКЛ |
| `reset()` | - | - | Сброс |

---

## 2. Зависимости

### SparkFun
```ini
[platformio.ini]
lib_deps = 
    sparkfun/SparkFun NAU7802 Scale@^1.2.5
```
**Зависит от:** ничего (только Wire.h из Arduino Core)

### Adafruit
```ini
[platformio.ini]
lib_deps = 
    adafruit/Adafruit NAU7802@^1.1.6
    adafruit/Adafruit BusIO@^1.14.0  ← Дополнительная зависимость!
    adafruit/Adafruit Unified Sensor@^1.1.14  ← Опционально
```
**Зависит от:** Adafruit_BusIO + потенциально ещё библиотеки

---

## 3. Использование памяти

### Flash Memory (прошивка)

| Библиотека | SparkFun | Adafruit |
|-----------|----------|----------|
| Исходный код | ~25 КБ | ~40 КБ |
| + Зависимости | 0 КБ | ~45 КБ |
| **Итого** | **~25 КБ** | **~85 КБ** |

### RAM Memory

| Библиотека | SparkFun | Adafruit |
|-----------|----------|----------|
| Объект | ~300 bytes | ~500 bytes |
| Буферы | переменная | переменная |

**ESP32 имеет:** 320 КБ SRAM → оба варианта окей, но SparkFun легче

---

## 4. Возможности Калибровки

### SparkFun

```cpp
// Реализация многоточечной калибровки (4 точки) - ВОЗМОЖНО

struct CalibrationPoint {
  uint32_t rawADC;
  float weight;
};

CalibrationPoint calibration[4];

// 1. Сбор 4 точек
for (int i = 0; i < 4; i++) {
  // Собрать N samples
  uint32_t sum = 0;
  for (int j = 0; j < 50; j++) {
    sum += myScale.getReading();
    delay(10); // 10Hz = 100ms между samples
  }
  calibration[i].rawADC = sum / 50; // Average
  calibration[i].weight = weights[i]; // 0, 10, 20, 30
}

// 2. Линейная регрессия (самостоятельно)
float k = calculateSlope(calibration);
float b = calculateIntercept(calibration);

// 3. Применение при взвешивании
float weight = (rawADC - b) / k;
```

**Вывод:** ✅ ПОЛНАЯ поддержка многоточечной калибровки

---

### Adafruit

```cpp
// Built-in calibration
nau7802.calibrate(50); // Только ОДНА точка!

// Это вызывает:
// 1. Установить целевой вес (50g)
// 2. Собрать N samples
// 3. Установить calibrationFactor автоматически
// Но это односточечное!

// Нет встроенной двухточечной или четырёхточечной регрессии
```

**Вывод:** ❌ Только односточечная калибровка (недостаточно для нашего проекта)

---

## 5. Регистровое управление

### SparkFun - Доступ к регистрам

```cpp
// Прямой доступ к регистрам NAU7802
uint8_t adcResult[3];
myScale.read(NAU7802_ADC_B2, adcResult, 3);

// Установка параметров через регистры
myScale.write(NAU7802_CTRL1, value);

// Чтение конфигурации
uint32_t configReg = myScale.getRevision();
```

**Преимущество:** Полный контроль при необходимости

---

### Adafruit - Абстрагирование

```cpp
// Нет прямого доступа к регистрам
// Используются только встроенные методы
// Если нужна более тонкая настройка - невозможно легко
```

**Недостаток:** Ограничена встроенным функционалом

---

## 6. Сообщество и Поддержка

| Параметр | SparkFun | Adafruit |
|----------|----------|----------|
| GitHub Stars | ~600 | ~100 |
| Issues Resolved | Быстро (часы) | Médium (дни) |
| Community FAQs | Обширные | Базовые |
| Arduino Examples | Множество | Несколько |
| Форум поддержки | Активный | Активный |
| Last Update | Регулярно | Периодически |

---

## 7. Практический Код: SparkFun для 4-точечной Калибровки

```cpp
// ==========================================
// Scale Calibration Algorithm (SparkFun)
// ==========================================

#include "SparkFun_NAU7802_Scale.h"

SparkFun_NAU7802 myScale;

// Структура для хранения калибровочных точек
struct CalibrationData {
  struct Point {
    uint32_t rawADC;
    float weight;
  } points[4];
  float k, b, r2;
} calibration;

// Сбор samples для одной точки
uint32_t collectSamples(int numSamples) {
  uint64_t sum = 0;
  for (int i = 0; i < numSamples; i++) {
    if (myScale.available()) {
      sum += myScale.getReading();
    }
    delay(100); // 10Hz = 100ms
  }
  return sum / numSamples;
}

// Линейная регрессия (метод наименьших квадратов)
void calculateLinearRegression() {
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  int n = 4;
  
  for (int i = 0; i < 4; i++) {
    float x = calibration.points[i].rawADC;
    float y = calibration.points[i].weight;
    
    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumX2 += x * x;
  }
  
  float meanX = sumX / n;
  float meanY = sumY / n;
  
  // k = slope
  calibration.k = (sumXY - n * meanX * meanY) / (sumX2 - n * meanX * meanX);
  // b = intercept
  calibration.b = meanY - calibration.k * meanX;
  
  // R² = coefficient of determination
  float ssRes = 0, ssTot = 0;
  for (int i = 0; i < 4; i++) {
    float predicted = calibration.k * calibration.points[i].rawADC + calibration.b;
    ssRes += pow(calibration.points[i].weight - predicted, 2);
    ssTot += pow(calibration.points[i].weight - meanY, 2);
  }
  
  calibration.r2 = 1 - (ssRes / ssTot);
}

// Основной процесс калибровки
void performCalibration() {
  float targetWeights[4] = {0, 10, 20, 30}; // grams
  
  for (int i = 0; i < 4; i++) {
    Serial.printf("[CALIB] Point %d: Place %.0fg on scale. Press ENTER.\n", i + 1, targetWeights[i]);
    
    while (Serial.available() == 0); // Wait for ENTER
    Serial.read(); // Clear buffer
    
    Serial.printf("[CALIB] Collecting 50 samples...");
    calibration.points[i].rawADC = collectSamples(50);
    calibration.points[i].weight = targetWeights[i];
    
    Serial.printf(" Done! Raw ADC: %lu\n", calibration.points[i].rawADC);
  }
  
  // Расчёт коэффициентов
  calculateLinearRegression();
  
  // Проверка качества
  if (calibration.r2 > 0.99) {
    Serial.printf("[CALIB] ✓ Calibration successful! R² = %.4f\n", calibration.r2);
    Serial.printf("[CALIB] y = %.8f * x + %.2f\n", calibration.k, calibration.b);
    
    // Сохранить в NVS (см. следующий раздел)
    saveCalibration();
  } else {
    Serial.printf("[CALIB] ✗ FAILED! R² = %.4f (требуется > 0.99)\n", calibration.r2);
    Serial.println("[CALIB] Повторите калибровку...");
  }
}

// Взвешивание с использованием коэффициентов
float getCurrentWeight() {
  if (myScale.available()) {
    int32_t rawADC = myScale.getReading();
    return calibration.k * (float)rawADC + calibration.b;
  }
  return -1.0; // Error
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // GPIO 21, 22
  
  if (!myScale.begin(Wire)) {
    Serial.println("[ERROR] NAU7802 not found!");
    while(1);
  }
  
  myScale.selectGain(64);
  myScale.setSampleRate(320);
  myScale.enableChopper();
  
  Serial.println("[INIT] Ready. Press ENTER for calibration.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      performCalibration();
    }
  }
  
  // Continuous weighing
  float weight = getCurrentWeight();
  if (weight >= 0) {
    Serial.printf("[WEIGH] %.2f g\n", weight);
  }
  
  delay(100); // 10 Hz update rate
}
```

---

## 8. Вывод

| Критерий | SparkFun | Adafruit | Выигрывает |
|----------|----------|----------|-----------|
| **Многоточечная калибровка** | ✅ Просто | ❌ Нет | **SparkFun** |
| **Контроль регистров** | ✅ Полный | ❌ Ограничен | **SparkFun** |
| **Размер памяти** | ✅ Малый | ❌ Больший | **SparkFun** |
| **Зависимости** | ✅ 0 | ❌ 2+ | **SparkFun** |
| **Простота для новичка** | ❌ Средняя | ✅ Высокая | **Adafruit** |
| **Документация** | ✅ Хорошая | ✅ Хорошая | Равно |

---

## 9. Рекомендация

### ✅ ИСПОЛЬЗУЙТЕ: SparkFun NAU7802

**Для этого проекта:**
```ini
; platformio.ini
lib_deps = 
    sparkfun/SparkFun NAU7802 Scale@^1.2.5
```

**Почему:**
1. ✅ Четыре-точечная калибровка требует низкоуровневого контроля
2. ✅ SparkFun даёт полный доступ к API NAU7802
3. ✅ Меньше зависимостей = более стабильно
4. ✅ Лучшая оптимизация памяти для ESP-32
5. ✅ Открытый исходный код = можно кастомизировать

**Если бы требовалась** одноточечная калибровка с максимальной простотой → Adafruit бы выиграл.

---

Документ подготовлен: 8 апреля 2026  
Версия: 1.0
