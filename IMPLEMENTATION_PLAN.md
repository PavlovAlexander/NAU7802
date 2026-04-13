# ПЛАН РЕАЛИЗАЦИИ: ESP-32 NAU7802 Scale Tester

**Статус:** Фаза планирования ✅ завершена  
**Следующий шаг:** Фаза 1 - Setup (PlatformIO + Hardware)

---

## Краткий Обзор Проекта

```
ЦЕЛЬ:  Создать тестер весов (ESP32 + NAU7802 на 50g)
       с калибровкой на 4 точках (0, 10, 20, 30g)
       с целевой точностью ±1 миллиграмм (±0.001g)
       
ВЫБОД: SparkFun NAU7802 Scale Library
LINGUA: C++ (Arduino framework)
I2C ADDR: 0x2A
GAIN: 128x (для достижения 1мг разрешения)
TARGET: 10 Hz непрерывное взвешивание с точностью ±1мг
```

---

## Документация

📋 **PRD.md** - Полные требования проекта (10 разделов)
📋 **LIBRARY_COMPARISON.md** - Сравнение SparkFun vs Adafruit

---

## Фазы Разработки

### ✅ ФАЗА 0: ПЛАНИРОВАНИЕ (ЗАВЕРШЕНО)
- [x] Анализ требований
- [x] Сравнение библиотек
- [x] Выбор оптимального решения (SparkFun ✅)
- [x] Архитектурное проектирование

**Артефакты:** этот файл + PRD.md + LIBRARY_COMPARISON.md

---

### 📋 ФАЗА 1: SETUP PlatformIO (1-2 дня)

#### Шаг 1.1: Инициализировать проект PlatformIO

```bash
# Создать новый проект для ESP-32 в VS Code
# File → Open in VS Code → этот каталог
# Затем: Commands → PlatformIO: New Project
# Board: Espressif ESP32 Dev Module
# Framework: Arduino
```

#### Шаг 1.2: Файл platformio.ini

```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
lib_deps = 
    sparkfun/SparkFun NAU7802 Scale@^1.2.5

# Optional: для отладки
build_flags = -DCORE_DEBUG_LEVEL=5
```

#### Шаг 1.3: Проверить связь с ESP-32

```cpp
// src/main.cpp - minimal test
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[TEST] ESP32 Ready!");
  Serial.printf("CPU Freq: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
}

void loop() {
  delay(1000);
}
```

**Проверка:** 
- Upload прошивку
- Monitor должно показать "ESP32 Ready!"

---

### 📋 ФАЗА 2: ИНИЦИАЛИЗАЦИЯ NAU7802 (2-3 дня)

#### Шаг 2.1: I2C Scan

```cpp
// Найти NAU7802 по адресу 0x2A
#include <Wire.h>

void scanI2C() {
  Serial.println("I2C Scan...");
  for (uint8_t addr = 0; addr < 128; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Device found at 0x%02X\n", addr);
    }
  }
}
```

#### Шаг 2.2: Базовый тест NAU7802

```cpp
#include "SparkFun_NAU7802_Scale.h"

SparkFun_NAU7802 myScale;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // GPIO 21 (SDA), GPIO 22 (SCL)
  
  if (!myScale.begin(Wire)) {
    Serial.println("[NAU7802] FAILED to initialize");
    while(1);
  }
  
  Serial.println("[NAU7802] ✓ Initialized at 0x2A");
  Serial.printf("Revision: 0x%02X\n", myScale.getRevision());
  
  // Configure for 1mg resolution on 50g load cell
  myScale.selectGain(128);    // Maximum gain for 1mg resolution
  myScale.setSampleRate(320); // 320 SPS for high resolution
  myScale.enableChopper();    // Noise rejection
  
  Serial.println("[NAU7802] Ready!");
}

void loop() {
  if (myScale.available()) {
    int32_t raw = myScale.getReading();
    Serial.printf("Raw ADC: %ld\n", raw);
  }
  delay(100); // 10 Hz
}
```

**Проверка:**
- Должны выводиться сырые ADC значения
- При добавлении веса значения должны возрастать

---

### 📋 ФАЗА 3: АЛГОРИТМ КАЛИБРОВКИ (3-4 дня)

#### Шаг 3.1: Структуры данных

```cpp
// include/calibration_data.h
#pragma once

struct CalibrationPoint {
  uint32_t rawADC;
  float weight;
};

struct CalibrationData {
  CalibrationPoint points[4];  // 0, 10, 20, 30 grams
  float k;                       // Slope
  float b;                       // Intercept
  float r2;                      // R-squared (quality)
  uint32_t timestamp;            // When calibrated
};
```

#### Шаг 3.2: Сбор samples

```cpp
// src/scale_calibration.cpp
uint32_t collectSamples(SparkFun_NAU7802& scale, int numSamples) {
  uint64_t sum = 0;
  int collected = 0;
  
  for (int i = 0; i < numSamples; i++) {
    if (scale.available()) {
      sum += scale.getReading();
      collected++;
    }
    delay(100); // 10 Hz = 100ms interval
  }
  
  if (collected == 0) return 0;
  return sum / collected;
}
```

#### Шаг 3.3: Линейная регрессия

```cpp
// src/math_utils.cpp
void performLinearRegression(CalibrationData& cal) {
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  int n = 4;
  
  for (int i = 0; i < 4; i++) {
    float x = cal.points[i].rawADC;
    float y = cal.points[i].weight;
    
    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumX2 += x * x;
  }
  
  float meanX = sumX / n;
  float meanY = sumY / n;
  
  // y = kx + b
  cal.k = (sumXY - n * meanX * meanY) / (sumX2 - n * meanX * meanX);
  cal.b = meanY - cal.k * meanX;
  
  // Calculate R²
  float ssRes = 0, ssTot = 0;
  for (int i = 0; i < 4; i++) {
    float predicted = cal.k * cal.points[i].rawADC + cal.b;
    ssRes += pow(cal.points[i].weight - predicted, 2);
    ssTot += pow(cal.points[i].weight - meanY, 2);
  }
  
  cal.r2 = (ssTot == 0) ? 1.0 : (1.0 - ssRes / ssTot);
}
```

#### Шаг 3.4: Интерактивная калибровка

```cpp
// src/calibration_ui.cpp
void runCalibrationWizard(SparkFun_NAU7802& scale, CalibrationData& cal) {
  float targetWeights[4] = {0, 10, 20, 30};
  
  Serial.println("\n[CALIB] === Calibration Wizard ===");
  
  for (int i = 0; i < 4; i++) {
    Serial.printf("[CALIB] STEP %d/4: Place %.0fg on scale.\n", i + 1, targetWeights[i]);
    Serial.println("[CALIB] Press ENTER when ready.");
    
    while (Serial.available() == 0);
    Serial.read();
    
    Serial.println("[CALIB] Collecting 50 samples (5 seconds)...");
    cal.points[i].rawADC = collectSamples(scale, 50);
    cal.points[i].weight = targetWeights[i];
    
    Serial.printf("[CALIB] ✓ Point %d captured: %lu ADC\n", i + 1, cal.points[i].rawADC);
  }
  
  // Calculate coefficients
  performLinearRegression(cal);
  
  // Validate
  if (cal.r2 > 0.99) {
    Serial.printf("[CALIB] ✓ PASS! R² = %.6f\n", cal.r2);
    Serial.printf("[CALIB] Equation: weight = %.8f * ADC + %.4f\n", cal.k, cal.b);
    saveCalibration(cal);  // NVS save
  } else {
    Serial.printf("[CALIB] ✗ FAIL! R² = %.6f (need > 0.99)\n", cal.r2);
    Serial.println("[CALIB] Retry calibration.");
  }
}
```

---

### 📋 ФАЗА 4: NVS СОХРАНЕНИЕ (1 день)

#### Шаг 4.1: Сохранение в ESP32 Flash

```cpp
// src/calibration_storage.cpp
#include <Preferences.h>

Preferences prefs;

void saveCalibration(const CalibrationData& cal) {
  prefs.begin("scale_cal", false); // false = read/write
  
  prefs.putFloat("k", cal.k);
  prefs.putFloat("b", cal.b);
  prefs.putFloat("r2", cal.r2);
  prefs.putULong("timestamp", cal.timestamp);
  
  // Store all points
  for (int i = 0; i < 4; i++) {
    char keyRaw[16], keyWeight[16];
    sprintf(keyRaw, "raw_%d", i);
    sprintf(keyWeight, "weight_%d", i);
    
    prefs.putULong(keyRaw, cal.points[i].rawADC);
    prefs.putFloat(keyWeight, cal.points[i].weight);
  }
  
  prefs.end();
  Serial.println("[STORAGE] Calibration saved!");
}

bool loadCalibration(CalibrationData& cal) {
  prefs.begin("scale_cal", true); // true = read-only
  
  cal.k = prefs.getFloat("k", 0.0);
  cal.b = prefs.getFloat("b", 0.0);
  cal.r2 = prefs.getFloat("r2", 0.0);
  
  if (cal.k == 0.0) {
    prefs.end();
    return false; // No calibration saved
  }
  
  for (int i = 0; i < 4; i++) {
    char keyRaw[16], keyWeight[16];
    sprintf(keyRaw, "raw_%d", i);
    sprintf(keyWeight, "weight_%d", i);
    
    cal.points[i].rawADC = prefs.getULong(keyRaw, 0);
    cal.points[i].weight = prefs.getFloat(keyWeight, 0.0);
  }
  
  prefs.end();
  Serial.println("[STORAGE] Calibration loaded!");
  return true;
}
```

---

### 📋 ФАЗА 5: РЕЖИМ ВЗВЕШИВАНИЯ (2 дня)

#### Шаг 5.1: Простое взвешивание

```cpp
// src/scale_weighing.cpp
float getCurrentWeight(const CalibrationData& cal, int32_t rawADC) {
  return cal.k * (float)rawADC + cal.b;
}
```

#### Шаг 5.2: Фильтрация (SMA-10 для достижения ±1мг точности)

```cpp
class SimpleMovingAverage {
  static const int BUFFER_SIZE = 10;  // 10-point moving average for 1mg stability
  float buffer[BUFFER_SIZE];
  int index = 0;
  
public:
  void add(float value) {
    buffer[index] = value;
    index = (index + 1) % BUFFER_SIZE;
  }
  
  float getAverage() {
    float sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      sum += buffer[i];
    }
    return sum / BUFFER_SIZE;
  }
};

SimpleMovingAverage weightFilter;

void continuousWeighing(SparkFun_NAU7802& scale, const CalibrationData& cal) {
  Serial.println("[WEIGH] Started continuous weighing at 10 Hz with ±1mg precision");
  
  while (true) {
    if (scale.available()) {
      int32_t rawADC = scale.getReading();
      float weight = getCurrentWeight(cal, rawADC);  // in grams
      weightFilter.add(weight);
      
      float smoothedWeight = weightFilter.getAverage();  // SMA-10 filtered
      
      // Display with 3 decimal places (1mg resolution)
      Serial.printf("[WEIGH] Raw: %ld | Calibrated: %.3fg | Filtered: %.3fg\n",
                    rawADC, weight, smoothedWeight);
    }
    
    delay(100); // 10 Hz = 100ms interval
  }
}
```

---

### 📋 ФАЗА 6: ИНТЕГРАЦИЯ & FSM (2-3 дня)

#### Шаг 6.1: State Machine

```cpp
// src/main.cpp
enum State {
  STATE_INIT,
  STATE_READY,
  STATE_CALIBRATING,
  STATE_WEIGHING,
  STATE_ERROR
};

State currentState = STATE_INIT;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (myScale.begin(Wire)) {
    myScale.selectGain(64);
    myScale.setSampleRate(320);
    myScale.enableChopper();
    
    if (loadCalibration(calibration)) {
      currentState = STATE_READY;
      Serial.println("[STATE] → READY (calibration loaded)");
    } else {
      currentState = STATE_READY;
      Serial.println("[STATE] → READY (calibration needed)");
    }
  } else {
    currentState = STATE_ERROR;
    Serial.println("[STATE] → ERROR");
  }
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Run state machine
  switch (currentState) {
    case STATE_READY:
      // Wait for calibration command
      break;
    case STATE_CALIBRATING:
      runCalibrationWizard(myScale, calibration);
      currentState = STATE_WEIGHING;
      break;
    case STATE_WEIGHING:
      continuousWeighing(myScale, calibration);
      break;
    case STATE_ERROR:
      delay(5000);
      break;
  }
}

void handleCommand(char cmd) {
  switch (cmd) {
    case '\n':
    case '\r':
      if (currentState == STATE_READY) {
        currentState = STATE_CALIBRATING;
      }
      break;
    case 'c':
      currentState = STATE_CALIBRATING;
      break;
    case 's':
      printStatus();
      break;
    case 'r':
      ESP.restart();
      break;
  }
}
```

---

### 📋 ФАЗА 7: ТЕСТИРОВАНИЕ (3 дня)

#### Шаг 7.1: Тестовый план

```
[ ] 1. Hardware connectivity
  [ ] NAU7802 доступна на 0x00
  [ ] Chopper работает
  [ ] 10 Hz update rate

[ ] 2. Calibration accuracy
  [ ] 4 точки собираются правильно
  [ ] R² > 0.99 достигается
  [ ] NVS сохранение работает

[ ] 3. Weighing accuracy
  [ ] 0g = ±0.1g
  [ ] 50g = ±1g (±2%)
  [ ] Стабильность ±0.2g на одной точке

[ ] 4. UI/Commands
  [ ] ENTER запускает калибровку
  [ ] 'c' повторно калибрует
  [ ] 's' показывает статус
  [ ] 'r' перезагружает
```

---

### 📋 ФАЗА 8: ДОКУМЕНТАЦИЯ (1 день)

- [ ] README.md с инструкциями
- [ ] Диаграммы подключения (Fritzing)
- [ ] Примеры использования
- [ ] Troubleshooting guide

---

## Структура Файлов Проекта

```
NAU7802_WeightTester/
├── platformio.ini
├── docs/
│   ├── PRD.md (✅ завершено)
│   ├── LIBRARY_COMPARISON.md (✅ завершено)
│   ├── IMPLEMENTATION_PLAN.md (этот файл)  
│   ├── DIAGRAMS/
│   │   ├── hardware_schematic.png
│   │   └── fsm_diagram.png
│   └── EXAMPLES/
│       ├── basic_init.cpp
│       └── calibration_example.cpp
├── src/
│   ├── main.cpp
│   ├── scale_init.cpp
│   ├── scale_calibration.cpp
│   ├── scale_weighing.cpp
│   ├── calibration_storage.cpp
│   ├── math_utils.cpp
│   ├── ui_console.cpp
│   └── state_machine.cpp
├── include/
│   ├── scale_init.h
│   ├── scale_calibration.h
│   ├── scale_weighing.h
│   ├── calibration_data.h
│   ├── math_utils.h
│   └── state_machine.h
├── lib/
│   └── (PlatformIO скачает SparkFun NAU7802)
└── README.md
```

---

## Чеклист Запуска

### Перед Фазой 1
- [ ] ESP-32 подключена к USB
- [ ] VS Code + PlatformIO установлены
- [ ] NAU7802 подключена к GPIO 21/22 (I2C)
- [ ] Тензодатчик подключен к NAU7802

### Перед Фазой 2
- [ ] platformio.ini настроен
- [ ] Проект компилируется
- [ ] Upload працует, monitor видит вывод

### Перед Фазой 3
- [ ] NAU7802 инициализируется (revision читается)
- [ ] ADC значения выводятся (меняются при добавлении веса)

### Перед Фазой 4
- [ ] Калибровка на 4 точках работает
- [ ] R² вычисляется и выводится

### Перед Фазой 5
- [ ] Калибровка сохраняется в NVS
- [ ] Калибровка загружается при перезагрузке

### Перед Фазой 6
- [ ] Взвешивание 10 Hz работает
- [ ] SMA-5 фильтр стабилизирует показания

### Перед Фазой 7
- [ ] Все нормально компилируется
- [ ] FSM работает (INIT → READY → WEIGHING)
- [ ] Команды работают

---

## Рекомендации

✅ **ИСПОЛЬЗУЙТЕ:** SparkFun NAU7802 Scale (не Adafruit!)

✅ **ВАЖНО:** Разработку вести поэтапно (фаза за фазой)

✅ **ТЕСТИРОВАНИЕ:** После каждой фазы проверять на железе

✅ **КАЛИБРОВКА:** Использовать точные гирьки (10, 20, 30g)

✅ **УСЛОВИЯ:** Калибровать в спокойной обстановке (без вибраций)

---

## Следующий Шаг

**Начните с Фазы 1:**
1. Создайте `platformio.ini`
2. Инициализируйте новый PlatformIO проект
3. Загрузите SparkFun NAU7802 библиотеку
4. Компилируйте и тестируйте connectivity

**После Фазы 1:** Обновите этот план с результатами.

---

**Документ подготовлен:** 8 апреля 2026  
**Версия:** 1.0  
**Статус:** Готов к разработке ✅
