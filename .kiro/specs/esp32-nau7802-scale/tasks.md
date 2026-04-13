# План реализации: ESP-32 NAU7802 Scale

## Обзор

Реализация прошивки тестера весов на C++ (Arduino/PlatformIO) по модульной архитектуре с FSM.
Каждый шаг строится на предыдущем и завершается интеграцией всех компонентов в `main.cpp`.

## Задачи

- [x] 1. Настройка структуры проекта и общих типов данных
  - Создать `platformio.ini` с конфигурацией `esp32doit-devkit-v1`, `monitor_speed = 115200`, зависимостью `sparkfun/SparkFun NAU7802 Scale@^1.2.5` и секцией `[env:native]` с `googletest` для тестов на хосте
  - Создать `include/calibration_data.h` со структурами `CalibrationPoint` (int32_t rawADC, float weight) и `CalibrationData` (points[4], k, b, r2, timestamp)
  - Создать заглушки заголовочных файлов: `scale_init.h`, `scale_calibration.h`, `scale_weighing.h`, `calibration_storage.h`, `math_utils.h`, `ui_console.h`
  - _Требования: 8.1, 8.2, 8.3, 8.5_

- [x] 2. Реализация модуля math_utils
  - [x] 2.1 Реализовать `math_utils.cpp` / `math_utils.h`
    - Функция `linearRegression(const CalibrationPoint* points, int n, float& k, float& b)` — МНК по формулам из дизайна
    - Функция `computeR2(const CalibrationPoint* points, int n, float k, float b)` — вычисление R²
    - _Требования: 3.4, 3.5_

  - [ ]* 2.2 Написать property-тест `test_linear_regression_roundtrip` (Property 2)
    - Генерировать случайные k, b; строить 4 точки на прямой; проверять |k'−k| < 1e-3 и |b'−b| < 1e-3
    - **Property 2: Round-trip линейной регрессии**
    - **Validates: Requirements 3.4**

  - [ ]* 2.3 Написать property-тест `test_r2_range` (Property 3)
    - Для точек на прямой: |R²−1.0| < 1e-5; для произвольных точек: R² ∈ [0.0, 1.0]
    - **Property 3: R² для точек на прямой равен 1.0, для произвольных — в [0, 1]**
    - **Validates: Requirements 3.5**

- [x] 3. Реализация модуля ui_console
  - [x] 3.1 Реализовать `ui_console.cpp` / `ui_console.h`
    - `printTagged(const char* tag, const char* msg)` — вывод с тегом
    - `printWeighLine(int32_t rawADC, float calibrated, float filtered)` — формат `[WEIGH] Raw: <rawADC> | Calibrated: <X.XXX>g | Filtered: <X.XXX>g`
    - `printStatus(const CalibrationData& cal)` — вывод `[STAT]` с адресом 0x2A, Gain 128, k, b, R², timestamp
    - `printError(const char* msg)` — вывод с тегом `[ERROR]`
    - _Требования: 5.4, 6.3, 7.1_

  - [ ]* 3.2 Написать property-тест `test_weigh_line_format` (Property 8)
    - Для любых rawADC, calibrated, filtered: строка содержит `[WEIGH]`, `Raw:`, `Calibrated:` + `g`, `Filtered:` + `g`
    - **Property 8: Формат строки взвешивания содержит все обязательные поля**
    - **Validates: Requirements 5.4**

  - [ ]* 3.3 Написать property-тест `test_status_format` (Property 9)
    - Для любых валидных CalibrationData: строка содержит `[STAT]`, `0x2A`, `128`, k, b, R², timestamp
    - **Property 9: Вывод статуса содержит все обязательные поля**
    - **Validates: Requirements 6.3**

- [x] 4. Реализация модуля scale_weighing (SMA-фильтр и вычисление веса)
  - [x] 4.1 Реализовать `scale_weighing.cpp` / `scale_weighing.h`
    - `computeWeight(float k, float b, int32_t rawADC) → float` — линейная модель
    - Класс `SMA_Filter`: кольцевой буфер 10 значений, методы `add(float)` и `getAverage() const`
    - `weighingTick(SparkFun_NAU7802& scale, const CalibrationData& cal, SMA_Filter& filter)` — один цикл: читает rawADC, вычисляет calibrated и filtered, выводит через `printWeighLine`
    - _Требования: 5.1, 5.2, 5.3, 5.4_

  - [ ]* 4.2 Написать property-тест `test_compute_weight` (Property 6)
    - Для любых k, b (float) и rawADC (int32_t): результат равен `k * (float)rawADC + b` с float-точностью
    - **Property 6: Корректность вычисления веса по линейной модели**
    - **Validates: Requirements 5.2**

  - [ ]* 4.3 Написать property-тест `test_sma_filter_average` (Property 7)
    - Для любой последовательности float-значений: `getAverage()` возвращает среднее последних min(n, 10) значений
    - **Property 7: SMA-фильтр возвращает среднее последних 10 значений**
    - **Validates: Requirements 5.3**

- [x] 5. Контрольная точка — убедиться, что все тесты проходят
  - Убедиться, что все тесты проходят, задать вопросы пользователю при необходимости.

- [x] 6. Реализация модуля calibration_storage (NVS)
  - [x] 6.1 Реализовать `calibration_storage.cpp` / `calibration_storage.h`
    - `saveCalibration(const CalibrationData& cal)` — сохранение k, b, r2, timestamp, raw_0..raw_3, weight_0..weight_3 в NVS namespace `scale_cal` через `Preferences.h`
    - `loadCalibration(CalibrationData& cal) → bool` — загрузка; возвращает false если k == 0.0
    - _Требования: 4.1, 4.2, 4.3, 4.4_

  - [ ]* 6.2 Написать property-тест `test_calibration_nvs_roundtrip` (Property 4)
    - Для любых валидных CalibrationData (k ≠ 0.0, r2 ≥ 0.99): save → load возвращает те же k, b, r2, timestamp и все 4 точки
    - **Property 4: Round-trip сериализации CalibrationData в NVS**
    - **Validates: Requirements 4.1, 4.2**

  - [ ]* 6.3 Написать property-тест `test_calibration_nvs_overwrite` (Property 5)
    - Для двух последовательных CalibrationData A и B: save(A) → save(B) → load возвращает B
    - **Property 5: Идемпотентность перезаписи калибровки в NVS**
    - **Validates: Requirements 4.4**

- [x] 7. Реализация модуля scale_calibration (мастер калибровки)
  - [x] 7.1 Реализовать `scale_calibration.cpp` / `scale_calibration.h`
    - `collectSamples(SparkFun_NAU7802& scale, int numSamples) → int32_t` — сбор numSamples отсчётов с интервалом 100 мс, возврат среднего
    - `runCalibrationWizard(SparkFun_NAU7802& scale, CalibrationData& cal) → bool` — 4-шаговый мастер: запрос ENTER → сбор 50 отсчётов → сохранение CalibrationPoint; после 4 точек вызов `linearRegression` и `computeR2`; если R² ≥ 0.99 — `saveCalibration` и return true; иначе вывод ошибки и return false
    - _Требования: 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7_

  - [ ]* 7.2 Написать property-тест `test_collect_samples_average` (Property 1)
    - Для любого массива из 50 int-значений: функция возвращает их среднее арифметическое (целочисленное деление)
    - **Property 1: Корректность вычисления среднего при сборе отсчётов**
    - **Validates: Requirements 3.2**

- [x] 8. Реализация модуля scale_init
  - Реализовать `scale_init.cpp` / `scale_init.h`
  - `initHardware(SparkFun_NAU7802& scale) → bool`: `Wire.begin(21, 22)`, `Wire.setClock(400000)`, `scale.begin(Wire)`, `scale.setGain(NAU7802_GAIN_128)`, `scale.setSampleRate(NAU7802_SPS_320)`, `scale.enableChopper()`; при ошибке — `printError` и return false
  - _Требования: 1.1, 1.2, 1.3, 1.4_

- [x] 9. Реализация FSM в main.cpp и интеграция всех модулей
  - [x] 9.1 Реализовать `main.cpp` с перечислением `AppState`, глобальными экземплярами `myScale`, `calibration`, `smaFilter`
    - `setup()`: инициализация Serial, вызов `initHardware`; при успехе → STATE_READY; при ошибке → STATE_ERROR
    - `loop()`: чтение команды из Serial → `handleCommand(cmd)`; диспетчер `switch(currentState)` — STATE_READY (попытка `loadCalibration`: успех → STATE_WEIGHING, иначе ждать ENTER), STATE_CALIBRATING (`runCalibrationWizard` → STATE_WEIGHING или остаться), STATE_WEIGHING (`weighingTick` каждые 100 мс), STATE_ERROR (ждать 5000 мс)
    - `handleCommand(char cmd)`: `'c'` → STATE_CALIBRATING, `'r'` → `ESP.restart()`, `'s'` → `printStatus`, ENTER в STATE_READY → STATE_CALIBRATING, ENTER в STATE_CALIBRATING → подтверждение точки
    - _Требования: 1.5, 2.1, 2.2, 2.3, 2.4, 6.1, 6.2, 6.3, 6.4, 7.2, 7.3_

  - [ ]* 9.2 Написать property-тест `test_restart_any_state` (Property 10)
    - Для любого состояния FSM из {STATE_INIT, STATE_READY, STATE_CALIBRATING, STATE_WEIGHING, STATE_ERROR}: `handleCommand('r')` вызывает `ESP.restart()`
    - **Property 10: Команда 'r' вызывает перезагрузку в любом состоянии FSM**
    - **Validates: Requirements 6.2, 7.3**

  - [ ]* 9.3 Написать property-тест `test_auto_weighing_on_valid_cal` (Property 11)
    - Для любых валидных CalibrationData (k ≠ 0.0): после `loadCalibration` в STATE_READY FSM переходит в STATE_WEIGHING без ожидания команды
    - **Property 11: Автопереход в STATE_WEIGHING при наличии валидной калибровки**
    - **Validates: Requirements 2.2**

  - [ ]* 9.4 Написать unit-тесты FSM в `test/test_fsm.cpp`
    - NAU7802 найден → STATE_READY; не найден → STATE_ERROR
    - k=0.0 в NVS → не переходить в STATE_WEIGHING
    - R²≥0.99 → STATE_WEIGHING; R²<0.99 → остаться в STATE_CALIBRATING
    - ENTER в STATE_READY → STATE_CALIBRATING; 'c' в STATE_WEIGHING → STATE_CALIBRATING
    - STATE_ERROR: задержка 5000 мс перед следующим действием
    - _Требования: 1.4, 1.5, 2.2, 2.3, 2.4, 3.6, 3.7, 6.1, 7.2_

- [x] 10. Финальная контрольная точка — убедиться, что все тесты проходят
  - Убедиться, что все тесты проходят, задать вопросы пользователю при необходимости.

## Примечания

- Задачи, помеченные `*`, являются необязательными и могут быть пропущены для ускорения MVP
- Каждая задача ссылается на конкретные требования для обеспечения трассируемости
- Property-тесты запускаются в `[env:native]` (GoogleTest + rapidcheck) без реального железа
- Unit-тесты FSM используют моки для `SparkFun_NAU7802` и `Preferences`
- Интеграционное тестирование на реальном железе выполняется вручную после сборки прошивки
