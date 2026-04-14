# Test Wizard — Руководство по интерактивному тестированию

## Обзор

Test Wizard (Мастер тестирования) — это интерактивный инструмент для пошагового тестирования всех улучшений точности весов. Мастер проводит пользователя через 6 тестов, автоматически собирает данные и генерирует подробный отчёт.

## Зачем нужен Test Wizard?

- ✅ **Автоматизация**: Не нужно вручную записывать результаты
- ✅ **Пошаговость**: Чёткие инструкции на каждом шаге
- ✅ **Полнота**: Все тесты в одном месте
- ✅ **Отчётность**: Красивый отчёт с PASS/FAIL индикаторами
- ✅ **Воспроизводимость**: Можно повторить тесты в любой момент

## Команды

```
test wizard start    - Запустить мастер с начала
test wizard resume   - Продолжить с последнего теста
test wizard report   - Показать текущий отчёт
test wizard reset    - Сбросить все результаты
test wizard skip     - Пропустить текущий тест
test wizard help     - Показать справку
```

## Структура тестов

### Test 1: Calibration Methods (15 минут)
**Цель**: Проверить все 4 метода калибровки и сравнить их R²

**Шаги**:
1. Мастер попросит положить 0g, 10g, 20g, 30g (4 раза для каждого метода)
2. Автоматически вычислит R² для каждого метода
3. Сохранит результаты

**Критерий успеха**: R² ≥ 0.99 для всех методов

**Пример вывода**:
```
╔════════════════════════════════════════════╗
║  TEST 1/6: Calibration Methods            ║
╚════════════════════════════════════════════╝

[1.1] Testing Linear Calibration
[INSTRUCTION] Place 0g weight and press ENTER
[WAITING] ...
[OK] Point 0 collected: rawADC = 307947

[INSTRUCTION] Place 10g weight and press ENTER
...

[RESULT] Linear Calibration: R² = 0.9998 ✓ PASS

[1.2] Testing Piecewise Linear Calibration
...
```

---

### Test 2: Stability Test (30 минут)
**Цель**: Измерить стабильность и точность для 4 весов

**Шаги**:
1. Для каждого веса (0g, 10g, 20g, 30g):
   - Положить гирьку
   - Мастер соберёт 100 измерений (автоматически)
   - Вычислит min, max, mean, σ
2. Сохранит результаты

**Критерий успеха**: σ < 2 мг для всех весов

**Пример вывода**:
```
╔════════════════════════════════════════════╗
║  TEST 2/6: Stability Test                 ║
╚════════════════════════════════════════════╝

[2.1] Testing stability at 0g
[INSTRUCTION] Place 0g weight (empty platform) and press ENTER
[WAITING] ...
[OK] Starting 100 measurements...

Progress: [████████████████████] 100/100

[RESULT] Stability at 0g:
  Min:    -0.003g
  Max:     0.002g
  Mean:   -0.001g
  StdDev:  0.0018g (±1.8mg) ✓ PASS

[2.2] Testing stability at 10g
...
```

---

### Test 3: Outlier Filter Test (10 минут)
**Цель**: Проверить эффективность фильтра выбросов

**Шаги**:
1. **Baseline** (без помех):
   - Положить 30g
   - Мастер соберёт 50 измерений
   - Подсчитает выбросы
2. **With interference** (с помехами):
   - Положить 30g
   - Мастер попросит махать рукой рядом с весами
   - Соберёт 50 измерений
   - Подсчитает отфильтрованные выбросы

**Критерий успеха**: ≥95% выбросов отфильтровано

**Пример вывода**:
```
╔════════════════════════════════════════════╗
║  TEST 3/6: Outlier Filter Test            ║
╚════════════════════════════════════════════╝

[3.1] Baseline (no interference)
[INSTRUCTION] Place 30g weight and press ENTER
[WAITING] ...
[OK] Collecting 50 measurements...

Progress: [████████████████████] 50/50

[RESULT] Baseline:
  Outliers detected: 2 (4%)

[3.2] With interference
[INSTRUCTION] Place 30g weight
[INSTRUCTION] Wave your hand near the scale during measurement
[INSTRUCTION] Press ENTER to start
[WAITING] ...
[OK] Collecting 50 measurements...

Progress: [████████████████████] 50/50

[RESULT] With interference:
  Outliers detected:  15 (30%)
  Outliers filtered:  15 (100%) ✓ PASS
```

---

### Test 4: Adaptive Averaging Test (5 минут)
**Цель**: Проверить переключение между режимами точности и скорости

**Шаги**:
1. **Stable weight**:
   - Положить 30g
   - Подождать 10 секунд
   - Мастер проверит количество отсчётов (должно быть maxSamples)
2. **Changing weight**:
   - Убрать гирьку
   - Мастер проверит время переключения на minSamples

**Критерий успеха**: Переключение за ≤2 измерения

**Пример вывода**:
```
╔════════════════════════════════════════════╗
║  TEST 4/6: Adaptive Averaging Test        ║
╚════════════════════════════════════════════╝

[4.1] Testing stable weight mode
[INSTRUCTION] Place 30g weight and press ENTER
[WAITING] ...
[OK] Waiting 10 seconds for stabilization...

Progress: [████████████████████] 10/10 seconds

[RESULT] Stable weight:
  Current samples: 50 (maxSamples) ✓ PASS
  StdDev: 0.0015g (stable)

[4.2] Testing changing weight mode
[INSTRUCTION] Remove the weight NOW
[OK] Monitoring transition...

[RESULT] Changing weight:
  Transition time: 2 measurements ✓ PASS
  Final samples: 10 (minSamples)
```

---

### Test 5: Calibration Method Comparison (5 минут)
**Цель**: Сравнить все методы калибровки на промежуточных весах

**Шаги**:
1. Для каждого веса (5g, 15g, 25g):
   - Положить гирьку
   - Мастер измерит вес всеми 4 методами
   - Покажет разницу между методами

**Критерий успеха**: Разница между методами < 10 мг

**Пример вывода**:
```
╔════════════════════════════════════════════╗
║  TEST 5/6: Calibration Method Comparison  ║
╚════════════════════════════════════════════╝

[5.1] Testing at 5g (between 0g and 10g)
[INSTRUCTION] Place 5g weight and press ENTER
[WAITING] ...
[OK] Measuring with all methods...

[RESULT] Weight at 5g:
  Linear:           5.002g
  Piecewise Linear: 5.001g
  Polynomial-2:     5.003g
  Polynomial-3:     5.002g
  Max difference:   0.002g (2mg) ✓ PASS

[5.2] Testing at 15g (between 10g and 20g)
...
```

---

### Test 6: Temperature Compensation (1-2 часа, опционально)
**Цель**: Проверить эффективность температурной компенсации

**Шаги**:
1. **Baseline**:
   - Измерить температуру и вес при комнатной температуре
2. **After temperature change**:
   - Изменить температуру (фен/холод)
   - Подождать 10 минут
   - Измерить температуру и вес
   - Сравнить дрейф с/без компенсации

**Критерий успеха**: Дрейф с компенсацией < 1 мг при ΔT=10°C

**Пример вывода**:
```
╔════════════════════════════════════════════╗
║  TEST 6/6: Temperature Compensation       ║
╚════════════════════════════════════════════╝

[6.1] Baseline measurement
[INSTRUCTION] Place 30g weight and press ENTER
[WAITING] ...
[OK] Measuring baseline...

[RESULT] Baseline:
  Temperature: 24.5°C
  Weight:      30.000g

[6.2] After temperature change
[INSTRUCTION] Change temperature (use hair dryer or move to cold place)
[INSTRUCTION] Wait 10 minutes
[INSTRUCTION] Press ENTER when ready
[WAITING] ...
[OK] Measuring after temperature change...

[RESULT] After temperature change:
  Temperature: 34.2°C (ΔT = +9.7°C)
  Weight without compensation: 30.008g (drift: +8mg)
  Weight with compensation:    30.001g (drift: +1mg) ✓ PASS
```

---

## Финальный отчёт

После завершения всех тестов команда `test wizard report` выведет:

```
╔════════════════════════════════════════════════════════════════╗
║                    TEST SUMMARY REPORT                        ║
║                  Scale Accuracy Improvements                  ║
╚════════════════════════════════════════════════════════════════╝

Test 1: Calibration Methods
  ✓ Linear:           R²=0.9998  PASS
  ✓ Piecewise Linear: R²=0.9999  PASS
  ✓ Polynomial-2:     R²=0.9997  PASS
  ✓ Polynomial-3:     R²=0.9998  PASS

Test 2: Stability Test
  ✓ 0g:   σ=0.0018g (±1.8mg)  PASS
  ✓ 10g:  σ=0.0015g (±1.5mg)  PASS
  ✓ 20g:  σ=0.0019g (±1.9mg)  PASS
  ✓ 30g:  σ=0.0020g (±2.0mg)  PASS

Test 3: Outlier Filter
  ✓ Baseline outliers:     2 (4%)
  ✓ Interference outliers: 15 (30%)
  ✓ Filtered:              15 (100%)  PASS

Test 4: Adaptive Averaging
  ✓ Stable mode:      50 samples  PASS
  ✓ Transition time:  2 measurements  PASS
  ✓ Changing mode:    10 samples  PASS

Test 5: Calibration Comparison
  ✓ At 5g:  max diff = 2mg  PASS
  ✓ At 15g: max diff = 3mg  PASS
  ✓ At 25g: max diff = 2mg  PASS

Test 6: Temperature Compensation
  ✓ Baseline:  T=24.5°C, W=30.000g
  ✓ Changed:   T=34.2°C (ΔT=+9.7°C)
  ✓ Drift without comp: +8mg
  ✓ Drift with comp:    +1mg  PASS

╔════════════════════════════════════════════════════════════════╗
║  OVERALL RESULT: 6/6 tests PASSED ✓                          ║
║  Total test time: 1h 15m                                      ║
╚════════════════════════════════════════════════════════════════╝

[INFO] Report saved to test_results.csv
[INFO] Use 'test wizard reset' to clear results and start over
```

---

## Экспорт результатов

Команда `test wizard export` создаст CSV файл:

```csv
test_name,metric,value,unit,status
calibration_linear,r2,0.9998,,PASS
calibration_piecewise,r2,0.9999,,PASS
calibration_poly2,r2,0.9997,,PASS
calibration_poly3,r2,0.9998,,PASS
stability_0g,stddev,0.0018,g,PASS
stability_10g,stddev,0.0015,g,PASS
stability_20g,stddev,0.0019,g,PASS
stability_30g,stddev,0.0020,g,PASS
outlier_baseline,count,2,,INFO
outlier_interference,count,15,,INFO
outlier_filtered,percent,100,%,PASS
adaptive_stable,samples,50,,PASS
adaptive_transition,measurements,2,,PASS
adaptive_changing,samples,10,,PASS
comparison_5g,max_diff,0.002,g,PASS
comparison_15g,max_diff,0.003,g,PASS
comparison_25g,max_diff,0.002,g,PASS
temperature_baseline_t,value,24.5,C,INFO
temperature_changed_t,value,34.2,C,INFO
temperature_drift_without,value,0.008,g,INFO
temperature_drift_with,value,0.001,g,PASS
```

---

## Советы по использованию

1. **Подготовка**:
   - Убедитесь, что у вас есть все гирьки (0g, 5g, 10g, 15g, 20g, 25g, 30g)
   - Весы должны быть откалиброваны
   - Выделите ~1.5 часа для всех тестов

2. **Во время тестов**:
   - Не трогайте весы во время измерений
   - Кладите гирьки аккуратно (без ударов)
   - Для теста выбросов махайте рукой на расстоянии 5-10 см

3. **Если тест провалился**:
   - Используйте `test wizard skip` чтобы пропустить
   - Или `test wizard reset` чтобы начать заново
   - Проверьте калибровку и повторите

4. **Анализ результатов**:
   - Покажите терминал с отчётом разработчику
   - CSV файл можно открыть в Excel для анализа
   - Сравните результаты с целевыми значениями

---

## Технические детали

### Структура данных

```cpp
struct TestResults {
    // Metadata
    uint32_t timestamp_start;
    uint32_t timestamp_end;
    uint8_t tests_completed;
    
    // Test 1: Calibration
    float r2[4]; // linear, piecewise, poly2, poly3
    
    // Test 2: Stability
    struct {
        float min, max, mean, stddev;
        int measurements;
    } stability[4]; // 0g, 10g, 20g, 30g
    
    // Test 3: Outliers
    int outliers_baseline;
    int outliers_interference;
    int outliers_filtered;
    
    // Test 4: Adaptive
    int samples_stable;
    int samples_changing;
    int transition_measurements;
    
    // Test 5: Comparison
    float weights_5g[4];
    float weights_15g[4];
    float weights_25g[4];
    
    // Test 6: Temperature
    float temp_baseline;
    float temp_changed;
    float weight_baseline;
    float weight_changed_no_comp;
    float weight_changed_with_comp;
};
```

### State Machine

```
START → Test1 → Test2 → Test3 → Test4 → Test5 → Test6 → COMPLETE
         ↓       ↓       ↓       ↓       ↓       ↓
       SKIP    SKIP    SKIP    SKIP    SKIP    SKIP
```

### Память

- TestResults: ~200 байт
- Буферы измерений: ~400 байт (100 измерений × 4 байта)
- Итого: ~600 байт RAM

---

## FAQ

**Q: Можно ли пропустить тест?**
A: Да, используйте `test wizard skip`

**Q: Можно ли повторить один тест?**
A: Да, используйте `test wizard reset` и затем `test wizard start`, пропускайте ненужные тесты

**Q: Сколько времени занимают все тесты?**
A: Без температурного теста: ~1 час. С температурным: ~2-3 часа

**Q: Что делать если тест провалился?**
A: Проверьте калибровку, убедитесь что гирьки точные, повторите тест

**Q: Можно ли запустить только один тест?**
A: Пока нет, но можно пропустить остальные через `skip`

**Q: Где хранятся результаты?**
A: В RAM (пока ESP32 включён) и в CSV файле на Serial

---

## Пример полного прохождения

```bash
# Запуск мастера
> test wizard start

# Следуйте инструкциям...
# (кладёте гирьки, нажимаете ENTER)

# Через ~1 час
> test wizard report

# Копируете отчёт и показываете разработчику

# Экспорт в CSV
> test wizard export

# Копируете CSV из Serial
```

Готово! 🎉
