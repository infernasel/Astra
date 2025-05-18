# Спецификация языка ASTRA

## Введение

ASTRA - это статически типизированный, компилируемый язык программирования, разработанный специально для аэрокосмических приложений и систем управления БПЛА. Он сочетает в себе строгую типизацию, детерминированное выполнение и высокую производительность, что делает его идеальным для критически важных систем.

## Синтаксис

### Базовая структура программы

Программа на ASTRA состоит из модулей, каждый из которых может содержать объявления функций, типов и переменных.

```astra
module navigation;

import math;
import sensors;

// Объявление константы
const MAX_ALTITUDE: float = 10000.0;

// Объявление функции
fn calculate_trajectory(current_pos: Vector3, target_pos: Vector3) -> Trajectory {
    // Реализация функции
}
```

### Типы данных

ASTRA предоставляет следующие встроенные типы данных:

| Тип | Описание | Пример |
|-----|----------|--------|
| `int` | 32-битное целое число со знаком | `let x: int = 42;` |
| `int64` | 64-битное целое число со знаком | `let y: int64 = 9223372036854775807;` |
| `uint` | 32-битное целое число без знака | `let z: uint = 4294967295;` |
| `float` | 32-битное число с плавающей точкой | `let a: float = 3.14;` |
| `double` | 64-битное число с плавающей точкой | `let b: double = 3.141592653589793;` |
| `bool` | Логический тип (true/false) | `let c: bool = true;` |
| `char` | Символьный тип | `let d: char = 'A';` |
| `string` | Строковый тип | `let e: string = "Hello, ASTRA!";` |

### Пользовательские типы

#### Структуры

```astra
struct Vector3 {
    x: float,
    y: float,
    z: float
}

// Создание экземпляра структуры
let position = Vector3 { x: 10.0, y: 20.0, z: 30.0 };
```

#### Перечисления

```astra
enum FlightMode {
    Manual,
    Assisted,
    Autonomous
}

// Использование перечисления
let mode: FlightMode = FlightMode.Autonomous;
```

### Управляющие конструкции

#### Условные операторы

```astra
if altitude > MAX_ALTITUDE {
    begin_descent();
} else if altitude < MIN_ALTITUDE {
    begin_ascent();
} else {
    maintain_altitude();
}
```

#### Циклы

```astra
// Цикл while
while battery_level > 10.0 {
    perform_operation();
}

// Цикл for
for i in 0..10 {
    log_data(i);
}

// Бесконечный цикл с выходом
loop {
    read_sensors();
    if emergency_detected() {
        break;
    }
}
```

### Функции

```astra
// Функция с параметрами и возвращаемым значением
fn calculate_distance(p1: Vector3, p2: Vector3) -> float {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let dz = p2.z - p1.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Функция без возвращаемого значения
fn log_position(position: Vector3) {
    log("Position: " + position.x + ", " + position.y + ", " + position.z);
}
```

### Обработка ошибок

ASTRA использует систему результатов для обработки ошибок, избегая исключений.

```astra
// Функция, которая может завершиться ошибкой
fn read_sensor_data() -> Result<SensorData, SensorError> {
    if sensor_available() {
        return Ok(get_sensor_readings());
    } else {
        return Err(SensorError.NotAvailable);
    }
}

// Использование результата
let result = read_sensor_data();
match result {
    Ok(data) => process_data(data),
    Err(SensorError.NotAvailable) => use_backup_data(),
    Err(e) => log_error(e)
}
```

## Система типов

### Безопасность типов

ASTRA обеспечивает строгую типизацию для предотвращения ошибок времени выполнения. Все преобразования типов должны быть явными.

```astra
let i: int = 42;
let f: float = i as float; // Явное преобразование типов
```

### Обобщенное программирование

```astra
// Обобщенная функция
fn find_min<T: Comparable>(a: T, b: T) -> T {
    if a < b {
        return a;
    } else {
        return b;
    }
}

// Обобщенная структура
struct Pair<T> {
    first: T,
    second: T
}
```

## Параллелизм и многопоточность

ASTRA предоставляет безопасные примитивы для параллельного программирования.

```astra
// Определение задачи
task read_sensors() {
    loop {
        let readings = get_all_sensor_readings();
        update_telemetry(readings);
        sleep(10); // Пауза в миллисекундах
    }
}

// Запуск задачи
spawn read_sensors();
```

## Встроенные библиотеки

ASTRA включает специализированные библиотеки для аэрокосмических приложений:

- `navigation`: Функции для навигации и определения положения
- `control`: Алгоритмы управления полетом
- `telemetry`: Сбор и передача данных телеметрии
- `sensors`: Интерфейсы для работы с датчиками
- `math`: Математические функции и алгоритмы

## Компиляция и выполнение

Программы на ASTRA компилируются в машинный код с использованием инфраструктуры LLVM, что обеспечивает высокую производительность и возможность кросс-компиляции для различных целевых платформ.

```bash
# Компиляция программы
astrac source.as -o program

# Выполнение программы
./program
```

## Дополнительные ресурсы

Для получения более подробной информации о языке ASTRA, обратитесь к следующим ресурсам:

- [Руководство по началу работы](Getting-Started-RU.md)
- [Примеры кода](https://github.com/infernasel/Astra/tree/main/examples)
- [Стандартная библиотека API](https://github.com/infernasel/Astra/wiki/API-Reference)