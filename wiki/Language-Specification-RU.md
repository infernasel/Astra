# Спецификация языка ASTRA

Этот документ предоставляет подробную спецификацию языка программирования ASTRA.

## Лексическая структура

### Ключевые слова

ASTRA резервирует следующие ключевые слова:

```
module    import    function    task    let    const    if    else    while    for    in
return    break     continue    try     catch   throw   parallel    async    await    true
false     null      this        super   new     delete  typeof      sizeof   as       is
```

### Идентификаторы

Идентификаторы в ASTRA должны начинаться с буквы или подчеркивания, за которыми следует любое количество букв, цифр или подчеркиваний.

```
[a-zA-Z_][a-zA-Z0-9_]*
```

### Литералы

#### Целочисленные литералы

```
123       // Десятичные
0x7B      // Шестнадцатеричные
0b1111011 // Двоичные
0o173     // Восьмеричные
```

#### Литералы с плавающей точкой

```
3.14
.5
1.
1e10
1.5e-5
```

#### Строковые литералы

```
"Привет, мир!"
"Строка 1\nСтрока 2"
"Экранирование \"кавычек\""
```

#### Символьные литералы

```
'A'
'\n'
'\''
```

#### Логические литералы

```
true
false
```

#### Нулевой литерал

```
null
```

## Типы данных

### Примитивные типы

- `int`: 32-битное целое число со знаком
- `float`: 64-битное число с плавающей точкой
- `bool`: Логическое значение (true или false)
- `char`: 16-битный символ Unicode
- `string`: Последовательность символов
- `void`: Отсутствие значения

### Составные типы

- `array`: Коллекция фиксированного размера элементов одного типа
- `vector`: Коллекция динамического размера элементов одного типа
- `map`: Коллекция пар ключ-значение
- `struct`: Пользовательская структура данных
- `enum`: Перечисляемый тип
- `tuple`: Коллекция фиксированного размера элементов разных типов

### Специальные типы

- `Vec2`: 2D вектор (x, y)
- `Vec3`: 3D вектор (x, y, z)
- `Quat`: Кватернион (x, y, z, w)
- `Matrix`: Матрица произвольных размеров
- `Time`: Представление времени
- `Duration`: Представление продолжительности

## Выражения

### Арифметические операторы

- `+`: Сложение
- `-`: Вычитание
- `*`: Умножение
- `/`: Деление
- `%`: Остаток от деления
- `**`: Возведение в степень

### Операторы сравнения

- `==`: Равно
- `!=`: Не равно
- `<`: Меньше
- `>`: Больше
- `<=`: Меньше или равно
- `>=`: Больше или равно

### Логические операторы

- `&&`: Логическое И
- `||`: Логическое ИЛИ
- `!`: Логическое НЕ

### Побитовые операторы

- `&`: Побитовое И
- `|`: Побитовое ИЛИ
- `^`: Побитовое исключающее ИЛИ
- `~`: Побитовое НЕ
- `<<`: Сдвиг влево
- `>>`: Сдвиг вправо

### Операторы присваивания

- `=`: Присваивание
- `+=`, `-=`, `*=`, `/=`, `%=`: Составное присваивание
- `&=`, `|=`, `^=`, `<<=`, `>>=`: Побитовое составное присваивание

### Другие операторы

- `()`: Вызов функции
- `[]`: Индексация массива
- `.`: Доступ к члену
- `->`: Доступ к члену через указатель
- `?:`: Тернарный условный оператор
- `??`: Оператор объединения с null
- `is`: Проверка типа
- `as`: Преобразование типа
- `typeof`: Информация о типе
- `sizeof`: Информация о размере

## Операторы

### Объявление переменных

```astra
let x = 5;
let y: int = 10;
const PI: float = 3.14159;
```

### Управление потоком

#### Оператор if

```astra
if (условие) {
    // код
} else if (другоеУсловие) {
    // код
} else {
    // код
}
```

#### Цикл while

```astra
while (условие) {
    // код
}
```

#### Цикл for

```astra
for (let i = 0; i < 10; i = i + 1) {
    // код
}

for (let элемент in коллекция) {
    // код
}
```

#### Break и Continue

```astra
while (true) {
    if (условие) break;
    if (другоеУсловие) continue;
}
```

#### Оператор return

```astra
function add(a: int, b: int): int {
    return a + b;
}
```

### Обработка ошибок

#### Try-Catch

```astra
try {
    // код, который может вызвать ошибку
} catch (e: ТипОшибки) {
    // обработка ошибки
} finally {
    // код очистки
}
```

#### Оператор throw

```astra
function divide(a: int, b: int): int {
    if (b == 0) {
        throw new DivisionByZeroError("Деление на ноль невозможно");
    }
    return a / b;
}
```

### Параллелизм

#### Блок parallel

```astra
parallel {
    // задача 1
    doSomething();
    
    // задача 2
    doSomethingElse();
}
```

#### Async-Await

```astra
async function fetchData(): string {
    let response = await httpGet("https://example.com/api");
    return response.body;
}
```

## Модули и импорты

### Объявление модуля

```astra
module MyModule;
```

### Оператор import

```astra
import Math;
import Navigation.Path;
import * from Sensors;
```

## Функции и задачи

### Объявление функции

```astra
function add(a: int, b: int): int {
    return a + b;
}
```

### Объявление задачи

```astra
task NavigateToPoint(point: Vec3): bool {
    // Реализация
    return true;
}
```

### Анонимные функции

```astra
let square = function(x: int): int {
    return x * x;
};
```

### Перегрузка функций

```astra
function add(a: int, b: int): int {
    return a + b;
}

function add(a: float, b: float): float {
    return a + b;
}
```

## Объектно-ориентированные возможности

### Объявление структуры

```astra
struct Point {
    x: float;
    y: float;
    
    function distance(other: Point): float {
        let dx = this.x - other.x;
        let dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
```

### Объявление перечисления

```astra
enum Direction {
    North,
    East,
    South,
    West
}
```

### Объявление интерфейса

```astra
interface Movable {
    function move(dx: float, dy: float): void;
    function getPosition(): Vec2;
}
```

### Реализация

```astra
struct Robot implements Movable {
    position: Vec2;
    
    function move(dx: float, dy: float): void {
        this.position.x += dx;
        this.position.y += dy;
    }
    
    function getPosition(): Vec2 {
        return this.position;
    }
}
```

## Управление памятью

ASTRA использует автоматическое управление памятью с детерминированным уничтожением объектов при выходе из области видимости.

## Аннотации

```astra
@Deprecated
function oldFunction(): void {
    // Реализация
}

@Inline
function fastFunction(): void {
    // Реализация
}

@Safe
function criticalFunction(): void {
    // Реализация
}
```

## Комментарии

```astra
// Однострочный комментарий

/*
 * Многострочный
 * комментарий
 */

/// Документационный комментарий для следующего объявления
function documentedFunction(): void {
    // Реализация
}
```