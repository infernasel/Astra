# Начало работы с ASTRA

Это руководство поможет вам настроить среду разработки и создать вашу первую программу на языке ASTRA.

## Установка

### Предварительные требования

- CMake 3.10 или выше
- Компилятор, совместимый с C++17
- LLVM 10.0 или выше (для генерации кода)

### Сборка из исходного кода

```bash
# Клонирование репозитория
git clone https://github.com/infernasel/Astra.git
cd astra-language

# Сборка компилятора
mkdir -p build/compiler && cd build/compiler
cmake ../../compiler
make

# Запуск тестов
make astra_tests
./tests/astra_tests
```

### Предварительно собранные бинарные файлы

Предварительно собранные бинарные файлы доступны для следующих платформ:

- **Windows (x86_64)**: Доступны в директории [release/windows](https://github.com/infernasel/Astra/tree/main/release/windows)
- **Linux (x86_64)**: Скоро будут доступны

## Ваша первая программа на ASTRA

Создайте файл с именем `hello.astra` со следующим содержимым:

```astra
// Это простая программа на ASTRA
function main() {
    print("Привет, мир!");
    return 0;
}
```

### Компиляция и запуск

```bash
# Компиляция программы
./astrac hello.astra -o hello

# Запуск программы
./hello
```

## Основной синтаксис

### Переменные и типы

```astra
// Объявление переменной с выводом типа
let name = "ASTRA";

// Явное объявление типа
let age: int = 1;

// Константы
const PI: float = 3.14159;

// Базовые типы
let i: int = 42;
let f: float = 3.14;
let b: bool = true;
let s: string = "Привет";
let c: char = 'А';
```

### Функции

```astra
// Объявление функции
function add(a: int, b: int): int {
    return a + b;
}

// Функция с параметрами по умолчанию
function greet(name: string = "мир"): string {
    return "Привет, " + name + "!";
}

// Вызов функции
let sum = add(5, 3);
let greeting = greet("ASTRA");
```

### Управление потоком выполнения

```astra
// Условный оператор if
if (x > 0) {
    print("Положительное");
} else if (x < 0) {
    print("Отрицательное");
} else {
    print("Ноль");
}

// Цикл while
let i = 0;
while (i < 10) {
    print(i);
    i = i + 1;
}

// Цикл for
for (let i = 0; i < 10; i = i + 1) {
    print(i);
}

// Цикл for по диапазону
for (let item in items) {
    print(item);
}
```

## Следующие шаги

- Изучите [Спецификацию языка](Language-Specification-RU)
- Узнайте о [Стандартной библиотеке](Standard-Library-RU)
- Откройте для себя [Расширенные возможности](Advanced-Features-RU)
- Поймите [Функции безопасности](Safety-Features-RU)
- Погрузитесь в [Интеграцию с оборудованием](Hardware-Integration-RU)