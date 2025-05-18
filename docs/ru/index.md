---
layout: default
title: Язык программирования ASTRA
---

# Язык программирования ASTRA

![Логотип ASTRA](/assets/images/astra_logo.png)

[![Статус сборки](https://img.shields.io/github/actions/workflow/status/infernasel/Astra/build.yml?branch=main&style=flat-square)](https://github.com/infernasel/Astra/actions)
[![Покрытие тестами](https://img.shields.io/badge/coverage-89.8%25-brightgreen?style=flat-square)](https://github.com/infernasel/Astra/actions)
[![Версия](https://img.shields.io/badge/version-0.1.1--alpha-blue?style=flat-square)](https://github.com/infernasel/Astra/releases)
[![Лицензия](https://img.shields.io/badge/license-Custom-orange?style=flat-square)](https://github.com/infernasel/Astra/blob/main/CUSTOM_LICENSE.md)
[![Платформы](https://img.shields.io/badge/platforms-Linux%20%7C%20Windows-lightgrey?style=flat-square)](https://github.com/infernasel/Astra/releases)

ASTRA (Autonomous System Task-oriented Reliable Architecture) — специализированный язык программирования, разработанный для управления космическими аппаратами и беспилотными летательными аппаратами (БПЛА). Этот проект направлен на создание высокоэффективного, безопасного языка для разработки программного обеспечения, используемого в управлении космическими аппаратами, спутниковых системах, межорбитальных перелетах и автономных полетах дронов.

## Ключевые особенности

- **Простой для изучения и понимания** для инженеров космической отрасли
- **Высокоскоростная компиляция** с минимальным использованием памяти
- **Эффективная поддержка** многопоточной обработки данных и асинхронных вычислений
- **Сильный модульный подход** к разработке
- **Типобезопасность** и защита от случайных изменений переменных
- **Специализированные возможности** для космоса и авиации:
  - Быстрые векторные операции и пространственные координаты
  - Специальные операции для расчетов траекторий и коррекции курса
  - Механизмы быстрого реагирования на внешние события
  - Простые способы описания моделей космических и атмосферных условий
  - Встроенные процедуры для моделирования объектов в космосе и атмосфере

## Пример кода

```astra
module FlightControl;

import Sensors;
import Navigation;

// Определение задачи для автономной навигации
task NavigateToPoint(point: Vec3) {
    // Обработка ошибок с помощью try-catch
    try {
        // Получение текущей позиции от GPS
        let currentPos = Sensors.GPS.getPosition();
        
        // Расчет пути
        let path = Navigation.findPath(currentPos, point);
        
        // Следование по пути с параллельным мониторингом
        parallel {
            // Основное управление полетом
            Navigation.followPath(path);
            
            // Параллельное обнаружение препятствий
            Sensors.ObstacleDetection.monitor();
        }
    } catch (e: NavigationError) {
        // Обработка ошибок навигации
        log.error("Навигация не удалась: ${e.message}");
        return false;
    }
    
    return true;
}
```

## Начало работы

Чтобы начать работу с ASTRA, ознакомьтесь с нашим [Руководством по началу работы](/ru/getting_started.html).

## Документация

- [Спецификация языка](/ru/language_specification.html)
- [Стандартная библиотека](/ru/standard_library.html)
- [Расширенные возможности](/ru/advanced_features.html)
- [Функции безопасности](/ru/safety_features.html)
- [Интеграция с оборудованием](/ru/hardware_integration.html)

## Лицензия

Этот проект доступен по [Пользовательской лицензии](https://github.com/infernasel/Astra/blob/main/CUSTOM_LICENSE.md) как для некоммерческого, так и для коммерческого использования в аэрокосмических приложениях и приложениях БПЛА.

Пожалуйста, внимательно ознакомьтесь с лицензией перед использованием этого программного обеспечения.