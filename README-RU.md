# 🚀 Язык программирования ASTRA

<div align="center">
  <img src="https://raw.githubusercontent.com/infernasel/Astra/master/docs/assets/astra-logo.png" alt="Логотип ASTRA" width="300"/>
  <br><br>
  
  [![Статус сборки](https://img.shields.io/github/actions/workflow/status/infernasel/Astra/build.yml?branch=main&style=for-the-badge&logo=github-actions&logoColor=white)](https://github.com/infernasel/Astra/actions)
  [![Покрытие тестами](https://img.shields.io/badge/покрытие-89.8%25-brightgreen?style=for-the-badge&logo=codecov&logoColor=white)](https://github.com/infernasel/Astra/actions)
  [![Версия](https://img.shields.io/badge/версия-0.1.1--alpha-blue?style=for-the-badge&logo=semver&logoColor=white)](https://github.com/infernasel/Astra/releases)
  [![Лицензия](https://img.shields.io/badge/лицензия-Custom-orange?style=for-the-badge&logo=open-source-initiative&logoColor=white)](https://github.com/infernasel/Astra/blob/main/CUSTOM_LICENSE.md)
  
  [![Платформы](https://img.shields.io/badge/платформы-Linux%20%7C%20Windows-lightgrey?style=for-the-badge&logo=linux&logoColor=white)](https://github.com/infernasel/Astra/releases)
  [![Документация](https://img.shields.io/badge/документация-wiki-yellow?style=for-the-badge&logo=gitbook&logoColor=white)](https://github.com/infernasel/Astra/wiki)
  [![PRs Welcome](https://img.shields.io/badge/PRs-приветствуются-brightgreen.svg?style=for-the-badge&logo=github&logoColor=white)](https://github.com/infernasel/Astra/blob/main/CONTRIBUTING.md)
  
  [![GitHub Stars](https://img.shields.io/github/stars/infernasel/Astra?style=for-the-badge&logo=github&logoColor=white)](https://github.com/infernasel/Astra/stargazers)
  [![GitHub Forks](https://img.shields.io/github/forks/infernasel/Astra?style=for-the-badge&logo=git&logoColor=white)](https://github.com/infernasel/Astra/network/members)
  [![GitHub Issues](https://img.shields.io/github/issues/infernasel/Astra?style=for-the-badge&logo=github&logoColor=white)](https://github.com/infernasel/Astra/issues)
  
  [![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://github.com/infernasel/Astra)
  [![LLVM](https://img.shields.io/badge/LLVM-Powered-2C3E50?style=for-the-badge&logo=llvm&logoColor=white)](https://github.com/infernasel/Astra)
  [![UAV](https://img.shields.io/badge/UAV-Compatible-1abc9c?style=for-the-badge&logo=drone&logoColor=white)](https://github.com/infernasel/Astra)
  
  [![Last Commit](https://img.shields.io/github/last-commit/infernasel/Astra?style=for-the-badge&logo=git&logoColor=white)](https://github.com/infernasel/Astra/commits/master)
  [![Open Issues](https://img.shields.io/github/issues-raw/infernasel/Astra?style=for-the-badge&logo=github&logoColor=white)](https://github.com/infernasel/Astra/issues)
  [![Contributors](https://img.shields.io/github/contributors/infernasel/Astra?style=for-the-badge&logo=github&logoColor=white)](https://github.com/infernasel/Astra/graphs/contributors)
</div>

ASTRA (Autonomous System Task-oriented Reliable Architecture) - специализированный язык программирования, разработанный для управления космическими аппаратами и беспилотными летательными аппаратами (БПЛА). Этот проект направлен на создание высокоэффективного, ориентированного на безопасность языка для разработки программного обеспечения, используемого в управлении космическими аппаратами, спутниковых системах, межорбитальных перелетах и автономных полетах дронов.

## Обзор проекта

ASTRA предлагает уникальный подход к программированию систем управления полетом, сочетая строгую типизацию, встроенные механизмы безопасности и высокую производительность. Язык разработан с учетом специфических требований аэрокосмической отрасли и операторов БПЛА.

### Ключевые особенности

- **Безопасность на уровне системы**: Встроенные проверки времени компиляции и выполнения для предотвращения критических ошибок
- **Детерминированное выполнение**: Предсказуемое поведение для критически важных систем управления
- **Оптимизированная производительность**: Компиляция в эффективный машинный код с использованием LLVM
- **Специализированные библиотеки**: Встроенная поддержка для навигации, управления и телеметрии
- **Кросс-платформенность**: Поддержка различных аппаратных платформ и операционных систем

## Начало работы

### Установка

Клонируйте репозиторий:

```bash
git clone https://github.com/infernasel/Astra.git
cd Astra
```

### Сборка из исходников

```bash
./build.sh
```

### Предварительно скомпилированные бинарные файлы

- **Linux (x86_64)**: Доступны в директории [release/linux](https://github.com/infernasel/Astra/tree/main/release/linux)
- **Windows (x86_64)**: Доступны в директории [release/windows](https://github.com/infernasel/Astra/tree/main/release/windows)

## Документация

Подробная документация доступна в нашей [Wiki](https://github.com/infernasel/Astra/wiki).

## Примеры

Примеры кода ASTRA можно найти в директории [examples](https://github.com/infernasel/Astra/tree/main/examples).

## Дорожная карта

Ознакомьтесь с нашей [дорожной картой](https://github.com/infernasel/Astra/blob/main/ROADMAP.md) для получения информации о планируемых функциях и улучшениях.

## Вклад в проект

Мы приветствуем вклады от сообщества! Пожалуйста, ознакомьтесь с нашим [руководством по внесению вклада](https://github.com/infernasel/Astra/blob/main/CONTRIBUTING.md).

## Лицензия

Этот проект доступен по [Пользовательской лицензии](https://github.com/infernasel/Astra/blob/main/CUSTOM_LICENSE.md) как для некоммерческого, так и для коммерческого использования в аэрокосмических приложениях и приложениях БПЛА.

## Контакты

- **Email**: support@arlist-interactive.ru
- **Проблемы**: [Трекер проблем GitHub](https://github.com/infernasel/Astra/issues)