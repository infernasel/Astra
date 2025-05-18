---
layout: default
title: ASTRA Programming Language
---

# ASTRA Programming Language

![ASTRA Logo](/assets/images/astra_logo.png)

[![Build Status](https://img.shields.io/github/actions/workflow/status/infernasel/astra-language/build.yml?branch=main&style=flat-square)](https://github.com/infernasel/astra-language/actions)
[![Test Coverage](https://img.shields.io/badge/coverage-89.8%25-brightgreen?style=flat-square)](https://github.com/infernasel/astra-language/actions)
[![Version](https://img.shields.io/badge/version-0.1.1--alpha-blue?style=flat-square)](https://github.com/infernasel/astra-language/releases)
[![License](https://img.shields.io/badge/license-Custom-orange?style=flat-square)](https://github.com/infernasel/astra-language/blob/main/CUSTOM_LICENSE.md)
[![Platforms](https://img.shields.io/badge/platforms-Linux%20%7C%20Windows-lightgrey?style=flat-square)](https://github.com/infernasel/astra-language/releases)

ASTRA (Autonomous System Task-oriented Reliable Architecture) is a specialized programming language designed for controlling spacecraft and unmanned aerial vehicles (UAVs). This project aims to create a high-efficiency, safety-focused language for developing software used in spacecraft control, satellite systems, interorbital transfers, and autonomous drone flights.

## Key Features

- **Easy to learn and understand** for space engineers
- **High-speed compilation** with minimal memory usage
- **Efficient support** for multi-threaded data processing and asynchronous computing
- **Strong modular approach** to development
- **Type safety** and protection against accidental variable changes
- **Specialized capabilities** for space and aviation:
  - Fast vector operations and spatial coordinates
  - Special operations for trajectory calculations and course corrections
  - Rapid response mechanisms for external events
  - Simple ways to describe space and atmospheric condition models
  - Built-in procedures for simulating objects in space and atmosphere

## Example Code

```astra
module FlightControl;

import Sensors;
import Navigation;

// Define a task for autonomous navigation
task NavigateToPoint(point: Vec3) {
    // Error handling with try-catch
    try {
        // Get current position from GPS
        let currentPos = Sensors.GPS.getPosition();
        
        // Calculate path
        let path = Navigation.findPath(currentPos, point);
        
        // Follow path with concurrent monitoring
        parallel {
            // Main flight control
            Navigation.followPath(path);
            
            // Concurrent obstacle detection
            Sensors.ObstacleDetection.monitor();
        }
    } catch (e: NavigationError) {
        // Handle navigation errors
        log.error("Navigation failed: ${e.message}");
        return false;
    }
    
    return true;
}
```

## Getting Started

To get started with ASTRA, check out our [Getting Started Guide](/getting_started.html).

## Documentation

- [Language Specification](/language_specification.html)
- [Standard Library](/standard_library.html)
- [Advanced Features](/advanced_features.html)
- [Safety Features](/safety_features.html)
- [Hardware Integration](/hardware_integration.html)

## License

This project is available under a [Custom License](https://github.com/infernasel/astra-language/blob/main/CUSTOM_LICENSE.md) for both non-commercial and commercial use in aerospace and UAV applications.

Please review the license carefully before using this software.