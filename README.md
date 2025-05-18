<p align="center">
  <img src="assets/astra_logo_dark.png" alt="ASTRA Programming Language Logo" width="400"/>
</p>

# 🚀 ASTRA Programming Language

<div align="center">
  <img src="https://raw.githubusercontent.com/infernasel/Astra/master/docs/assets/astra-logo.png" alt="ASTRA Logo" width="300"/>
  <br><br>
  
  [![Build Status](https://img.shields.io/github/actions/workflow/status/infernasel/Astra/build.yml?branch=main&style=for-the-badge&logo=github-actions&logoColor=white)](https://github.com/infernasel/Astra/actions)
  [![Test Coverage](https://img.shields.io/badge/coverage-89.8%25-brightgreen?style=for-the-badge&logo=codecov&logoColor=white)](https://github.com/infernasel/Astra/actions)
  [![Version](https://img.shields.io/badge/version-0.1.1--alpha-blue?style=for-the-badge&logo=semver&logoColor=white)](https://github.com/infernasel/Astra/releases)
  [![License](https://img.shields.io/badge/license-Custom-orange?style=for-the-badge&logo=open-source-initiative&logoColor=white)](https://github.com/infernasel/Astra/blob/main/CUSTOM_LICENSE.md)
  
  [![Platforms](https://img.shields.io/badge/platforms-Linux%20%7C%20Windows-lightgrey?style=for-the-badge&logo=linux&logoColor=white)](https://github.com/infernasel/Astra/releases)
  [![Documentation](https://img.shields.io/badge/docs-wiki-yellow?style=for-the-badge&logo=gitbook&logoColor=white)](https://github.com/infernasel/Astra/wiki)
  [![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=for-the-badge&logo=github&logoColor=white)](https://github.com/infernasel/Astra/blob/main/CONTRIBUTING.md)
  
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

ASTRA (Autonomous System Task-oriented Reliable Architecture) is a specialized programming language designed for controlling spacecraft and unmanned aerial vehicles (UAVs). This project aims to create a high-efficiency, safety-focused language for developing software used in spacecraft control, satellite systems, interorbital transfers, and autonomous drone flights.

## Project Overview

The ASTRA language is designed with the following key features:

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

## Project Structure

- `/compiler` - ASTRA language compiler implementation
- `/interpreter` - ASTRA language interpreter
- `/docs` - Language specification and documentation
- `/examples` - Example ASTRA programs
- `/tools` - Development tools and utilities
- `/hardware` - Hardware abstraction layer for different UAV platforms

## Development Status

- **Current Version**: v0.1.1 Alpha
- **Test Coverage**: 
  - Lines: 89.8%
  - Functions: 83.6%
- **Platforms**:
  - Linux (x86_64)
  - Windows (x86_64)

## Getting Started

### Prerequisites

- CMake 3.10 or higher
- C++17 compatible compiler
- LLVM 10.0 or higher (for code generation)

### Building from Source

```bash
# Clone the repository
git clone https://github.com/infernasel/Astra.git
cd astra-language

# Build the compiler
mkdir -p build/compiler && cd build/compiler
cmake ../../compiler
make

# Run tests
make astra_tests
./tests/astra_tests
```

### Pre-built Binaries

Pre-built binaries are available for the following platforms:

- **Windows (x86_64)**: Available in the [release/windows](https://github.com/infernasel/Astra/tree/main/release/windows) directory
- **Linux (x86_64)**: Coming soon

## Example ASTRA Code

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

## Safety Features

- Formal verification at source code level
- Automatic algorithm correctness checking
- Detailed exception and error analysis
- Comprehensive logging system

## Documentation

For detailed documentation, please visit our [Wiki](https://github.com/infernasel/Astra/wiki).

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for more information.

## License

This project is available under a [Custom License](CUSTOM_LICENSE.md) for both non-commercial and commercial use in aerospace and UAV applications.

Please review the license carefully before using this software.

## Contact

For support or inquiries, please contact us at [support@arlist-interactive.ru](mailto:support@arlist-interactive.ru)