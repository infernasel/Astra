# ASTRA Language Specification (Draft)

## 1. Introduction

ASTRA is a specialized programming language designed for spacecraft and UAV control systems. This document outlines the language's syntax, semantics, and core features.

## 2. Language Fundamentals

### 2.1 Syntax Overview

ASTRA uses a clean, readable syntax designed to be accessible to engineers with minimal programming experience while providing powerful capabilities for complex aerospace applications.

### 2.2 Data Types

#### Basic Types
- `int` - Integer values
- `float` - Floating-point values (high precision for trajectory calculations)
- `bool` - Boolean values (true/false)
- `string` - Text strings
- `time` - Time representation with nanosecond precision

#### Specialized Types
- `vector3` - Three-dimensional vector for spatial coordinates
- `quaternion` - For rotation representation
- `matrix` - For mathematical operations
- `trajectory` - For defining and manipulating flight paths
- `sensor` - For handling sensor data

### 2.3 Variables and Constants

Variables in ASTRA are statically typed and must be declared before use:

```astra
var speed: float = 0.0;
const MAX_ALTITUDE: float = 35786.0; // Geostationary orbit altitude in km
```

Constants are immutable after declaration and are recommended for values that should not change during execution.

### 2.4 Control Structures

#### Conditional Statements
```astra
if condition {
    // code
} else if another_condition {
    // code
} else {
    // code
}
```

#### Loops
```astra
// Count-controlled loop
for i in 0..10 {
    // code
}

// Condition-controlled loop
while condition {
    // code
}

// Infinite loop with break condition
loop {
    // code
    if exit_condition {
        break;
    }
}
```

### 2.5 Functions

Functions are defined with the `func` keyword:

```astra
func calculate_orbit(altitude: float, inclination: float) -> trajectory {
    // Function body
    return new_trajectory;
}
```

### 2.6 Error Handling

ASTRA uses a robust error handling system:

```astra
try {
    // Code that might fail
} catch error: OutOfFuelError {
    // Handle specific error
} catch error {
    // Handle any other error
} finally {
    // Always executed
}
```

### 2.7 Modules and Imports

Code organization through modules:

```astra
import navigation.trajectory;
import sensors.camera;

module thruster_control {
    // Module contents
}
```

## 3. Specialized Features

### 3.1 Real-time Constraints

Time-critical operations can be annotated with timing constraints:

```astra
@deadline(5ms)
func emergency_stop() {
    // Must complete within 5 milliseconds
}
```

### 3.2 Concurrent Programming

```astra
// Create a task that runs concurrently
task update_telemetry() {
    loop {
        // Update telemetry data
        sleep(100ms);
    }
}

// Synchronization primitives
var fuel_lock: mutex;
with fuel_lock {
    // Critical section with exclusive access
}
```

### 3.3 Safety Features

```astra
// Range-checked variable
var altitude: float<0.0..100000.0>;

// Formal verification annotation
@verify(altitude > 0.0, "Altitude must be positive")
func adjust_orbit(altitude: float) {
    // Function implementation
}
```

### 3.4 Hardware Interaction

```astra
// Define a sensor interface
sensor gps {
    func get_coordinates() -> vector3;
    func get_accuracy() -> float;
}

// Implement for specific hardware
implement gps for GPS_HARDWARE_X {
    // Implementation details
}
```

## 4. Standard Library

ASTRA includes a comprehensive standard library with modules for:

- Orbital mechanics
- Navigation
- Sensor data processing
- Communication protocols
- Telemetry
- Simulation

## 5. Memory Management

ASTRA uses deterministic memory management with compile-time checks to prevent memory leaks and ensure predictable performance.

## 6. Compilation and Execution

ASTRA programs are compiled to efficient machine code with specialized optimizations for aerospace applications. The compiler performs extensive static analysis to catch potential issues before runtime.

---

*Note: This specification is a draft and subject to change as the language design evolves.*