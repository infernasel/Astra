# Getting Started with ASTRA

This guide will help you set up your development environment and create your first ASTRA program.

## Installation

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

## Your First ASTRA Program

Create a file named `hello.astra` with the following content:

```astra
// This is a simple ASTRA program
function main() {
    print("Hello, World!");
    return 0;
}
```

### Compiling and Running

```bash
# Compile the program
./astrac hello.astra -o hello

# Run the program
./hello
```

## Basic Syntax

### Variables and Types

```astra
// Variable declaration with type inference
let name = "ASTRA";

// Explicit type declaration
let age: int = 1;

// Constants
const PI: float = 3.14159;

// Basic types
let i: int = 42;
let f: float = 3.14;
let b: bool = true;
let s: string = "Hello";
let c: char = 'A';
```

### Functions

```astra
// Function declaration
function add(a: int, b: int): int {
    return a + b;
}

// Function with default parameters
function greet(name: string = "World"): string {
    return "Hello, " + name + "!";
}

// Function call
let sum = add(5, 3);
let greeting = greet("ASTRA");
```

### Control Flow

```astra
// If statement
if (x > 0) {
    print("Positive");
} else if (x < 0) {
    print("Negative");
} else {
    print("Zero");
}

// While loop
let i = 0;
while (i < 10) {
    print(i);
    i = i + 1;
}

// For loop
for (let i = 0; i < 10; i = i + 1) {
    print(i);
}

// Range-based for loop
for (let item in items) {
    print(item);
}
```

## Next Steps

- Explore the [Language Specification](Language-Specification)
- Learn about the [Standard Library](Standard-Library)
- Discover [Advanced Features](Advanced-Features)
- Understand [Safety Features](Safety-Features)
- Dive into [Hardware Integration](Hardware-Integration)