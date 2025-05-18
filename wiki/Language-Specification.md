# ASTRA Language Specification

This document provides a comprehensive specification of the ASTRA programming language.

## Lexical Structure

### Keywords

ASTRA reserves the following keywords:

```
module    import    function    task    let    const    if    else    while    for    in
return    break     continue    try     catch   throw   parallel    async    await    true
false     null      this        super   new     delete  typeof      sizeof   as       is
```

### Identifiers

Identifiers in ASTRA must start with a letter or underscore, followed by any number of letters, digits, or underscores.

```
[a-zA-Z_][a-zA-Z0-9_]*
```

### Literals

#### Integer Literals

```
123       // Decimal
0x7B      // Hexadecimal
0b1111011 // Binary
0o173     // Octal
```

#### Floating-Point Literals

```
3.14
.5
1.
1e10
1.5e-5
```

#### String Literals

```
"Hello, World!"
"Line 1\nLine 2"
"Escape \"quotes\""
```

#### Character Literals

```
'A'
'\n'
'\''
```

#### Boolean Literals

```
true
false
```

#### Null Literal

```
null
```

## Data Types

### Primitive Types

- `int`: 32-bit signed integer
- `float`: 64-bit floating-point number
- `bool`: Boolean value (true or false)
- `char`: 16-bit Unicode character
- `string`: Sequence of characters
- `void`: Absence of a value

### Composite Types

- `array`: Fixed-size collection of elements of the same type
- `vector`: Dynamically-sized collection of elements of the same type
- `map`: Collection of key-value pairs
- `struct`: User-defined data structure
- `enum`: Enumerated type
- `tuple`: Fixed-size collection of elements of different types

### Special Types

- `Vec2`: 2D vector (x, y)
- `Vec3`: 3D vector (x, y, z)
- `Quat`: Quaternion (x, y, z, w)
- `Matrix`: Matrix of arbitrary dimensions
- `Time`: Time representation
- `Duration`: Duration representation

## Expressions

### Arithmetic Operators

- `+`: Addition
- `-`: Subtraction
- `*`: Multiplication
- `/`: Division
- `%`: Modulo
- `**`: Exponentiation

### Comparison Operators

- `==`: Equal to
- `!=`: Not equal to
- `<`: Less than
- `>`: Greater than
- `<=`: Less than or equal to
- `>=`: Greater than or equal to

### Logical Operators

- `&&`: Logical AND
- `||`: Logical OR
- `!`: Logical NOT

### Bitwise Operators

- `&`: Bitwise AND
- `|`: Bitwise OR
- `^`: Bitwise XOR
- `~`: Bitwise NOT
- `<<`: Left shift
- `>>`: Right shift

### Assignment Operators

- `=`: Assignment
- `+=`, `-=`, `*=`, `/=`, `%=`: Compound assignment
- `&=`, `|=`, `^=`, `<<=`, `>>=`: Bitwise compound assignment

### Other Operators

- `()`: Function call
- `[]`: Array indexing
- `.`: Member access
- `->`: Pointer member access
- `?:`: Ternary conditional
- `??`: Null coalescing
- `is`: Type checking
- `as`: Type conversion
- `typeof`: Type information
- `sizeof`: Size information

## Statements

### Variable Declaration

```astra
let x = 5;
let y: int = 10;
const PI: float = 3.14159;
```

### Control Flow

#### If Statement

```astra
if (condition) {
    // code
} else if (anotherCondition) {
    // code
} else {
    // code
}
```

#### While Loop

```astra
while (condition) {
    // code
}
```

#### For Loop

```astra
for (let i = 0; i < 10; i = i + 1) {
    // code
}

for (let item in collection) {
    // code
}
```

#### Break and Continue

```astra
while (true) {
    if (condition) break;
    if (anotherCondition) continue;
}
```

#### Return Statement

```astra
function add(a: int, b: int): int {
    return a + b;
}
```

### Error Handling

#### Try-Catch

```astra
try {
    // code that might throw an error
} catch (e: ErrorType) {
    // handle the error
} finally {
    // cleanup code
}
```

#### Throw Statement

```astra
function divide(a: int, b: int): int {
    if (b == 0) {
        throw new DivisionByZeroError("Cannot divide by zero");
    }
    return a / b;
}
```

### Concurrency

#### Parallel Block

```astra
parallel {
    // task 1
    doSomething();
    
    // task 2
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

## Modules and Imports

### Module Declaration

```astra
module MyModule;
```

### Import Statement

```astra
import Math;
import Navigation.Path;
import * from Sensors;
```

## Functions and Tasks

### Function Declaration

```astra
function add(a: int, b: int): int {
    return a + b;
}
```

### Task Declaration

```astra
task NavigateToPoint(point: Vec3): bool {
    // Implementation
    return true;
}
```

### Anonymous Functions

```astra
let square = function(x: int): int {
    return x * x;
};
```

### Function Overloading

```astra
function add(a: int, b: int): int {
    return a + b;
}

function add(a: float, b: float): float {
    return a + b;
}
```

## Object-Oriented Features

### Struct Declaration

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

### Enum Declaration

```astra
enum Direction {
    North,
    East,
    South,
    West
}
```

### Interface Declaration

```astra
interface Movable {
    function move(dx: float, dy: float): void;
    function getPosition(): Vec2;
}
```

### Implementation

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

## Memory Management

ASTRA uses automatic memory management with deterministic destruction of objects when they go out of scope.

## Annotations

```astra
@Deprecated
function oldFunction(): void {
    // Implementation
}

@Inline
function fastFunction(): void {
    // Implementation
}

@Safe
function criticalFunction(): void {
    // Implementation
}
```

## Comments

```astra
// Single-line comment

/*
 * Multi-line
 * comment
 */

/// Documentation comment for the following declaration
function documentedFunction(): void {
    // Implementation
}
```