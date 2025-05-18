# ASTRA Advanced Features

This document describes the advanced features of the ASTRA programming language that make it particularly well-suited for aerospace and UAV applications.

## Safety-Critical Annotations

ASTRA provides a set of annotations that can be used to mark code as safety-critical and enforce additional checks and guarantees.

### `@safety_critical`

The `@safety_critical` annotation marks a function as safety-critical, indicating that it should receive extra scrutiny during static analysis and verification.

```astra
@safety_critical
func control_thrust(engine_id: int, thrust_level: float) -> bool {
    // This function controls engine thrust and is safety-critical
    // Additional static analysis will be performed on this function
    // ...
}
```

### `@verify`

The `@verify` annotation specifies a condition that must be true at a specific point in the code. The compiler will attempt to statically verify this condition or insert a runtime check.

```astra
@verify(thrust_level >= 0.0 && thrust_level <= 1.0, "Thrust level must be between 0 and 1")
func set_thrust(thrust_level: float) -> void {
    // The verify annotation ensures thrust_level is in valid range
    // ...
}
```

### `@invariant`

The `@invariant` annotation specifies a condition that must remain true throughout the execution of a function or method.

```astra
class AttitudeController {
    private pid_roll: PID;
    private pid_pitch: PID;
    private pid_yaw: PID;
    
    @invariant(this.pid_roll != null && this.pid_pitch != null && this.pid_yaw != null,
              "PID controllers must not be null")
    public func update(attitude: Quaternion, target: Quaternion) -> Vector3 {
        // The invariant ensures PID controllers are never null
        // ...
    }
}
```

### `@deadline`

The `@deadline` annotation specifies a maximum execution time for a function. The compiler will attempt to statically verify this deadline or insert runtime monitoring.

```astra
@deadline(5ms)
func update_control_loop() -> void {
    // This function must complete within 5 milliseconds
    // ...
}
```

### `@fallback`

The `@fallback` annotation specifies a fallback value or action to take if a function fails or exceeds its deadline.

```astra
@deadline(10ms)
@fallback(return last_valid_reading)
func read_sensor() -> SensorReading {
    // If this function exceeds its 10ms deadline,
    // it will return the last valid reading instead
    // ...
}
```

## Formal Verification

ASTRA includes built-in support for formal verification of critical code sections. The compiler can generate verification conditions that can be checked by external theorem provers.

### Specifying Pre and Post Conditions

```astra
func binary_search(array: array<int>, value: int) -> int
    requires array != null
    requires array.is_sorted()
    ensures result == -1 || array[result] == value
    ensures result >= 0 ==> array[result] == value
    ensures result == -1 ==> !array.contains(value)
{
    // Implementation of binary search
    // The compiler will verify that the implementation
    // satisfies the specified pre and post conditions
    // ...
}
```

### Proving Loop Invariants

```astra
func sum(numbers: array<int>) -> int
    requires numbers != null
    ensures result == sum_of(numbers)
{
    var total = 0;
    var i = 0;
    
    while i < numbers.length
        invariant i >= 0 && i <= numbers.length
        invariant total == sum_of(numbers.slice(0, i))
    {
        total += numbers[i];
        i++;
    }
    
    return total;
}
```

## Memory Safety

ASTRA provides several features to ensure memory safety without sacrificing performance.

### Ownership and Borrowing

Similar to Rust, ASTRA uses an ownership system to prevent data races and ensure memory safety without garbage collection.

```astra
func process_data(data: owned array<float>) -> array<float> {
    // This function takes ownership of the data array
    // The caller can no longer access it after this call
    
    var result = transform(data);
    
    // The data array is automatically freed when it goes out of scope
    return result;
}

func analyze_data(data: borrowed array<float>) -> Stats {
    // This function borrows the data array
    // The caller retains ownership and responsibility for freeing it
    
    var stats = compute_statistics(data);
    
    // The function cannot modify the data array
    return stats;
}

func modify_data(data: borrowed mut array<float>) -> void {
    // This function borrows the data array mutably
    // The caller retains ownership but cannot access it while borrowed
    
    for i in 0..data.length {
        data[i] = process(data[i]);
    }
}
```

### Region-Based Memory Management

For real-time systems, ASTRA provides region-based memory management to avoid unpredictable garbage collection pauses.

```astra
func process_telemetry() -> TelemetryReport {
    // Create a memory region for this function
    region telemetry_region {
        // All allocations in this block use the region
        var data = array<SensorReading>[1000];
        
        // Fill the array with sensor readings
        for i in 0..1000 {
            data[i] = read_sensor(i);
        }
        
        // Process the data
        var report = analyze_readings(data);
        
        // Return the report (it will be copied out of the region)
        return report;
        
        // The entire region is freed at once when the block exits
    }
}
```

## Concurrency and Parallelism

ASTRA provides safe and efficient concurrency primitives designed for real-time systems.

### Tasks

Tasks are lightweight concurrent units of execution, similar to goroutines in Go or async functions in other languages.

```astra
task read_sensors() {
    while true {
        var readings = collect_sensor_data();
        sensor_channel.send(readings);
        time.sleep(10ms);
    }
}

task process_sensor_data() {
    while true {
        var readings = sensor_channel.receive();
        var processed = process_readings(readings);
        control_channel.send(processed);
    }
}

task control_actuators() {
    while true {
        var control_data = control_channel.receive();
        update_actuators(control_data);
    }
}

func main() {
    // Start the tasks
    spawn read_sensors();
    spawn process_sensor_data();
    spawn control_actuators();
    
    // Wait for termination signal
    wait_for_shutdown();
}
```

### Channels

Channels provide a safe way for tasks to communicate without shared memory.

```astra
// Create typed channels
var sensor_channel = Channel<SensorData>(10);  // Buffer size of 10
var control_channel = Channel<ControlData>(5);  // Buffer size of 5

// Send data on a channel
sensor_channel.send(sensor_data);

// Receive data from a channel (blocks if empty)
var data = sensor_channel.receive();

// Try to receive without blocking
var result = sensor_channel.try_receive();
if result.has_value() {
    var data = result.value();
    // Process data
}

// Send with timeout
var success = control_channel.send_timeout(control_data, 5ms);

// Receive with timeout
var result = sensor_channel.receive_timeout(10ms);
if result.has_value() {
    var data = result.value();
    // Process data
}
```

### Synchronization Primitives

ASTRA provides various synchronization primitives for more complex concurrency patterns.

```astra
// Mutex for mutual exclusion
var state_mutex = Mutex();

func update_state(new_value: int) -> void {
    var lock = state_mutex.lock();  // Automatically released when lock goes out of scope
    shared_state = new_value;
}

// Semaphore for resource counting
var resource_semaphore = Semaphore(5);  // 5 available resources

func use_resource() -> void {
    resource_semaphore.acquire();  // Blocks if no resources available
    try {
        // Use the resource
    } finally {
        resource_semaphore.release();  // Always release the resource
    }
}

// Read-write lock for reader-writer pattern
var data_lock = RWLock();

func read_data() -> int {
    var read_lock = data_lock.read_lock();  // Multiple readers can acquire simultaneously
    return shared_data;
}

func write_data(new_value: int) -> void {
    var write_lock = data_lock.write_lock();  // Exclusive access
    shared_data = new_value;
}
```

## Real-Time Features

ASTRA includes features specifically designed for real-time systems.

### Priority-Based Scheduling

Tasks can be assigned priorities to ensure critical operations are executed in a timely manner.

```astra
// Create a high-priority task
@priority(10)  // Higher number means higher priority
task control_loop() {
    while true {
        // High-priority control code
        time.sleep(1ms);
    }
}

// Create a medium-priority task
@priority(5)
task telemetry() {
    while true {
        // Medium-priority telemetry code
        time.sleep(10ms);
    }
}

// Create a low-priority task
@priority(1)
task logging() {
    while true {
        // Low-priority logging code
        time.sleep(100ms);
    }
}
```

### Deadline Scheduling

For hard real-time systems, ASTRA supports deadline-based scheduling.

```astra
// This task must run every 10ms
@periodic(10ms)
task attitude_control() {
    // Control code
}

// This task must complete within 5ms
@deadline(5ms)
task sensor_fusion() {
    // Sensor fusion code
}

// This task runs at a specific time
@scheduled("2023-05-01T12:00:00Z")
task mission_start() {
    // Mission initialization code
}
```

### Time Management

ASTRA provides precise time management functions for real-time operations.

```astra
// Get current time
var now = time.now();

// Sleep for a specific duration
time.sleep(5ms);

// Wait until a specific time
time.sleep_until(mission_start_time);

// Create a periodic timer
var timer = time.create_periodic_timer(10ms);
while true {
    timer.wait_for_next_tick();
    // This code runs every 10ms
}

// Measure execution time
var start = time.now();
perform_operation();
var duration = time.now() - start;
log("Operation took " + duration.to_string());
```

## Domain-Specific Features

ASTRA includes features specifically designed for aerospace and UAV applications.

### Units of Measurement

ASTRA has built-in support for units of measurement to prevent errors like the Mars Climate Orbiter failure.

```astra
// Define variables with units
var altitude: float<meters> = 1000.0;
var speed: float<meters/second> = 25.0;
var acceleration: float<meters/second^2> = 9.81;
var mass: float<kilograms> = 2.5;
var force: float<newtons> = mass * acceleration;  // Units are automatically derived

// Conversion between units
var altitude_feet: float<feet> = altitude.convert_to<feet>();
var speed_mph: float<miles/hour> = speed.convert_to<miles/hour>();

// Compile-time unit checking
var time: float<seconds> = altitude / speed;  // OK: meters / (meters/second) = seconds
var error = altitude + speed;  // Compile error: cannot add meters and meters/second
```

### Coordinate Systems

ASTRA provides built-in support for various coordinate systems used in aerospace.

```astra
// Define positions in different coordinate systems
var position_lla = LLA(37.7749, -122.4194, 10.0);  // Latitude, Longitude, Altitude
var position_ecef = ECEF(1234567.0, 2345678.0, 3456789.0);  // Earth-Centered, Earth-Fixed
var position_enu = ENU(100.0, 200.0, 50.0);  // East, North, Up

// Convert between coordinate systems
var ecef_position = position_lla.to_ecef();
var enu_position = position_ecef.to_enu(reference_lla);
var lla_position = position_enu.to_lla(reference_lla);

// Calculate distances
var distance = position_lla.distance_to(another_lla);
var ground_distance = position_lla.ground_distance_to(another_lla);
```

### Flight Dynamics

ASTRA includes types and functions for flight dynamics calculations.

```astra
// Define attitude using quaternions
var attitude = Quaternion.from_euler(roll, pitch, yaw);

// Define angular velocity
var angular_velocity = Vector3(p, q, r);  // Roll, pitch, yaw rates

// Update attitude based on angular velocity
var dt = 0.01;  // 10ms
attitude = attitude.integrate(angular_velocity, dt);

// Convert to Euler angles
var euler = attitude.to_euler();
var roll = euler.x;
var pitch = euler.y;
var yaw = euler.z;

// Calculate direction cosine matrix
var dcm = attitude.to_dcm();

// Rotate a vector from body to inertial frame
var body_vector = Vector3(1.0, 0.0, 0.0);  // X-axis in body frame
var inertial_vector = attitude.rotate_vector(body_vector);
```

## Interoperability

ASTRA is designed to interoperate with existing systems and languages.

### Foreign Function Interface (FFI)

ASTRA can call functions from C, C++, and other languages.

```astra
// Import C functions
@extern("libsensor.so")
func read_sensor_c(sensor_id: int) -> float;

// Import C++ functions
@extern("libcontrol.so", "C++")
func update_control_cpp(input: float) -> float;

// Use imported functions
var sensor_value = read_sensor_c(1);
var control_output = update_control_cpp(sensor_value);
```

### Embedded Assembly

For performance-critical sections or hardware access, ASTRA allows inline assembly.

```astra
func fast_inverse_sqrt(x: float) -> float {
    var result: float;
    
    asm {
        // Famous Quake III inverse square root
        movss xmm0, [x]
        movss xmm1, [x]
        rsqrtss xmm0, xmm0
        mulss xmm1, xmm0
        mulss xmm1, xmm0
        movss xmm2, [0.5]
        mulss xmm2, xmm0
        movss xmm3, [1.5]
        subss xmm3, xmm1
        mulss xmm2, xmm3
        movss [result], xmm2
    }
    
    return result;
}
```

### Hardware Access

ASTRA provides safe abstractions for direct hardware access.

```astra
// Memory-mapped I/O
var gpio_register = mmio<uint32>(0x40020000);
gpio_register |= (1 << 5);  // Set bit 5

// Direct port I/O (x86)
@platform("x86")
func read_port(port: uint16) -> uint8 {
    var value: uint8;
    asm {
        mov dx, [port]
        in al, dx
        mov [value], al
    }
    return value;
}

// Interrupt handlers
@interrupt(5)
func timer_interrupt() {
    // Handle timer interrupt
}
```

## Metaprogramming

ASTRA supports metaprogramming for generating code at compile time.

### Compile-Time Function Execution

```astra
// Compile-time function
constexpr func factorial(n: int) -> int {
    if n <= 1 {
        return 1;
    } else {
        return n * factorial(n - 1);
    }
}

// Use in constant expressions
const LOOKUP_TABLE_SIZE = factorial(5);  // Computed at compile time
```

### Code Generation

```astra
// Generate vector operations for different dimensions
@generate("N in [2, 3, 4]")
struct Vector{N} {
    @generate("i in 0..N")
    var v{i}: float;
    
    @generate("op in ['+', '-', '*', '/']")
    func operator{op}(other: Vector{N}) -> Vector{N} {
        var result = Vector{N}();
        @generate("i in 0..N")
        result.v{i} = this.v{i} {op} other.v{i};
        return result;
    }
}

// This generates Vector2, Vector3, and Vector4 structs
// with appropriate operators
```

### Reflection

ASTRA provides compile-time reflection capabilities.

```astra
// Get type information at compile time
constexpr func type_info<T>() -> TypeInfo {
    return reflect<T>();
}

// Use reflection to generate serialization code
@serialize
struct TelemetryData {
    var timestamp: int64;
    var position: Vector3;
    var velocity: Vector3;
    var attitude: Quaternion;
}

// The @serialize annotation generates code like:
func serialize(data: TelemetryData) -> array<byte> {
    var result = array<byte>[];
    result.append_all(serialize(data.timestamp));
    result.append_all(serialize(data.position));
    result.append_all(serialize(data.velocity));
    result.append_all(serialize(data.attitude));
    return result;
}

func deserialize(bytes: array<byte>) -> TelemetryData {
    var data = TelemetryData();
    var offset = 0;
    
    data.timestamp = deserialize<int64>(bytes, offset);
    offset += sizeof<int64>();
    
    data.position = deserialize<Vector3>(bytes, offset);
    offset += sizeof<Vector3>();
    
    data.velocity = deserialize<Vector3>(bytes, offset);
    offset += sizeof<Vector3>();
    
    data.attitude = deserialize<Quaternion>(bytes, offset);
    
    return data;
}
```

## Conclusion

ASTRA's advanced features make it uniquely suited for developing reliable, efficient, and safe software for aerospace and UAV applications. By combining modern language features with domain-specific capabilities, ASTRA enables developers to write code that is both expressive and robust.

For more information on using these features, refer to the specific documentation sections or the API reference.