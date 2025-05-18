# ASTRA Hardware Integration Guide

This document provides a comprehensive guide to integrating ASTRA with various hardware platforms commonly used in aerospace and UAV applications.

## Table of Contents

1. [Introduction](#introduction)
2. [Supported Hardware Platforms](#supported-hardware-platforms)
3. [Hardware Abstraction Layer](#hardware-abstraction-layer)
4. [Direct Hardware Access](#direct-hardware-access)
5. [Sensor Integration](#sensor-integration)
6. [Actuator Control](#actuator-control)
7. [Communication Interfaces](#communication-interfaces)
8. [Real-Time Operating Systems](#real-time-operating-systems)
9. [Embedded Assembly](#embedded-assembly)
10. [Driver Development](#driver-development)
11. [Hardware-in-the-Loop Testing](#hardware-in-the-loop-testing)
12. [Deployment and Flashing](#deployment-and-flashing)

## Introduction

ASTRA is designed to work closely with hardware while maintaining safety and reliability. This guide explains how to integrate ASTRA code with various hardware platforms, sensors, actuators, and communication interfaces commonly used in aerospace and UAV applications.

The language provides several mechanisms for hardware interaction:

1. **Hardware Abstraction Layer (HAL)**: A high-level, platform-independent API for common hardware components.
2. **Direct Hardware Access**: Low-level access to memory-mapped I/O, ports, and registers.
3. **Driver Interface**: A standardized way to develop and use hardware drivers.
4. **Embedded Assembly**: The ability to include platform-specific assembly code when needed.

## Supported Hardware Platforms

ASTRA officially supports the following hardware platforms:

### Microcontrollers

- **ARM Cortex-M Series**
  - STM32F4/F7/H7 (STMicroelectronics)
  - SAMD51/SAME5x (Microchip)
  - NXP i.MX RT Series
  - TI TMS570 (Safety-critical applications)

- **RISC-V**
  - SiFive FE310
  - GD32V Series
  - ESP32-C3/C6

- **Other Architectures**
  - ESP32 (Xtensa)
  - AVR (Limited support)
  - MSP430 (Limited support)

### Single-Board Computers

- **ARM-based**
  - Raspberry Pi (3/4/5)
  - NVIDIA Jetson (Nano/Xavier/Orin)
  - BeagleBone Black/Blue
  - NXP i.MX8

- **x86-based**
  - Intel NUC
  - UP Board
  - AAEON PICO

### Flight Controllers

- Pixhawk Series (PX4)
- ArduPilot Compatible Hardware
- CUAV V5+/X7/X7 Pro
- Holybro Durandal/Kakute
- BetaFlight/INAV Compatible FC

### Custom ASTRA Hardware

- ASTRA Reference Platform (ARP-1)
- ASTRA Safety-Critical Controller (ASCC)
- ASTRA Sensor Fusion Module (ASFM)

## Hardware Abstraction Layer

The ASTRA Hardware Abstraction Layer (HAL) provides a consistent API for interacting with hardware across different platforms.

### HAL Architecture

```
+-------------------+
| Application Code  |
+-------------------+
          |
+-------------------+
|   ASTRA HAL API   |
+-------------------+
          |
+-------------------+
| Platform-Specific |
|  Implementation   |
+-------------------+
          |
+-------------------+
|    Hardware       |
+-------------------+
```

### Using the HAL

```astra
import hardware.gpio;
import hardware.i2c;
import hardware.spi;
import hardware.uart;

func main() {
    // Configure GPIO pin as output
    var led_pin = gpio.Pin(gpio.PORT_A, 5, gpio.OUTPUT);
    
    // Configure I2C interface
    var i2c_bus = i2c.Bus(i2c.I2C1, 400000);  // I2C1 at 400kHz
    
    // Configure SPI interface
    var spi_bus = spi.Bus(spi.SPI1, 1000000, spi.MODE_0);  // SPI1 at 1MHz, mode 0
    
    // Configure UART interface
    var uart_port = uart.Port(uart.UART2, 115200);  // UART2 at 115200 baud
    
    // Use GPIO
    led_pin.write(gpio.HIGH);
    time.sleep(500ms);
    led_pin.write(gpio.LOW);
    
    // Use I2C to communicate with a sensor
    var sensor_addr = 0x68;
    var reg_addr = 0x3B;
    var data = i2c_bus.read_registers(sensor_addr, reg_addr, 6);
    
    // Use SPI to communicate with a device
    var cs_pin = gpio.Pin(gpio.PORT_A, 4, gpio.OUTPUT);
    cs_pin.write(gpio.LOW);  // Assert chip select
    var spi_data = spi_bus.transfer(array<byte>[0x01, 0x02, 0x03]);
    cs_pin.write(gpio.HIGH);  // Deassert chip select
    
    // Use UART to send data
    uart_port.write("Hello, ASTRA!\n");
    
    // Read from UART with timeout
    var response = uart_port.read_line(1000ms);  // 1 second timeout
}
```

### Platform-Specific Configuration

Each platform may require specific configuration. This is handled through platform configuration files:

```astra
// platform_config.astra
@platform("stm32f4")
func configure_platform() {
    // STM32F4-specific configuration
    hardware.clock.set_system_clock(168000000);  // 168 MHz
    hardware.clock.enable_peripheral(hardware.clock.GPIOA);
    hardware.clock.enable_peripheral(hardware.clock.I2C1);
    hardware.clock.enable_peripheral(hardware.clock.SPI1);
    hardware.clock.enable_peripheral(hardware.clock.USART2);
}

@platform("rpi4")
func configure_platform() {
    // Raspberry Pi 4-specific configuration
    hardware.gpio.use_bcm_numbering();
    hardware.i2c.enable(1);  // Enable I2C1
    hardware.spi.enable(0);  // Enable SPI0
    hardware.uart.enable(0); // Enable UART0
}
```

## Direct Hardware Access

For cases where the HAL doesn't provide the necessary functionality, ASTRA allows direct hardware access.

### Memory-Mapped I/O

```astra
// Define register addresses
const GPIO_BASE: uint32 = 0x40020000;
const GPIO_MODER: uint32 = GPIO_BASE + 0x00;
const GPIO_ODR: uint32 = GPIO_BASE + 0x14;

func configure_gpio() {
    // Create memory-mapped register
    var moder_reg = mmio<uint32>(GPIO_MODER);
    var odr_reg = mmio<uint32>(GPIO_ODR);
    
    // Configure PA5 as output (bits 10-11 = 01)
    moder_reg = (moder_reg & ~(0x3 << 10)) | (0x1 << 10);
    
    // Set PA5 high (bit 5 = 1)
    odr_reg |= (1 << 5);
    
    // Clear PA5 (bit 5 = 0)
    odr_reg &= ~(1 << 5);
}
```

### Port I/O (x86 platforms)

```astra
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

@platform("x86")
func write_port(port: uint16, value: uint8) {
    asm {
        mov dx, [port]
        mov al, [value]
        out dx, al
    }
}
```

### Interrupt Handlers

```astra
// Define interrupt handler for EXTI Line 0
@interrupt(EXTI0_IRQn)
func exti0_handler() {
    // Handle interrupt
    gpio.toggle(gpio.PORT_A, 5);
    
    // Clear interrupt flag
    var exti_pr = mmio<uint32>(EXTI_BASE + 0x14);
    exti_pr = (1 << 0);  // Write 1 to clear
}

// Enable interrupt
func enable_button_interrupt() {
    // Configure PA0 as input with interrupt
    gpio.configure(gpio.PORT_A, 0, gpio.INPUT_PULLUP);
    
    // Configure EXTI Line 0 to trigger on falling edge
    var exti_ftsr = mmio<uint32>(EXTI_BASE + 0x0C);
    exti_ftsr |= (1 << 0);
    
    // Enable EXTI Line 0 interrupt
    var exti_imr = mmio<uint32>(EXTI_BASE + 0x00);
    exti_imr |= (1 << 0);
    
    // Enable EXTI0 interrupt in NVIC
    nvic.enable_interrupt(EXTI0_IRQn);
}
```

## Sensor Integration

ASTRA provides a standardized way to integrate various sensors commonly used in aerospace and UAV applications.

### Sensor Interface

```astra
// Generic sensor interface
interface Sensor<T> {
    func initialize() -> bool;
    func read() -> T;
    func calibrate() -> bool;
    func self_test() -> bool;
}

// Specific sensor implementations
class IMU implements Sensor<IMUData> {
    private i2c_bus: i2c.Bus;
    private address: uint8;
    
    public func constructor(bus: i2c.Bus, addr: uint8 = 0x68) {
        this.i2c_bus = bus;
        this.address = addr;
    }
    
    public func initialize() -> bool {
        // Initialize IMU
        // Reset device
        var result = this.i2c_bus.write_register(this.address, 0x6B, 0x80);
        time.sleep(100ms);
        
        // Wake up device
        result = result && this.i2c_bus.write_register(this.address, 0x6B, 0x00);
        
        // Configure gyroscope and accelerometer
        result = result && this.i2c_bus.write_register(this.address, 0x1B, 0x08);  // Gyro ±500°/s
        result = result && this.i2c_bus.write_register(this.address, 0x1C, 0x08);  // Accel ±4g
        
        return result;
    }
    
    public func read() -> IMUData {
        var data = IMUData();
        
        // Read accelerometer data
        var accel_bytes = this.i2c_bus.read_registers(this.address, 0x3B, 6);
        data.accel_x = (accel_bytes[0] << 8 | accel_bytes[1]) as int16 as float / 8192.0;  // ±4g scale
        data.accel_y = (accel_bytes[2] << 8 | accel_bytes[3]) as int16 as float / 8192.0;
        data.accel_z = (accel_bytes[4] << 8 | accel_bytes[5]) as int16 as float / 8192.0;
        
        // Read gyroscope data
        var gyro_bytes = this.i2c_bus.read_registers(this.address, 0x43, 6);
        data.gyro_x = (gyro_bytes[0] << 8 | gyro_bytes[1]) as int16 as float / 65.5;  // ±500°/s scale
        data.gyro_y = (gyro_bytes[2] << 8 | gyro_bytes[3]) as int16 as float / 65.5;
        data.gyro_z = (gyro_bytes[4] << 8 | gyro_bytes[5]) as int16 as float / 65.5;
        
        return data;
    }
    
    public func calibrate() -> bool {
        // Calibration implementation
        // ...
        return true;
    }
    
    public func self_test() -> bool {
        // Self-test implementation
        // ...
        return true;
    }
}
```

### Sensor Fusion

```astra
class SensorFusion {
    private imu: IMU;
    private barometer: Barometer;
    private gps: GPS;
    private magnetometer: Magnetometer;
    
    private attitude: Quaternion;
    private position: Vector3;
    private velocity: Vector3;
    
    // Kalman filter state
    private state_estimate: Matrix;
    private covariance: Matrix;
    
    public func constructor(imu: IMU, baro: Barometer, gps: GPS, mag: Magnetometer) {
        this.imu = imu;
        this.barometer = baro;
        this.gps = gps;
        this.magnetometer = mag;
        
        this.attitude = Quaternion.identity();
        this.position = Vector3(0.0, 0.0, 0.0);
        this.velocity = Vector3(0.0, 0.0, 0.0);
        
        // Initialize Kalman filter
        this.initialize_kalman_filter();
    }
    
    public func update(dt: float) -> void {
        // Read sensor data
        var imu_data = this.imu.read();
        var baro_data = this.barometer.read();
        var gps_data = this.gps.read();
        var mag_data = this.magnetometer.read();
        
        // Predict step (using IMU data)
        this.predict(imu_data, dt);
        
        // Update step (using absolute sensors)
        this.update_attitude(mag_data);
        this.update_position(gps_data, baro_data);
    }
    
    private func predict(imu_data: IMUData, dt: float) -> void {
        // Attitude prediction using gyroscope
        var gyro = Vector3(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
        this.attitude = this.attitude.integrate(gyro, dt);
        
        // Velocity prediction using accelerometer
        var accel_body = Vector3(imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        var accel_earth = this.attitude.rotate_vector(accel_body);
        accel_earth.z -= 9.81;  // Remove gravity
        
        this.velocity += accel_earth * dt;
        
        // Position prediction using velocity
        this.position += this.velocity * dt;
        
        // Kalman filter prediction step
        // ...
    }
    
    private func update_attitude(mag_data: MagnetometerData) -> void {
        // Update attitude using magnetometer
        // ...
    }
    
    private func update_position(gps_data: GPSData, baro_data: BarometerData) -> void {
        // Update position using GPS and barometer
        // ...
    }
    
    private func initialize_kalman_filter() -> void {
        // Initialize Kalman filter state and covariance
        // ...
    }
    
    public func get_attitude() -> Quaternion {
        return this.attitude;
    }
    
    public func get_position() -> Vector3 {
        return this.position;
    }
    
    public func get_velocity() -> Vector3 {
        return this.velocity;
    }
}
```

## Actuator Control

ASTRA provides interfaces for controlling various actuators used in aerospace and UAV applications.

### Motor Control

```astra
// PWM-based motor controller
class MotorController {
    private pwm_pins: array<pwm.Pin>;
    private min_pulse_width: float;
    private max_pulse_width: float;
    private motor_values: array<float>;
    
    public func constructor(pins: array<pwm.Pin>, min_pulse: float = 1000.0, max_pulse: float = 2000.0) {
        this.pwm_pins = pins;
        this.min_pulse_width = min_pulse;
        this.max_pulse_width = max_pulse;
        this.motor_values = array<float>[pins.length];
        
        // Initialize all motors to zero
        for (var i = 0; i < pins.length; i++) {
            this.motor_values[i] = 0.0;
            this.pwm_pins[i].set_period(20000);  // 20ms period (50Hz)
            this.pwm_pins[i].set_pulse_width(this.min_pulse_width);
        }
    }
    
    public func set_motor(index: int, value: float) -> bool {
        if (index < 0 || index >= this.pwm_pins.length) {
            return false;
        }
        
        // Clamp value between 0.0 and 1.0
        var clamped_value = Math.clamp(value, 0.0, 1.0);
        this.motor_values[index] = clamped_value;
        
        // Convert to pulse width
        var pulse_width = this.min_pulse_width + clamped_value * (this.max_pulse_width - this.min_pulse_width);
        
        // Set PWM pulse width
        this.pwm_pins[index].set_pulse_width(pulse_width);
        
        return true;
    }
    
    public func set_all_motors(values: array<float>) -> bool {
        if (values.length != this.pwm_pins.length) {
            return false;
        }
        
        for (var i = 0; i < values.length; i++) {
            this.set_motor(i, values[i]);
        }
        
        return true;
    }
    
    public func arm() -> void {
        // Send minimum pulse width to all motors
        for (var i = 0; i < this.pwm_pins.length; i++) {
            this.pwm_pins[i].set_pulse_width(this.min_pulse_width);
        }
        
        // Wait for ESCs to initialize
        time.sleep(3s);
    }
    
    public func disarm() -> void {
        // Send minimum pulse width to all motors
        for (var i = 0; i < this.pwm_pins.length; i++) {
            this.pwm_pins[i].set_pulse_width(this.min_pulse_width);
        }
    }
}
```

### Servo Control

```astra
class ServoController {
    private pwm_pin: pwm.Pin;
    private min_angle: float;
    private max_angle: float;
    private min_pulse_width: float;
    private max_pulse_width: float;
    private current_angle: float;
    
    public func constructor(pin: pwm.Pin, min_angle: float = 0.0, max_angle: float = 180.0,
                           min_pulse: float = 1000.0, max_pulse: float = 2000.0) {
        this.pwm_pin = pin;
        this.min_angle = min_angle;
        this.max_angle = max_angle;
        this.min_pulse_width = min_pulse;
        this.max_pulse_width = max_pulse;
        this.current_angle = (min_angle + max_angle) / 2.0;
        
        // Initialize PWM
        this.pwm_pin.set_period(20000);  // 20ms period (50Hz)
        this.set_angle(this.current_angle);
    }
    
    public func set_angle(angle: float) -> bool {
        // Clamp angle to valid range
        var clamped_angle = Math.clamp(angle, this.min_angle, this.max_angle);
        this.current_angle = clamped_angle;
        
        // Convert angle to pulse width
        var normalized_angle = (clamped_angle - this.min_angle) / (this.max_angle - this.min_angle);
        var pulse_width = this.min_pulse_width + normalized_angle * (this.max_pulse_width - this.min_pulse_width);
        
        // Set PWM pulse width
        this.pwm_pin.set_pulse_width(pulse_width);
        
        return true;
    }
    
    public func get_angle() -> float {
        return this.current_angle;
    }
}
```

## Communication Interfaces

ASTRA provides interfaces for various communication protocols used in aerospace and UAV applications.

### MAVLink

```astra
import comm.mavlink;

class MAVLinkInterface {
    private uart: uart.Port;
    private system_id: uint8;
    private component_id: uint8;
    
    public func constructor(uart_port: uart.Port, sys_id: uint8 = 1, comp_id: uint8 = 1) {
        this.uart = uart_port;
        this.system_id = sys_id;
        this.component_id = comp_id;
    }
    
    public func send_heartbeat() -> void {
        var msg = mavlink.Message();
        msg.id = mavlink.MSG_ID_HEARTBEAT;
        msg.system_id = this.system_id;
        msg.component_id = this.component_id;
        
        // Set heartbeat fields
        msg.payload.heartbeat.type = mavlink.MAV_TYPE_QUADROTOR;
        msg.payload.heartbeat.autopilot = mavlink.MAV_AUTOPILOT_GENERIC;
        msg.payload.heartbeat.base_mode = mavlink.MAV_MODE_FLAG_SAFETY_ARMED;
        msg.payload.heartbeat.custom_mode = 0;
        msg.payload.heartbeat.system_status = mavlink.MAV_STATE_ACTIVE;
        
        // Pack and send message
        var packet = mavlink.pack_message(msg);
        this.uart.write(packet);
    }
    
    public func send_attitude(roll: float, pitch: float, yaw: float,
                             roll_rate: float, pitch_rate: float, yaw_rate: float) -> void {
        var msg = mavlink.Message();
        msg.id = mavlink.MSG_ID_ATTITUDE;
        msg.system_id = this.system_id;
        msg.component_id = this.component_id;
        
        // Set attitude fields
        msg.payload.attitude.time_boot_ms = time.millis();
        msg.payload.attitude.roll = roll;
        msg.payload.attitude.pitch = pitch;
        msg.payload.attitude.yaw = yaw;
        msg.payload.attitude.rollspeed = roll_rate;
        msg.payload.attitude.pitchspeed = pitch_rate;
        msg.payload.attitude.yawspeed = yaw_rate;
        
        // Pack and send message
        var packet = mavlink.pack_message(msg);
        this.uart.write(packet);
    }
    
    public func receive() -> mavlink.Message? {
        // Check if data is available
        if (this.uart.available() > 0) {
            // Read bytes until a complete message is received
            var byte = this.uart.read_byte();
            var msg = mavlink.parse_byte(byte);
            
            if (msg != null) {
                return msg;
            }
        }
        
        return null;
    }
    
    public func process_messages() -> void {
        while (true) {
            var msg = this.receive();
            if (msg == null) {
                break;
            }
            
            // Process message based on ID
            if (msg.id == mavlink.MSG_ID_COMMAND_LONG) {
                this.handle_command(msg.payload.command_long);
            } else if (msg.id == mavlink.MSG_ID_PARAM_REQUEST_READ) {
                this.handle_param_request(msg.payload.param_request_read);
            }
            // Handle other message types...
        }
    }
    
    private func handle_command(cmd: mavlink.CommandLong) -> void {
        // Handle command
        // ...
    }
    
    private func handle_param_request(req: mavlink.ParamRequestRead) -> void {
        // Handle parameter request
        // ...
    }
}
```

### CAN Bus

```astra
import comm.can;

class CANInterface {
    private can_bus: can.Bus;
    
    public func constructor(bus: can.Bus) {
        this.can_bus = bus;
    }
    
    public func initialize() -> bool {
        return this.can_bus.initialize(can.BAUD_1M);
    }
    
    public func send_message(id: uint32, data: array<byte>) -> bool {
        var msg = can.Message();
        msg.id = id;
        msg.extended = (id > 0x7FF);  // Extended ID if > 11 bits
        msg.rtr = false;  // Not a remote frame
        msg.dlc = data.length;
        msg.data = data;
        
        return this.can_bus.send(msg);
    }
    
    public func receive_message(timeout_ms: uint32 = 100) -> can.Message? {
        return this.can_bus.receive(timeout_ms);
    }
    
    public func set_filter(id: uint32, mask: uint32) -> bool {
        return this.can_bus.set_filter(0, id, mask);
    }
}
```

## Real-Time Operating Systems

ASTRA can be integrated with various Real-Time Operating Systems (RTOS) for time-critical applications.

### FreeRTOS Integration

```astra
import rtos.freertos;

// Task function
func control_task(params: any) -> void {
    while (true) {
        // Read sensors
        var sensor_data = read_sensors();
        
        // Update control algorithm
        var control_output = update_control(sensor_data);
        
        // Set actuators
        set_actuators(control_output);
        
        // Delay until next cycle
        freertos.delay(10);  // 10ms delay
    }
}

func telemetry_task(params: any) -> void {
    while (true) {
        // Collect telemetry data
        var telemetry_data = collect_telemetry();
        
        // Send telemetry
        send_telemetry(telemetry_data);
        
        // Delay until next cycle
        freertos.delay(100);  // 100ms delay
    }
}

func main() {
    // Initialize hardware
    initialize_hardware();
    
    // Create tasks
    var control_task_handle = freertos.create_task(
        control_task,
        "Control",
        1024,  // Stack size
        null,  // Parameters
        4      // Priority (higher number = higher priority)
    );
    
    var telemetry_task_handle = freertos.create_task(
        telemetry_task,
        "Telemetry",
        1024,  // Stack size
        null,  // Parameters
        2      // Priority
    );
    
    // Start scheduler
    freertos.start_scheduler();
    
    // Should never reach here
    while (true) {}
}
```

### POSIX Threads (for Linux-based platforms)

```astra
import os.posix;

// Thread function
func control_thread(arg: any) -> any {
    while (true) {
        // Read sensors
        var sensor_data = read_sensors();
        
        // Update control algorithm
        var control_output = update_control(sensor_data);
        
        // Set actuators
        set_actuators(control_output);
        
        // Sleep until next cycle
        posix.usleep(10000);  // 10ms sleep
    }
    
    return null;
}

func telemetry_thread(arg: any) -> any {
    while (true) {
        // Collect telemetry data
        var telemetry_data = collect_telemetry();
        
        // Send telemetry
        send_telemetry(telemetry_data);
        
        // Sleep until next cycle
        posix.usleep(100000);  // 100ms sleep
    }
    
    return null;
}

func main() {
    // Initialize hardware
    initialize_hardware();
    
    // Create threads
    var control_thread_id = posix.pthread_t();
    var telemetry_thread_id = posix.pthread_t();
    
    posix.pthread_create(control_thread_id, null, control_thread, null);
    posix.pthread_create(telemetry_thread_id, null, telemetry_thread, null);
    
    // Wait for threads to finish (they won't in this case)
    posix.pthread_join(control_thread_id, null);
    posix.pthread_join(telemetry_thread_id, null);
}
```

## Embedded Assembly

For performance-critical sections or hardware-specific operations, ASTRA allows embedding assembly code.

### ARM Cortex-M Assembly

```astra
@platform("arm-cortex-m4")
func fast_fir_filter(input: array<float>, coeffs: array<float>, output: array<float>, length: int) -> void {
    asm {
        push {r4-r11, lr}
        
        // r0 = input array
        // r1 = coeffs array
        // r2 = output array
        // r3 = length
        
        // Use SIMD instructions for faster processing
        // ...
        
        pop {r4-r11, pc}
    }
}
```

### RISC-V Assembly

```astra
@platform("riscv")
func atomic_add(addr: *int, value: int) -> int {
    var result: int;
    
    asm {
        // a0 = addr
        // a1 = value
        // a0 will contain the return value
        
        // Atomic add operation
        amoadd.w a0, a1, (a0)
        
        // Move result to output variable
        mv %[result], a0
    } : [result] "=r" (result) : [addr] "r" (addr), [value] "r" (value)
    
    return result;
}
```

### x86 Assembly

```astra
@platform("x86_64")
func fast_memcpy(dest: *void, src: *void, size: size_t) -> *void {
    asm {
        mov rdi, [dest]
        mov rsi, [src]
        mov rcx, [size]
        rep movsb
        mov rax, [dest]
    }
    
    return dest;
}
```

## Driver Development

ASTRA provides a standardized way to develop hardware drivers.

### Driver Interface

```astra
// Generic driver interface
interface Driver {
    func initialize() -> bool;
    func deinitialize() -> bool;
    func is_initialized() -> bool;
}

// Device driver interface
interface DeviceDriver extends Driver {
    func read(buffer: *void, size: size_t) -> int;
    func write(buffer: *void, size: size_t) -> int;
    func ioctl(command: uint32, arg: *void) -> int;
}
```

### Example: MPU6050 IMU Driver

```astra
class MPU6050Driver implements DeviceDriver {
    private i2c_bus: i2c.Bus;
    private address: uint8;
    private initialized: bool;
    
    // MPU6050 registers
    private const REG_PWR_MGMT_1: uint8 = 0x6B;
    private const REG_GYRO_CONFIG: uint8 = 0x1B;
    private const REG_ACCEL_CONFIG: uint8 = 0x1C;
    private const REG_ACCEL_XOUT_H: uint8 = 0x3B;
    private const REG_GYRO_XOUT_H: uint8 = 0x43;
    
    public func constructor(bus: i2c.Bus, addr: uint8 = 0x68) {
        this.i2c_bus = bus;
        this.address = addr;
        this.initialized = false;
    }
    
    public func initialize() -> bool {
        if (this.initialized) {
            return true;
        }
        
        // Reset device
        var result = this.i2c_bus.write_register(this.address, this.REG_PWR_MGMT_1, 0x80);
        time.sleep(100ms);
        
        // Wake up device
        result = result && this.i2c_bus.write_register(this.address, this.REG_PWR_MGMT_1, 0x00);
        
        // Configure gyroscope (±500°/s)
        result = result && this.i2c_bus.write_register(this.address, this.REG_GYRO_CONFIG, 0x08);
        
        // Configure accelerometer (±4g)
        result = result && this.i2c_bus.write_register(this.address, this.REG_ACCEL_CONFIG, 0x08);
        
        this.initialized = result;
        return result;
    }
    
    public func deinitialize() -> bool {
        if (!this.initialized) {
            return true;
        }
        
        // Put device in sleep mode
        var result = this.i2c_bus.write_register(this.address, this.REG_PWR_MGMT_1, 0x40);
        
        this.initialized = !result;
        return result;
    }
    
    public func is_initialized() -> bool {
        return this.initialized;
    }
    
    public func read(buffer: *void, size: size_t) -> int {
        if (!this.initialized || buffer == null || size < 14) {
            return -1;
        }
        
        // Read accelerometer and gyroscope data (14 bytes total)
        var data = this.i2c_bus.read_registers(this.address, this.REG_ACCEL_XOUT_H, 14);
        
        if (data.length != 14) {
            return -1;
        }
        
        // Copy data to buffer
        memory.copy(buffer, data.data(), 14);
        
        return 14;
    }
    
    public func write(buffer: *void, size: size_t) -> int {
        // MPU6050 doesn't support bulk writes in this manner
        return -1;
    }
    
    public func ioctl(command: uint32, arg: *void) -> int {
        if (!this.initialized) {
            return -1;
        }
        
        // Handle various control commands
        if (command == IOCTL_MPU6050_SET_GYRO_RANGE) {
            var range = *(arg as *uint8);
            return this.set_gyro_range(range) ? 0 : -1;
        } else if (command == IOCTL_MPU6050_SET_ACCEL_RANGE) {
            var range = *(arg as *uint8);
            return this.set_accel_range(range) ? 0 : -1;
        } else if (command == IOCTL_MPU6050_RESET) {
            return this.reset() ? 0 : -1;
        }
        
        return -1;
    }
    
    // Helper methods
    private func set_gyro_range(range: uint8) -> bool {
        return this.i2c_bus.write_register(this.address, this.REG_GYRO_CONFIG, range & 0x18);
    }
    
    private func set_accel_range(range: uint8) -> bool {
        return this.i2c_bus.write_register(this.address, this.REG_ACCEL_CONFIG, range & 0x18);
    }
    
    private func reset() -> bool {
        var result = this.i2c_bus.write_register(this.address, this.REG_PWR_MGMT_1, 0x80);
        time.sleep(100ms);
        result = result && this.i2c_bus.write_register(this.address, this.REG_PWR_MGMT_1, 0x00);
        return result;
    }
}
```

## Hardware-in-the-Loop Testing

ASTRA provides support for Hardware-in-the-Loop (HIL) testing to validate software against simulated or real hardware.

### HIL Simulation Interface

```astra
interface HILSimulation {
    func initialize() -> bool;
    func step(dt: float) -> bool;
    func set_inputs(inputs: Map<string, any>) -> bool;
    func get_outputs() -> Map<string, any>;
    func shutdown() -> bool;
}

class QuadcopterHIL implements HILSimulation {
    private sim_socket: network.Socket;
    private initialized: bool;
    
    public func constructor(host: string = "localhost", port: int = 5760) {
        this.sim_socket = network.Socket(network.TCP);
        this.initialized = false;
    }
    
    public func initialize() -> bool {
        if (this.initialized) {
            return true;
        }
        
        // Connect to simulation server
        var result = this.sim_socket.connect(host, port);
        if (!result) {
            return false;
        }
        
        // Send initialization command
        var init_cmd = json.stringify({
            "command": "initialize",
            "vehicle": "quadcopter",
            "parameters": {
                "mass": 1.5,
                "arm_length": 0.25,
                "inertia": [0.02, 0.02, 0.04]
            }
        });
        
        this.sim_socket.send(init_cmd + "\n");
        
        // Read response
        var response = this.sim_socket.receive_line();
        var response_obj = json.parse(response);
        
        this.initialized = response_obj.get("status", "") == "ok";
        return this.initialized;
    }
    
    public func step(dt: float) -> bool {
        if (!this.initialized) {
            return false;
        }
        
        // Send step command
        var step_cmd = json.stringify({
            "command": "step",
            "dt": dt
        });
        
        this.sim_socket.send(step_cmd + "\n");
        
        // Read response
        var response = this.sim_socket.receive_line();
        var response_obj = json.parse(response);
        
        return response_obj.get("status", "") == "ok";
    }
    
    public func set_inputs(inputs: Map<string, any>) -> bool {
        if (!this.initialized) {
            return false;
        }
        
        // Send inputs command
        var inputs_cmd = json.stringify({
            "command": "set_inputs",
            "inputs": inputs
        });
        
        this.sim_socket.send(inputs_cmd + "\n");
        
        // Read response
        var response = this.sim_socket.receive_line();
        var response_obj = json.parse(response);
        
        return response_obj.get("status", "") == "ok";
    }
    
    public func get_outputs() -> Map<string, any> {
        if (!this.initialized) {
            return Map<string, any>();
        }
        
        // Send get_outputs command
        var outputs_cmd = json.stringify({
            "command": "get_outputs"
        });
        
        this.sim_socket.send(outputs_cmd + "\n");
        
        // Read response
        var response = this.sim_socket.receive_line();
        var response_obj = json.parse(response);
        
        if (response_obj.get("status", "") != "ok") {
            return Map<string, any>();
        }
        
        return response_obj.get("outputs", Map<string, any>());
    }
    
    public func shutdown() -> bool {
        if (!this.initialized) {
            return true;
        }
        
        // Send shutdown command
        var shutdown_cmd = json.stringify({
            "command": "shutdown"
        });
        
        this.sim_socket.send(shutdown_cmd + "\n");
        
        // Read response
        var response = this.sim_socket.receive_line();
        var response_obj = json.parse(response);
        
        this.sim_socket.close();
        this.initialized = false;
        
        return response_obj.get("status", "") == "ok";
    }
}
```

### HIL Test Example

```astra
func test_quadcopter_controller() -> bool {
    // Create HIL simulation
    var hil = QuadcopterHIL();
    if (!hil.initialize()) {
        io.println("Failed to initialize HIL simulation");
        return false;
    }
    
    // Create controller
    var controller = QuadcopterController();
    controller.initialize();
    
    // Set initial setpoint
    controller.set_position_setpoint(Vector3(0.0, 0.0, 1.0));  // 1m altitude
    
    // Run simulation for 10 seconds
    var dt = 0.01;  // 10ms
    var sim_time = 0.0;
    var max_sim_time = 10.0;
    
    while (sim_time < max_sim_time) {
        // Get sensor data from simulation
        var outputs = hil.get_outputs();
        var position = Vector3(
            outputs.get("position_x", 0.0) as float,
            outputs.get("position_y", 0.0) as float,
            outputs.get("position_z", 0.0) as float
        );
        var velocity = Vector3(
            outputs.get("velocity_x", 0.0) as float,
            outputs.get("velocity_y", 0.0) as float,
            outputs.get("velocity_z", 0.0) as float
        );
        var attitude = Quaternion(
            outputs.get("attitude_w", 1.0) as float,
            outputs.get("attitude_x", 0.0) as float,
            outputs.get("attitude_y", 0.0) as float,
            outputs.get("attitude_z", 0.0) as float
        );
        var angular_velocity = Vector3(
            outputs.get("angular_velocity_x", 0.0) as float,
            outputs.get("angular_velocity_y", 0.0) as float,
            outputs.get("angular_velocity_z", 0.0) as float
        );
        
        // Update controller
        controller.update(position, velocity, attitude, angular_velocity, dt);
        
        // Get motor commands from controller
        var motor_commands = controller.get_motor_commands();
        
        // Send motor commands to simulation
        var inputs = Map<string, any>();
        inputs.set("motor1", motor_commands[0]);
        inputs.set("motor2", motor_commands[1]);
        inputs.set("motor3", motor_commands[2]);
        inputs.set("motor4", motor_commands[3]);
        
        hil.set_inputs(inputs);
        
        // Step simulation
        hil.step(dt);
        
        // Update simulation time
        sim_time += dt;
        
        // Log data every 100ms
        if (Math.fmod(sim_time, 0.1) < dt) {
            io.println("Time: " + sim_time.toString() + "s, Altitude: " + position.z.toString() + "m");
        }
    }
    
    // Check final position
    var final_outputs = hil.get_outputs();
    var final_position = Vector3(
        final_outputs.get("position_x", 0.0) as float,
        final_outputs.get("position_y", 0.0) as float,
        final_outputs.get("position_z", 0.0) as float
    );
    
    // Shutdown simulation
    hil.shutdown();
    
    // Check if altitude is within 5% of setpoint
    var altitude_error = Math.abs(final_position.z - 1.0);
    var success = altitude_error < 0.05;
    
    io.println("Test " + (success ? "passed" : "failed") + ": Final altitude error = " + altitude_error.toString() + "m");
    
    return success;
}
```

## Deployment and Flashing

ASTRA provides tools for deploying and flashing code to target hardware.

### Deployment Configuration

```astra
// deployment.astra
@deployment("stm32f4")
func configure_deployment() -> DeploymentConfig {
    var config = DeploymentConfig();
    
    // Set target device
    config.target = "STM32F407VG";
    
    // Set flash options
    config.flash_address = 0x08000000;
    config.flash_size = 1024 * 1024;  // 1MB
    
    // Set linker script
    config.linker_script = "stm32f4_flash.ld";
    
    // Set compiler options
    config.compiler_options = [
        "-mcpu=cortex-m4",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-O2"
    ];
    
    // Set build artifacts
    config.output_formats = ["elf", "bin", "hex"];
    
    // Set flash command
    config.flash_command = "st-flash write {output.bin} 0x08000000";
    
    return config;
}

@deployment("rpi4")
func configure_deployment() -> DeploymentConfig {
    var config = DeploymentConfig();
    
    // Set target device
    config.target = "RaspberryPi4";
    
    // Set compiler options
    config.compiler_options = [
        "-march=armv8-a",
        "-mcpu=cortex-a72",
        "-O2"
    ];
    
    // Set build artifacts
    config.output_formats = ["elf"];
    
    // Set deployment command
    config.deploy_command = "scp {output.elf} pi@{target_ip}:/home/pi/";
    
    return config;
}
```

### Flashing Example

```astra
import tools.flash;

func flash_firmware(device: string, firmware_path: string) -> bool {
    // Create flasher for the specified device
    var flasher: flash.Flasher;
    
    if (device == "stm32f4") {
        flasher = flash.STM32Flasher();
    } else if (device == "esp32") {
        flasher = flash.ESP32Flasher();
    } else if (device == "nrf52") {
        flasher = flash.NRF52Flasher();
    } else {
        io.println("Unsupported device: " + device);
        return false;
    }
    
    // Connect to device
    if (!flasher.connect()) {
        io.println("Failed to connect to device");
        return false;
    }
    
    // Erase flash
    io.println("Erasing flash...");
    if (!flasher.erase()) {
        io.println("Failed to erase flash");
        flasher.disconnect();
        return false;
    }
    
    // Flash firmware
    io.println("Flashing firmware...");
    if (!flasher.flash(firmware_path)) {
        io.println("Failed to flash firmware");
        flasher.disconnect();
        return false;
    }
    
    // Verify firmware
    io.println("Verifying firmware...");
    if (!flasher.verify(firmware_path)) {
        io.println("Firmware verification failed");
        flasher.disconnect();
        return false;
    }
    
    // Reset device
    io.println("Resetting device...");
    if (!flasher.reset()) {
        io.println("Failed to reset device");
        flasher.disconnect();
        return false;
    }
    
    // Disconnect
    flasher.disconnect();
    
    io.println("Firmware flashed successfully");
    return true;
}
```

## Conclusion

This guide provides a comprehensive overview of hardware integration with ASTRA. By leveraging the language's hardware abstraction layer, direct hardware access capabilities, and standardized interfaces, developers can create reliable and efficient code for a wide range of aerospace and UAV applications.

For platform-specific details and advanced topics, refer to the following resources:

- [ASTRA Hardware Reference Manual](hardware_reference.md)
- [ASTRA Driver Development Guide](driver_development.md)
- [ASTRA RTOS Integration Guide](rtos_integration.md)
- [ASTRA Deployment and Flashing Guide](deployment_guide.md)