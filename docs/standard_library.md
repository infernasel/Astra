# ASTRA Standard Library

The ASTRA standard library provides a comprehensive set of modules and functions specifically designed for aerospace and UAV applications. This document outlines the major components of the standard library.

## Core Modules

### `core.math`

Basic mathematical operations and functions:

- Arithmetic operations
- Trigonometric functions
- Statistical functions
- Numerical integration and differentiation
- Random number generation

```astra
import core.math;

var angle = Math.radians(45.0);
var sin_value = Math.sin(angle);
var random_value = Math.random();
```

### `core.vector`

Vector operations for 2D, 3D, and 4D vectors:

- Vector creation and manipulation
- Vector arithmetic (addition, subtraction, scaling)
- Dot and cross products
- Normalization and magnitude
- Vector rotations and transformations

```astra
import core.vector;

var position = Vector3(1.0, 2.0, 3.0);
var direction = Vector3(0.0, 0.0, 1.0);
var normalized = direction.normalize();
var distance = position.distance(Vector3(4.0, 5.0, 6.0));
```

### `core.matrix`

Matrix operations for 2x2, 3x3, and 4x4 matrices:

- Matrix creation and manipulation
- Matrix arithmetic
- Determinants and inverses
- Transformations (rotation, scaling, translation)
- Perspective and orthographic projections

```astra
import core.matrix;

var transform = Matrix4.identity();
transform = transform.translate(Vector3(1.0, 0.0, 0.0));
transform = transform.rotate_y(Math.radians(45.0));
var inverse = transform.inverse();
```

### `core.quaternion`

Quaternion operations for 3D rotations:

- Quaternion creation and manipulation
- Conversion to/from Euler angles and matrices
- Quaternion interpolation (SLERP)
- Rotation of vectors
- Composition of rotations

```astra
import core.quaternion;

var rotation = Quaternion.from_euler(0.0, Math.radians(90.0), 0.0);
var vector = Vector3(1.0, 0.0, 0.0);
var rotated = rotation.rotate_vector(vector);
```

### `core.time`

Time handling and representation:

- High-precision time representation
- Time conversions (UTC, TAI, GPS time)
- Time intervals and durations
- Scheduling and timing operations

```astra
import core.time;

var mission_start = time.now();
var elapsed = time.duration_since(mission_start);
```

### `core.io`

Input/output operations:

- File handling
- Data serialization
- Network communication
- Console input/output

```astra
import core.io;

var log_file = io.open("/data/mission.log", "w");
log_file.write_line("Mission started at: " + time.now().to_string());
```

### `core.concurrency`

Concurrency primitives:

- Tasks and threads
- Channels for message passing
- Synchronization primitives
- Atomic operations

```astra
import core.concurrency;

var data_channel = concurrency.channel<telemetry_data>();
task process_telemetry() {
    while true {
        var data = data_channel.receive();
        // Process data
    }
}
```

## Aerospace Modules

### `orbital.mechanics`

Orbital mechanics calculations:

- Orbit determination and propagation
- Orbital elements and conversions
- Maneuver planning
- Perturbation modeling
- Collision avoidance

```astra
import orbital.mechanics;

var earth = CelestialBody("Earth", 5.972e24, 6371.0);
var orbit = Orbit.from_elements(
    earth,
    semi_major_axis: 6778.0, // km
    eccentricity: 0.001,
    inclination: Math.radians(51.6), // ISS inclination
    raan: Math.radians(45.0),
    arg_perigee: Math.radians(30.0),
    true_anomaly: Math.radians(0.0)
);

var propagated_orbit = orbit.propagate(Time.hours(1.5));
var position = orbit.get_position(Time.now());
var velocity = orbit.get_velocity(Time.now());
```

### `orbital.trajectory`

Trajectory design and analysis:

- Trajectory optimization
- Entry, descent, and landing trajectories
- Interplanetary trajectories
- Gravity assists

```astra
import orbital.trajectory;

var transfer = trajectory.hohmann_transfer(
    origin_orbit: earth_orbit,
    destination_orbit: mars_orbit,
    departure_date: time.date(2026, 9, 15)
);
```

### `navigation.guidance`

Guidance algorithms:

- Path planning
- Waypoint navigation
- Obstacle avoidance
- Terrain following

```astra
import navigation.guidance;

var waypoints = [
    vector3(47.123, 8.456, 100.0),
    vector3(47.125, 8.458, 120.0),
    vector3(47.127, 8.457, 100.0)
];

var path = guidance.plan_path(
    start: current_position,
    waypoints: waypoints,
    avoid_obstacles: true
);
```

### `control.systems`

Control systems:

- PID controllers
- Kalman filters
- Attitude controllers
- Trajectory controllers
- Model predictive control

```astra
import control.systems;

// Create a PID controller
var pid = PID(
    kp: 0.5,
    ki: 0.1,
    kd: 0.2
);
pid.set_output_limits(-1.0, 1.0);

// Create an attitude controller
var attitude_controller = AttitudeController(
    roll_pid: PID(4.0, 0.2, 1.0),
    pitch_pid: PID(4.0, 0.2, 1.0),
    yaw_pid: PID(4.0, 0.2, 1.0)
);

// Update the controller
var torque = attitude_controller.update(
    current_attitude,
    angular_velocity,
    target_attitude
);
```

### `propulsion.thrusters`

Propulsion system control:

- Thruster modeling and control
- Fuel management
- Thrust optimization
- Engine performance monitoring

```astra
import propulsion.thrusters;

var main_engine = thrusters.get_engine("MAIN");
main_engine.set_thrust_level(0.75); // 75% thrust

var fuel_remaining = thrusters.get_fuel_level();
var burn_time = thrusters.calculate_burn_time(delta_v);
```

## Sensor and Data Modules

### `sensors.imu`

Inertial Measurement Unit interface:

- Acceleration measurement
- Angular velocity measurement
- Attitude determination
- Sensor fusion

```astra
import sensors.imu;

// Create an IMU with specified accuracies
var imu = IMU(0.01, 0.001, 0.001);  // accel, gyro, mag accuracy

// Update sensor readings
imu.update();

// Get sensor data
var acceleration = imu.getAcceleration();
var angular_velocity = imu.getAngularVelocity();
var orientation = imu.getOrientation();
var magnetic_field = imu.getMagneticField();
```

### `sensors.gps`

Global Positioning System interface:

- Position determination
- Velocity measurement
- Time synchronization
- GPS error modeling

```astra
import sensors.gps;

var position = gps.get_position();
var velocity = gps.get_velocity();
var accuracy = gps.get_position_accuracy();
```

### `sensors.camera`

Camera control and image processing:

- Image capture
- Video recording
- Basic image processing
- Computer vision algorithms

```astra
import sensors.camera;

var main_camera = camera.get_camera("MAIN");
var image = main_camera.capture_image();
image.save("/data/images/capture_" + time.now().format("%Y%m%d_%H%M%S") + ".jpg");
```

### `sensors.lidar`

LIDAR sensor interface:

- Point cloud acquisition
- Distance measurement
- Terrain mapping
- Obstacle detection

```astra
import sensors.lidar;

var scan = lidar.perform_scan(
    horizontal_resolution: 1.0, // degrees
    vertical_resolution: 1.0,   // degrees
    range: 100.0                // meters
);

var obstacles = lidar.detect_obstacles(scan);
```

### `data.telemetry`

Telemetry data handling:

- Data collection
- Transmission
- Compression
- Encryption

```astra
import data.telemetry;

var telemetry_packet = telemetry.create_packet();
telemetry_packet.add("altitude", current_altitude);
telemetry_packet.add("velocity", current_velocity);
telemetry_packet.add("fuel", fuel_remaining);

telemetry.transmit(telemetry_packet);
```

### `signal.processing`

Signal processing tools:

- Filters (low-pass, high-pass, band-pass)
- FFT and spectral analysis
- Signal generation
- Convolution and correlation
- Noise reduction

```astra
import signal.processing;

// Create a low-pass filter
var low_pass = Filter.create_low_pass(
    cutoff_frequency: 10.0,  // Hz
    sample_rate: 100.0       // Hz
);

// Apply filter to signal
var filtered_signal = low_pass.apply(raw_signal);

// Perform FFT
var spectrum = signal.calculateFFT(filtered_signal);

// Generate a test signal
var sine_wave = signal.generateSine(
    frequency: 5.0,   // Hz
    amplitude: 1.0,
    duration: 10.0,   // seconds
    sample_rate: 100.0 // Hz
);
```

## Communication Modules

### `comm.radio`

Radio communication:

- Transmitter and receiver control
- Signal modulation
- Link budget calculation
- Error correction

```astra
import comm.radio;

var main_radio = radio.get_transceiver("MAIN");
main_radio.set_frequency(437.5); // MHz
main_radio.set_power(5.0); // Watts

var message = "STATUS: All systems nominal";
main_radio.transmit(message);
```

### `comm.network`

Network communication:

- TCP/IP networking
- UDP communication
- HTTP client/server
- WebSocket support

```astra
import comm.network;

var telemetry_server = network.udp_socket();
telemetry_server.bind("0.0.0.0", 8080);

var telemetry_data = collect_telemetry();
telemetry_server.send(ground_station_address, telemetry_data.serialize());
```

## Simulation and Testing Modules

### `sim.environment`

Environmental simulation:

- Atmospheric modeling
- Space environment simulation
- Weather simulation
- Terrain generation

```astra
import sim.environment;

var atmosphere = environment.earth_atmosphere();
var density = atmosphere.get_density(altitude);
var wind = atmosphere.get_wind(position, altitude);
```

### `sim.spacecraft`

Spacecraft simulation:

- Spacecraft dynamics
- Subsystem simulation
- Failure modeling
- Mission simulation

```astra
import sim.spacecraft;

var satellite = spacecraft.create_satellite(
    mass: 150.0, // kg
    moments_of_inertia: [10.0, 12.0, 8.0], // kg·m²
    drag_coefficient: 2.2
);

spacecraft.simulate(
    satellite: satellite,
    duration: time.hours(24),
    time_step: time.seconds(1)
);
```

### `test.verification`

Testing and verification tools:

- Unit testing
- Property-based testing
- Coverage analysis
- Performance benchmarking

```astra
import test.verification;

test "orbital transfer calculation" {
    // Test setup
    var initial_orbit = orbit(500.0, 0.0);
    var target_orbit = orbit(1000.0, 0.0);
    
    // Function under test
    var transfer = mechanics.hohmann_transfer(initial_orbit, target_orbit);
    
    // Assertions
    assert(transfer.delta_v > 0.0);
    assert_approx_equal(
        transfer.final_orbit.semi_major_axis,
        target_orbit.semi_major_axis,
        tolerance: 0.1
    );
}
```

## Safety and Security Modules

### `safety.fault_detection`

Fault detection and handling:

- Error detection
- Fault isolation
- Recovery procedures
- Redundancy management

```astra
import safety.fault_detection;

var fault_monitor = fault_detection.create_monitor();
fault_monitor.add_check("battery_voltage", battery.voltage, 
                       min: 3.2, max: 4.2);
fault_monitor.add_check("imu_status", imu.status, 
                       expected: status.operational);

fault_monitor.on_fault("battery_voltage", () => {
    log.warning("Battery voltage out of range");
    power.enter_low_power_mode();
});
```

### `security.crypto`

Cryptography and security:

- Encryption/decryption
- Digital signatures
- Secure communication
- Authentication

```astra
import security.crypto;

var key = crypto.generate_key(crypto.algorithm.AES_256);
var encrypted_data = crypto.encrypt(command_data, key);
var signature = crypto.sign(encrypted_data, private_key);

comm.transmit_secure(encrypted_data, signature);
```

## Conclusion

The ASTRA standard library provides a comprehensive set of tools and functions specifically designed for aerospace and UAV applications. By offering specialized modules for orbital mechanics, navigation, propulsion, sensors, and more, it enables developers to focus on their specific application logic rather than reimplementing common aerospace algorithms and interfaces.