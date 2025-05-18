# ASTRA Integration Guide

This document provides comprehensive guidance on integrating ASTRA with existing systems, frameworks, and programming languages commonly used in aerospace and UAV applications.

## Table of Contents

1. [Introduction](#introduction)
2. [Foreign Function Interface (FFI)](#foreign-function-interface-ffi)
3. [Integration with C/C++](#integration-with-cc)
4. [Integration with Python](#integration-with-python)
5. [Integration with ROS](#integration-with-ros)
6. [Integration with MAVLink](#integration-with-mavlink)
7. [Integration with Flight Controllers](#integration-with-flight-controllers)
8. [Integration with Simulation Environments](#integration-with-simulation-environments)
9. [Integration with Data Processing Pipelines](#integration-with-data-processing-pipelines)
10. [Integration with Cloud Services](#integration-with-cloud-services)
11. [Case Studies](#case-studies)

## Introduction

ASTRA is designed to work seamlessly with existing systems and frameworks used in aerospace and UAV applications. This guide explains how to integrate ASTRA code with various programming languages, frameworks, and platforms.

Integration can occur at several levels:

1. **Code Level**: Calling functions across language boundaries
2. **Data Level**: Sharing data structures and serialization formats
3. **Process Level**: Communication between separate processes
4. **System Level**: Integration with operating systems and hardware

## Foreign Function Interface (FFI)

ASTRA's Foreign Function Interface (FFI) allows calling functions from other languages and exposing ASTRA functions to be called from other languages.

### Calling C Functions from ASTRA

```astra
// Import C functions
@extern("libc.so.6")
func c_sqrt(x: double) -> double;

@extern("libm.so.6")
func c_sin(x: double) -> double;

@extern("libsensor.so")
func read_sensor_c(sensor_id: int) -> float;

// Use imported C functions
func compute_value(x: float) -> float {
    var y = c_sqrt(x as double) as float;
    var z = c_sin(y as double) as float;
    return z;
}
```

### Calling C++ Functions from ASTRA

```astra
// Import C++ functions (requires C++ name mangling)
@extern("libcontrol.so", "C++")
func update_control_cpp(input: float) -> float;

// Import C++ class methods
@extern("libcontrol.so", "C++")
class Controller {
    @extern("Controller::Controller")
    public func constructor();
    
    @extern("Controller::update")
    public func update(input: float) -> float;
    
    @extern("Controller::reset")
    public func reset() -> void;
}

// Use imported C++ class
func control_system() -> void {
    var controller = Controller();
    
    while (true) {
        var sensor_value = read_sensor();
        var control_output = controller.update(sensor_value);
        set_actuator(control_output);
        time.sleep(10ms);
    }
}
```

### Exposing ASTRA Functions to C/C++

```astra
// Export ASTRA functions to be called from C/C++
@export("process_telemetry")
func process_telemetry(data: *void, size: int) -> int {
    // Convert C data to ASTRA types
    var telemetry = TelemetryData.from_c_struct(data, size);
    
    // Process telemetry
    var result = process(telemetry);
    
    // Return result code
    return result.code;
}

@export("initialize_system")
func initialize_system() -> bool {
    // Initialize ASTRA subsystems
    return true;
}
```

### Handling Callbacks

```astra
// Define callback type
type SensorCallback = func(sensor_id: int, value: float) -> void;

// Import C function that takes a callback
@extern("libsensor.so")
func register_sensor_callback_c(callback: SensorCallback) -> int;

// ASTRA function to be used as callback
func on_sensor_data(sensor_id: int, value: float) -> void {
    io.println("Sensor " + sensor_id.toString() + ": " + value.toString());
    process_sensor_data(sensor_id, value);
}

// Register callback
func setup_sensor_callbacks() -> void {
    var result = register_sensor_callback_c(on_sensor_data);
    if (result != 0) {
        io.println("Failed to register sensor callback: " + result.toString());
    }
}
```

## Integration with C/C++

### Embedding ASTRA in C/C++ Applications

```c
// C code
#include <astra_runtime.h>

int main() {
    // Initialize ASTRA runtime
    astra_init();
    
    // Load and compile ASTRA code
    astra_module_t* module = astra_load_file("control_system.astra");
    if (!module) {
        fprintf(stderr, "Failed to load ASTRA module\n");
        return 1;
    }
    
    // Get function from module
    astra_function_t* init_func = astra_get_function(module, "initialize");
    astra_function_t* update_func = astra_get_function(module, "update_control");
    
    if (!init_func || !update_func) {
        fprintf(stderr, "Failed to get ASTRA functions\n");
        astra_module_free(module);
        return 1;
    }
    
    // Call initialization function
    astra_value_t result;
    if (astra_call_function(init_func, NULL, 0, &result) != ASTRA_SUCCESS) {
        fprintf(stderr, "Failed to initialize ASTRA system\n");
        astra_module_free(module);
        return 1;
    }
    
    // Main control loop
    while (running) {
        // Prepare arguments
        float sensor_data[10];
        read_sensors(sensor_data);
        
        astra_value_t args[1];
        args[0] = astra_create_array_float(sensor_data, 10);
        
        // Call ASTRA update function
        astra_value_t control_result;
        if (astra_call_function(update_func, args, 1, &control_result) != ASTRA_SUCCESS) {
            fprintf(stderr, "Failed to call ASTRA update function\n");
            break;
        }
        
        // Extract control outputs
        float* control_outputs = astra_get_array_float(control_result);
        set_actuators(control_outputs, astra_get_array_length(control_result));
        
        // Clean up
        astra_value_free(args[0]);
        astra_value_free(control_result);
        
        // Sleep
        usleep(10000);  // 10ms
    }
    
    // Clean up
    astra_module_free(module);
    astra_shutdown();
    
    return 0;
}
```

### Using C/C++ Libraries in ASTRA

```astra
// ASTRA code using C++ libraries

// Import Eigen library functions
@extern("libeigen.so", "C++")
namespace Eigen {
    class Matrix3d {
        @extern("Eigen::Matrix3d::Matrix3d")
        public func constructor();
        
        @extern("Eigen::Matrix3d::operator()")
        public func get(row: int, col: int) -> double;
        
        @extern("Eigen::Matrix3d::operator()")
        public func set(row: int, col: int, value: double) -> void;
        
        @extern("Eigen::Matrix3d::inverse")
        public func inverse() -> Matrix3d;
        
        @extern("Eigen::Matrix3d::determinant")
        public func determinant() -> double;
    }
    
    class Vector3d {
        @extern("Eigen::Vector3d::Vector3d")
        public func constructor();
        
        @extern("Eigen::Vector3d::Vector3d")
        public func constructor(x: double, y: double, z: double);
        
        @extern("Eigen::Vector3d::operator()")
        public func get(index: int) -> double;
        
        @extern("Eigen::Vector3d::operator()")
        public func set(index: int, value: double) -> void;
        
        @extern("Eigen::Vector3d::dot")
        public func dot(other: Vector3d) -> double;
        
        @extern("Eigen::Vector3d::cross")
        public func cross(other: Vector3d) -> Vector3d;
    }
}

// Use Eigen in ASTRA code
func compute_inverse_dynamics(mass_matrix: Eigen.Matrix3d, coriolis: Eigen.Vector3d, gravity: Eigen.Vector3d, 
                             desired_accel: Eigen.Vector3d) -> Eigen.Vector3d {
    // τ = M(q)q̈ + C(q,q̇) + G(q)
    var inverse_mass = mass_matrix.inverse();
    var temp = desired_accel.clone();
    
    // Subtract coriolis and gravity
    for (var i = 0; i < 3; i++) {
        temp.set(i, temp.get(i) - coriolis.get(i) - gravity.get(i));
    }
    
    // Compute torques
    var torques = Eigen.Vector3d();
    for (var i = 0; i < 3; i++) {
        var sum = 0.0;
        for (var j = 0; j < 3; j++) {
            sum += inverse_mass.get(i, j) * temp.get(j);
        }
        torques.set(i, sum);
    }
    
    return torques;
}
```

### Shared Memory Communication

```astra
import shared_memory;

// Create or open shared memory segment
var shm = shared_memory.create("control_data", 1024);  // 1KB

// Write data to shared memory
func write_control_data(data: ControlData) -> void {
    var view = shm.get_view();
    
    // Write data fields
    view.write_float64(0, data.timestamp);
    view.write_float32(8, data.roll);
    view.write_float32(12, data.pitch);
    view.write_float32(16, data.yaw);
    view.write_float32(20, data.thrust);
    
    // Signal that new data is available
    shm.signal();
}

// Read data from shared memory
func read_control_data() -> ControlData {
    var view = shm.get_view();
    var data = ControlData();
    
    // Wait for new data
    shm.wait();
    
    // Read data fields
    data.timestamp = view.read_float64(0);
    data.roll = view.read_float32(8);
    data.pitch = view.read_float32(12);
    data.yaw = view.read_float32(16);
    data.thrust = view.read_float32(20);
    
    return data;
}
```

## Integration with Python

### Calling Python from ASTRA

```astra
import python;

// Initialize Python interpreter
func initialize_python() -> void {
    python.initialize();
    
    // Add module search paths
    python.add_path("/path/to/python/modules");
    
    // Import required modules
    python.execute("import numpy as np");
    python.execute("import scipy.signal as signal");
}

// Call Python functions
func filter_signal(data: array<float>) -> array<float> {
    // Convert ASTRA array to Python list
    var py_data = python.create_list();
    for (var i = 0; i < data.length; i++) {
        python.list_append(py_data, python.create_float(data[i]));
    }
    
    // Create filter parameters
    var b = python.execute("np.array([0.1, 0.2, 0.4, 0.2, 0.1])");
    var a = python.execute("np.array([1.0, 0.0, 0.0, 0.0, 0.0])");
    
    // Apply filter
    var py_result = python.call_function("signal.lfilter", [b, a, py_data]);
    
    // Convert Python result back to ASTRA array
    var result = array<float>[python.list_length(py_result)];
    for (var i = 0; i < result.length; i++) {
        result[i] = python.to_float(python.list_get(py_result, i));
    }
    
    // Clean up Python objects
    python.decref(py_data);
    python.decref(b);
    python.decref(a);
    python.decref(py_result);
    
    return result;
}

// Clean up Python interpreter
func finalize_python() -> void {
    python.finalize();
}
```

### Calling ASTRA from Python

```python
# Python code
import astra

# Load ASTRA module
module = astra.load_module("navigation.astra")

# Create ASTRA objects
position = astra.Vector3(1.0, 2.0, 3.0)
velocity = astra.Vector3(0.1, 0.2, 0.3)
waypoint = astra.Vector3(10.0, 20.0, 30.0)

# Call ASTRA functions
distance = module.calculate_distance(position, waypoint)
time_to_waypoint = module.estimate_arrival_time(position, velocity, waypoint)

print(f"Distance to waypoint: {distance} meters")
print(f"Estimated arrival time: {time_to_waypoint} seconds")

# Create callback function
def on_waypoint_reached(waypoint_id, position):
    print(f"Reached waypoint {waypoint_id} at position {position}")

# Register Python callback with ASTRA
module.set_waypoint_callback(on_waypoint_reached)

# Run ASTRA navigation system
navigator = module.Navigator()
navigator.initialize()
navigator.add_waypoint(waypoint)
navigator.start()

# Wait for navigation to complete
while navigator.is_active():
    status = navigator.get_status()
    print(f"Navigation status: {status}")
    time.sleep(1.0)
```

### Using Python Libraries for Data Processing

```astra
import python;

// Use Python's scikit-learn for machine learning
func train_anomaly_detector(normal_data: array<array<float>>) -> any {
    // Initialize Python
    python.initialize();
    
    // Import required modules
    python.execute("import numpy as np");
    python.execute("from sklearn.ensemble import IsolationForest");
    
    // Convert training data to numpy array
    var py_data = python.execute("np.zeros((?, ?), dtype=np.float32)", [normal_data.length, normal_data[0].length]);
    for (var i = 0; i < normal_data.length; i++) {
        for (var j = 0; j < normal_data[i].length; j++) {
            python.execute("?[?, ?] = ?", [py_data, i, j, normal_data[i][j]]);
        }
    }
    
    // Create and train model
    var model = python.execute("IsolationForest(contamination=0.01, random_state=42)");
    python.call_method(model, "fit", [py_data]);
    
    // Return model (as Python object)
    return model;
}

// Use trained model for anomaly detection
func detect_anomalies(model: any, data: array<float>) -> bool {
    // Convert data to numpy array
    var py_data = python.execute("np.array(?, dtype=np.float32)", [data]);
    
    // Reshape to 2D array with one sample
    py_data = python.execute("?.reshape(1, -1)", [py_data]);
    
    // Predict anomaly score
    var result = python.call_method(model, "predict", [py_data]);
    
    // Convert result to boolean (1 = normal, -1 = anomaly)
    var is_anomaly = python.to_int(python.list_get(result, 0)) == -1;
    
    // Clean up Python objects
    python.decref(py_data);
    python.decref(result);
    
    return is_anomaly;
}
```

## Integration with ROS

### Creating ROS Nodes in ASTRA

```astra
import ros;

// Initialize ROS node
func initialize_ros() -> void {
    ros.init("astra_control_node");
}

// Create publishers and subscribers
var attitude_pub = ros.create_publisher<geometry_msgs.QuaternionStamped>("/attitude", 10);
var imu_sub = ros.create_subscriber<sensor_msgs.Imu>("/imu/data", 10, on_imu_data);
var cmd_vel_sub = ros.create_subscriber<geometry_msgs.Twist>("/cmd_vel", 10, on_cmd_vel);

// Callback for IMU data
func on_imu_data(msg: sensor_msgs.Imu) -> void {
    // Extract orientation quaternion
    var quat = Quaternion(
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    );
    
    // Extract angular velocity
    var angular_vel = Vector3(
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z
    );
    
    // Extract linear acceleration
    var linear_accel = Vector3(
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z
    );
    
    // Process IMU data
    process_imu_data(quat, angular_vel, linear_accel);
}

// Callback for velocity commands
func on_cmd_vel(msg: geometry_msgs.Twist) -> void {
    // Extract linear velocity
    var linear_vel = Vector3(
        msg.linear.x,
        msg.linear.y,
        msg.linear.z
    );
    
    // Extract angular velocity
    var angular_vel = Vector3(
        msg.angular.x,
        msg.angular.y,
        msg.angular.z
    );
    
    // Process velocity command
    process_velocity_command(linear_vel, angular_vel);
}

// Publish attitude data
func publish_attitude(attitude: Quaternion) -> void {
    var msg = geometry_msgs.QuaternionStamped();
    
    // Set header
    msg.header.stamp = ros.time.now();
    msg.header.frame_id = "base_link";
    
    // Set quaternion
    msg.quaternion.w = attitude.w;
    msg.quaternion.x = attitude.x;
    msg.quaternion.y = attitude.y;
    msg.quaternion.z = attitude.z;
    
    // Publish message
    attitude_pub.publish(msg);
}

// Main ROS node loop
func run_ros_node() -> void {
    // Set loop rate
    var rate = ros.Rate(100);  // 100 Hz
    
    while (ros.ok()) {
        // Perform control computations
        var attitude = compute_attitude();
        
        // Publish results
        publish_attitude(attitude);
        
        // Sleep to maintain loop rate
        rate.sleep();
    }
}
```

### Using ROS Services

```astra
import ros;

// Define service client
var set_mode_client = ros.create_service_client<mavros_msgs.SetMode>("/mavros/set_mode");

// Call ROS service
func set_flight_mode(mode: string) -> bool {
    var request = mavros_msgs.SetMode.Request();
    request.custom_mode = mode;
    
    var response = set_mode_client.call(request);
    
    if (response != null) {
        return response.mode_sent;
    }
    
    return false;
}

// Define service server
var get_status_server = ros.create_service_server<custom_msgs.GetStatus>("/drone/get_status", on_get_status);

// Service callback
func on_get_status(request: custom_msgs.GetStatus.Request) -> custom_msgs.GetStatus.Response {
    var response = custom_msgs.GetStatus.Response();
    
    // Fill response with current status
    var status = get_current_status();
    response.battery_level = status.battery_level;
    response.gps_fix = status.gps_fix;
    response.armed = status.armed;
    response.mode = status.mode;
    
    return response;
}
```

### Using ROS Actions

```astra
import ros;

// Define action client
var takeoff_client = ros.create_action_client<mavros_msgs.CommandTOL>("/mavros/cmd/takeoff");

// Send action goal
func takeoff(altitude: float) -> bool {
    var goal = mavros_msgs.CommandTOL.Goal();
    goal.min_pitch = 0.0;
    goal.yaw = 0.0;
    goal.latitude = 0.0;  // Use current position
    goal.longitude = 0.0;  // Use current position
    goal.altitude = altitude;
    
    // Send goal and wait for result
    var result = takeoff_client.send_goal_and_wait(goal, 30.0);  // 30 second timeout
    
    if (result != null) {
        return result.success;
    }
    
    return false;
}

// Define action server
var navigate_server = ros.create_action_server<custom_msgs.Navigate>("/drone/navigate", on_navigate_goal);

// Action goal callback
func on_navigate_goal(goal: custom_msgs.Navigate.Goal) -> void {
    // Extract waypoints
    var waypoints = array<Vector3>[goal.waypoints.length];
    for (var i = 0; i < goal.waypoints.length; i++) {
        waypoints[i] = Vector3(
            goal.waypoints[i].x,
            goal.waypoints[i].y,
            goal.waypoints[i].z
        );
    }
    
    // Start navigation
    var navigator = Navigator(waypoints);
    navigator.start();
    
    // Publish feedback during navigation
    while (navigator.is_active()) {
        var current_waypoint = navigator.get_current_waypoint_index();
        var position = navigator.get_current_position();
        
        var feedback = custom_msgs.Navigate.Feedback();
        feedback.current_waypoint = current_waypoint;
        feedback.current_position.x = position.x;
        feedback.current_position.y = position.y;
        feedback.current_position.z = position.z;
        
        navigate_server.publish_feedback(feedback);
        
        time.sleep(100ms);
    }
    
    // Send result
    var result = custom_msgs.Navigate.Result();
    result.success = navigator.is_completed();
    result.error_message = navigator.get_error_message();
    
    if (result.success) {
        navigate_server.set_succeeded(result);
    } else {
        navigate_server.set_aborted(result);
    }
}
```

## Integration with MAVLink

### Sending MAVLink Messages

```astra
import mavlink;

// Create MAVLink connection
var connection = mavlink.Connection("serial:///dev/ttyACM0:57600");

// Set system and component IDs
connection.set_system_id(1);
connection.set_component_id(1);

// Send heartbeat message
func send_heartbeat() -> void {
    var msg = mavlink.Message();
    msg.id = mavlink.HEARTBEAT;
    msg.type = mavlink.MAV_TYPE_QUADROTOR;
    msg.autopilot = mavlink.MAV_AUTOPILOT_GENERIC;
    msg.base_mode = mavlink.MAV_MODE_GUIDED_ARMED;
    msg.custom_mode = 0;
    msg.system_status = mavlink.MAV_STATE_ACTIVE;
    
    connection.send_message(msg);
}

// Send attitude message
func send_attitude(roll: float, pitch: float, yaw: float,
                  roll_speed: float, pitch_speed: float, yaw_speed: float) -> void {
    var msg = mavlink.Message();
    msg.id = mavlink.ATTITUDE;
    msg.time_boot_ms = time.millis();
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    msg.rollspeed = roll_speed;
    msg.pitchspeed = pitch_speed;
    msg.yawspeed = yaw_speed;
    
    connection.send_message(msg);
}

// Send position message
func send_global_position(lat: double, lon: double, alt: float,
                         vx: float, vy: float, vz: float) -> void {
    var msg = mavlink.Message();
    msg.id = mavlink.GLOBAL_POSITION_INT;
    msg.time_boot_ms = time.millis();
    msg.lat = (lat * 1e7) as int;  // degrees * 1e7
    msg.lon = (lon * 1e7) as int;  // degrees * 1e7
    msg.alt = (alt * 1000) as int;  // meters * 1000
    msg.relative_alt = (alt * 1000) as int;  // meters * 1000
    msg.vx = (vx * 100) as int;  // m/s * 100
    msg.vy = (vy * 100) as int;  // m/s * 100
    msg.vz = (vz * 100) as int;  // m/s * 100
    
    connection.send_message(msg);
}
```

### Receiving MAVLink Messages

```astra
import mavlink;

// Create MAVLink connection
var connection = mavlink.Connection("udpin:0.0.0.0:14550");

// Set message callbacks
connection.set_message_callback(mavlink.HEARTBEAT, on_heartbeat);
connection.set_message_callback(mavlink.COMMAND_LONG, on_command);
connection.set_message_callback(mavlink.SET_MODE, on_set_mode);

// Heartbeat callback
func on_heartbeat(msg: mavlink.Message) -> void {
    var system_id = msg.system_id;
    var component_id = msg.component_id;
    var type = msg.type;
    var autopilot = msg.autopilot;
    var base_mode = msg.base_mode;
    var custom_mode = msg.custom_mode;
    var system_status = msg.system_status;
    
    io.println("Received heartbeat from system " + system_id.toString() +
              ", component " + component_id.toString());
}

// Command callback
func on_command(msg: mavlink.Message) -> void {
    var system_id = msg.system_id;
    var component_id = msg.component_id;
    var command = msg.command;
    var confirmation = msg.confirmation;
    var param1 = msg.param1;
    var param2 = msg.param2;
    var param3 = msg.param3;
    var param4 = msg.param4;
    var param5 = msg.param5;
    var param6 = msg.param6;
    var param7 = msg.param7;
    
    io.println("Received command " + command.toString() + " from system " + system_id.toString());
    
    // Handle specific commands
    if (command == mavlink.MAV_CMD_NAV_TAKEOFF) {
        var altitude = param7;
        handle_takeoff_command(altitude);
    } else if (command == mavlink.MAV_CMD_NAV_LAND) {
        handle_land_command();
    }
    
    // Send acknowledgement
    send_command_ack(command, mavlink.MAV_RESULT_ACCEPTED, system_id, component_id);
}

// Set mode callback
func on_set_mode(msg: mavlink.Message) -> void {
    var system_id = msg.system_id;
    var component_id = msg.component_id;
    var base_mode = msg.base_mode;
    var custom_mode = msg.custom_mode;
    
    io.println("Received set mode command from system " + system_id.toString() +
              ": base_mode=" + base_mode.toString() + ", custom_mode=" + custom_mode.toString());
    
    handle_set_mode(base_mode, custom_mode);
}

// Send command acknowledgement
func send_command_ack(command: uint16, result: uint8, target_system: uint8, target_component: uint8) -> void {
    var msg = mavlink.Message();
    msg.id = mavlink.COMMAND_ACK;
    msg.command = command;
    msg.result = result;
    msg.target_system = target_system;
    msg.target_component = target_component;
    
    connection.send_message(msg);
}

// Main MAVLink processing loop
func process_mavlink() -> void {
    while (true) {
        // Process incoming messages
        connection.update();
        
        // Send periodic messages
        send_heartbeat();
        
        // Sleep to avoid busy waiting
        time.sleep(100ms);
    }
}
```

### Implementing a MAVLink Mission Protocol

```astra
import mavlink;

// Mission items
var mission_items = array<MissionItem>[];
var current_mission_item = 0;
var mission_active = false;

// Mission item structure
struct MissionItem {
    var seq: uint16;
    var frame: uint8;
    var command: uint16;
    var current: uint8;
    var autocontinue: uint8;
    var param1: float;
    var param2: float;
    var param3: float;
    var param4: float;
    var x: float;  // latitude
    var y: float;  // longitude
    var z: float;  // altitude
}

// Handle mission request list
func handle_mission_request_list(msg: mavlink.Message) -> void {
    var system_id = msg.system_id;
    var component_id = msg.component_id;
    
    // Send mission count
    var count_msg = mavlink.Message();
    count_msg.id = mavlink.MISSION_COUNT;
    count_msg.target_system = system_id;
    count_msg.target_component = component_id;
    count_msg.count = mission_items.length as uint16;
    
    connection.send_message(count_msg);
}

// Handle mission request
func handle_mission_request(msg: mavlink.Message) -> void {
    var system_id = msg.system_id;
    var component_id = msg.component_id;
    var seq = msg.seq;
    
    if (seq < mission_items.length) {
        var item = mission_items[seq];
        
        var item_msg = mavlink.Message();
        item_msg.id = mavlink.MISSION_ITEM;
        item_msg.target_system = system_id;
        item_msg.target_component = component_id;
        item_msg.seq = item.seq;
        item_msg.frame = item.frame;
        item_msg.command = item.command;
        item_msg.current = item.current;
        item_msg.autocontinue = item.autocontinue;
        item_msg.param1 = item.param1;
        item_msg.param2 = item.param2;
        item_msg.param3 = item.param3;
        item_msg.param4 = item.param4;
        item_msg.x = item.x;
        item_msg.y = item.y;
        item_msg.z = item.z;
        
        connection.send_message(item_msg);
    }
}

// Handle mission ack
func handle_mission_ack(msg: mavlink.Message) -> void {
    var type = msg.type;
    
    if (type == mavlink.MAV_MISSION_ACCEPTED) {
        io.println("Mission accepted");
    } else {
        io.println("Mission not accepted: " + type.toString());
    }
}

// Handle mission item reached
func handle_mission_item_reached(msg: mavlink.Message) -> void {
    var seq = msg.seq;
    
    io.println("Mission item " + seq.toString() + " reached");
    
    if (seq == mission_items.length - 1) {
        io.println("Mission completed");
        mission_active = false;
    }
}

// Start mission execution
func start_mission() -> void {
    if (mission_items.length > 0) {
        var msg = mavlink.Message();
        msg.id = mavlink.MISSION_SET_CURRENT;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.seq = 0;
        
        connection.send_message(msg);
        
        current_mission_item = 0;
        mission_active = true;
        
        io.println("Mission started");
    }
}

// Add mission item
func add_mission_item(lat: float, lon: float, alt: float, command: uint16) -> void {
    var item = MissionItem();
    item.seq = mission_items.length as uint16;
    item.frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT;
    item.command = command;
    item.current = 0;
    item.autocontinue = 1;
    item.param1 = 0.0;  // Depends on command
    item.param2 = 0.0;  // Depends on command
    item.param3 = 0.0;  // Depends on command
    item.param4 = 0.0;  // Depends on command
    item.x = lat;
    item.y = lon;
    item.z = alt;
    
    mission_items.push(item);
}

// Clear mission
func clear_mission() -> void {
    mission_items = array<MissionItem>[];
    current_mission_item = 0;
    mission_active = false;
    
    var msg = mavlink.Message();
    msg.id = mavlink.MISSION_CLEAR_ALL;
    msg.target_system = 1;
    msg.target_component = 1;
    
    connection.send_message(msg);
    
    io.println("Mission cleared");
}
```

## Integration with Flight Controllers

### PX4 Integration

```astra
import mavlink;
import px4;

// Connect to PX4 flight controller
var px4_connection = px4.connect("serial:///dev/ttyACM0:921600");

// Set up offboard control
func setup_offboard_control() -> bool {
    // Set the system to offboard mode
    var result = px4_connection.set_mode(px4.VEHICLE_MODE_OFFBOARD);
    if (!result) {
        io.println("Failed to set offboard mode");
        return false;
    }
    
    // Arm the vehicle
    result = px4_connection.arm();
    if (!result) {
        io.println("Failed to arm vehicle");
        return false;
    }
    
    return true;
}

// Send position setpoint
func send_position_setpoint(x: float, y: float, z: float, yaw: float) -> void {
    px4_connection.set_position_target_local_ned(
        time.millis(),
        px4_connection.get_system_id(),
        px4_connection.get_component_id(),
        px4.FRAME_LOCAL_NED,
        0b0000111111111000,  // Use position and yaw
        x, y, z,             // Position
        0.0, 0.0, 0.0,       // Velocity
        0.0, 0.0, 0.0,       // Acceleration
        yaw, 0.0             // Yaw and yaw rate
    );
}

// Send velocity setpoint
func send_velocity_setpoint(vx: float, vy: float, vz: float, yaw_rate: float) -> void {
    px4_connection.set_position_target_local_ned(
        time.millis(),
        px4_connection.get_system_id(),
        px4_connection.get_component_id(),
        px4.FRAME_LOCAL_NED,
        0b0000111111000111,  // Use velocity and yaw rate
        0.0, 0.0, 0.0,       // Position
        vx, vy, vz,          // Velocity
        0.0, 0.0, 0.0,       // Acceleration
        0.0, yaw_rate        // Yaw and yaw rate
    );
}

// Send attitude setpoint
func send_attitude_setpoint(roll: float, pitch: float, yaw: float, thrust: float) -> void {
    px4_connection.set_attitude_target(
        time.millis(),
        px4_connection.get_system_id(),
        px4_connection.get_component_id(),
        0b00000111,          // Ignore rate
        Quaternion.from_euler(roll, pitch, yaw),
        0.0, 0.0, 0.0,       // Roll, pitch, yaw rates
        thrust
    );
}

// Receive vehicle state
func get_vehicle_state() -> px4.VehicleState {
    return px4_connection.get_vehicle_state();
}

// Main control loop
func run_control_loop() -> void {
    // Set up offboard control
    if (!setup_offboard_control()) {
        io.println("Failed to set up offboard control");
        return;
    }
    
    // Control loop
    var rate = time.Rate(50);  // 50 Hz
    
    while (true) {
        // Get current state
        var state = get_vehicle_state();
        
        // Compute control commands
        var target_position = compute_target_position(state);
        
        // Send setpoint
        send_position_setpoint(
            target_position.x,
            target_position.y,
            target_position.z,
            target_position.yaw
        );
        
        // Sleep to maintain loop rate
        rate.sleep();
    }
}
```

### ArduPilot Integration

```astra
import mavlink;
import ardupilot;

// Connect to ArduPilot flight controller
var ardupilot_connection = ardupilot.connect("udp:localhost:14550");

// Set up guided mode
func setup_guided_mode() -> bool {
    // Set the system to guided mode
    var result = ardupilot_connection.set_mode(ardupilot.MODE_GUIDED);
    if (!result) {
        io.println("Failed to set guided mode");
        return false;
    }
    
    // Arm the vehicle
    result = ardupilot_connection.arm();
    if (!result) {
        io.println("Failed to arm vehicle");
        return false;
    }
    
    return true;
}

// Send position target
func send_position_target(lat: float, lon: float, alt: float) -> void {
    ardupilot_connection.send_position_target_global_int(
        time.millis(),
        ardupilot_connection.get_system_id(),
        ardupilot_connection.get_component_id(),
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  // Use position
        (lat * 1e7) as int,  // Latitude (degrees * 1e7)
        (lon * 1e7) as int,  // Longitude (degrees * 1e7)
        alt,                 // Altitude (meters)
        0.0, 0.0, 0.0,       // Velocity
        0.0, 0.0, 0.0,       // Acceleration
        0.0, 0.0             // Yaw and yaw rate
    );
}

// Send velocity target
func send_velocity_target(vx: float, vy: float, vz: float, yaw_rate: float) -> void {
    ardupilot_connection.send_position_target_local_ned(
        time.millis(),
        ardupilot_connection.get_system_id(),
        ardupilot_connection.get_component_id(),
        mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  // Use velocity and yaw rate
        0.0, 0.0, 0.0,       // Position
        vx, vy, vz,          // Velocity
        0.0, 0.0, 0.0,       // Acceleration
        0.0, yaw_rate        // Yaw and yaw rate
    );
}

// Takeoff command
func takeoff(altitude: float) -> bool {
    return ardupilot_connection.takeoff(altitude);
}

// Land command
func land() -> bool {
    return ardupilot_connection.land();
}

// Return to launch
func return_to_launch() -> bool {
    return ardupilot_connection.return_to_launch();
}

// Get vehicle state
func get_vehicle_state() -> ardupilot.VehicleState {
    return ardupilot_connection.get_vehicle_state();
}

// Main control loop
func run_mission() -> void {
    // Set up guided mode
    if (!setup_guided_mode()) {
        io.println("Failed to set up guided mode");
        return;
    }
    
    // Takeoff
    if (!takeoff(10.0)) {
        io.println("Failed to takeoff");
        return;
    }
    
    // Wait for takeoff to complete
    while (get_vehicle_state().altitude < 9.5) {
        time.sleep(100ms);
    }
    
    io.println("Takeoff complete");
    
    // Fly to waypoints
    var waypoints = [
        (47.3977419, 8.5455938, 10.0),  // Latitude, longitude, altitude
        (47.3977419, 8.5456938, 15.0),
        (47.3978419, 8.5456938, 15.0),
        (47.3978419, 8.5455938, 10.0)
    ];
    
    for (var i = 0; i < waypoints.length; i++) {
        var waypoint = waypoints[i];
        
        io.println("Flying to waypoint " + (i + 1).toString());
        send_position_target(waypoint.0, waypoint.1, waypoint.2);
        
        // Wait for waypoint to be reached
        var timeout = time.now() + 30.0;  // 30 second timeout
        while (time.now() < timeout) {
            var state = get_vehicle_state();
            var distance = calculate_distance(
                state.latitude, state.longitude, state.altitude,
                waypoint.0, waypoint.1, waypoint.2
            );
            
            if (distance < 1.0) {
                io.println("Waypoint " + (i + 1).toString() + " reached");
                break;
            }
            
            time.sleep(100ms);
        }
    }
    
    // Return to launch
    io.println("Returning to launch");
    return_to_launch();
    
    // Wait for landing
    while (get_vehicle_state().armed) {
        time.sleep(100ms);
    }
    
    io.println("Mission complete");
}
```

### Betaflight Integration

```astra
import serial;
import msp;  // MultiWii Serial Protocol

// Connect to Betaflight flight controller
var bf_serial = serial.open("/dev/ttyUSB0", 115200);
var bf_connection = msp.Connection(bf_serial);

// Get flight controller information
func get_fc_info() -> void {
    // Request API version
    var api_version = bf_connection.send_command(msp.MSP_API_VERSION);
    io.println("API Version: " + api_version.data[1].toString() + "." + api_version.data[2].toString() + "." + api_version.data[3].toString());
    
    // Request FC variant
    var fc_variant = bf_connection.send_command(msp.MSP_FC_VARIANT);
    var variant_str = "";
    for (var i = 0; i < 4; i++) {
        variant_str += String.fromCharCode(fc_variant.data[i]);
    }
    io.println("FC Variant: " + variant_str);
    
    // Request FC version
    var fc_version = bf_connection.send_command(msp.MSP_FC_VERSION);
    io.println("FC Version: " + fc_version.data[0].toString() + "." + fc_version.data[1].toString() + "." + fc_version.data[2].toString());
}

// Arm the flight controller
func arm() -> void {
    // Get current RC channels
    var rc_channels = bf_connection.send_command(msp.MSP_RC);
    
    // Set throttle low and yaw right to arm
    var channels = array<uint16>[rc_channels.data.length / 2];
    for (var i = 0; i < channels.length; i++) {
        var low_byte = rc_channels.data[i * 2];
        var high_byte = rc_channels.data[i * 2 + 1];
        channels[i] = (high_byte << 8) | low_byte;
    }
    
    // Set throttle low (channel 3)
    channels[2] = 1000;
    
    // Set yaw right (channel 4)
    channels[3] = 2000;
    
    // Send RC channels
    var data = array<byte>[channels.length * 2];
    for (var i = 0; i < channels.length; i++) {
        data[i * 2] = (channels[i] & 0xFF) as byte;
        data[i * 2 + 1] = ((channels[i] >> 8) & 0xFF) as byte;
    }
    
    bf_connection.send_command_with_data(msp.MSP_SET_RAW_RC, data);
    
    io.println("Armed");
}

// Disarm the flight controller
func disarm() -> void {
    // Get current RC channels
    var rc_channels = bf_connection.send_command(msp.MSP_RC);
    
    // Set throttle low and yaw left to disarm
    var channels = array<uint16>[rc_channels.data.length / 2];
    for (var i = 0; i < channels.length; i++) {
        var low_byte = rc_channels.data[i * 2];
        var high_byte = rc_channels.data[i * 2 + 1];
        channels[i] = (high_byte << 8) | low_byte;
    }
    
    // Set throttle low (channel 3)
    channels[2] = 1000;
    
    // Set yaw left (channel 4)
    channels[3] = 1000;
    
    // Send RC channels
    var data = array<byte>[channels.length * 2];
    for (var i = 0; i < channels.length; i++) {
        data[i * 2] = (channels[i] & 0xFF) as byte;
        data[i * 2 + 1] = ((channels[i] >> 8) & 0xFF) as byte;
    }
    
    bf_connection.send_command_with_data(msp.MSP_SET_RAW_RC, data);
    
    io.println("Disarmed");
}

// Set RC channels
func set_rc_channels(roll: uint16, pitch: uint16, throttle: uint16, yaw: uint16) -> void {
    // Get current RC channels to determine how many channels are available
    var rc_channels = bf_connection.send_command(msp.MSP_RC);
    var channel_count = rc_channels.data.length / 2;
    
    // Create channel array
    var channels = array<uint16>[channel_count];
    
    // Set primary channels
    channels[0] = roll;      // Roll
    channels[1] = pitch;     // Pitch
    channels[2] = throttle;  // Throttle
    channels[3] = yaw;       // Yaw
    
    // Keep other channels at their current values
    for (var i = 4; i < channel_count; i++) {
        var low_byte = rc_channels.data[i * 2];
        var high_byte = rc_channels.data[i * 2 + 1];
        channels[i] = (high_byte << 8) | low_byte;
    }
    
    // Send RC channels
    var data = array<byte>[channels.length * 2];
    for (var i = 0; i < channels.length; i++) {
        data[i * 2] = (channels[i] & 0xFF) as byte;
        data[i * 2 + 1] = ((channels[i] >> 8) & 0xFF) as byte;
    }
    
    bf_connection.send_command_with_data(msp.MSP_SET_RAW_RC, data);
}

// Get attitude (roll, pitch, yaw)
func get_attitude() -> (float, float, float) {
    var attitude = bf_connection.send_command(msp.MSP_ATTITUDE);
    
    var roll = ((attitude.data[0] | (attitude.data[1] << 8)) / 10.0);
    var pitch = ((attitude.data[2] | (attitude.data[3] << 8)) / 10.0);
    var yaw = (attitude.data[4] | (attitude.data[5] << 8));
    
    return (roll, pitch, yaw);
}

// Main control loop
func run_control_loop() -> void {
    // Get FC info
    get_fc_info();
    
    // Arm the flight controller
    arm();
    
    // Wait for arming to complete
    time.sleep(1s);
    
    // Control loop
    var start_time = time.now();
    var duration = 10.0;  // 10 seconds
    
    while (time.now() - start_time < duration) {
        // Get current attitude
        var (roll, pitch, yaw) = get_attitude();
        io.println("Roll: " + roll.toString() + ", Pitch: " + pitch.toString() + ", Yaw: " + yaw.toString());
        
        // Set RC channels (hover with slight roll oscillation)
        var t = time.now() - start_time;
        var roll_value = 1500 + (Math.sin(t * 2.0) * 100.0) as uint16;
        set_rc_channels(roll_value, 1500, 1300, 1500);  // Roll, Pitch, Throttle, Yaw
        
        // Sleep to maintain loop rate
        time.sleep(20ms);
    }
    
    // Disarm
    disarm();
    
    io.println("Control loop complete");
}
```

## Integration with Simulation Environments

### Gazebo Integration

```astra
import ros;
import gazebo_msgs;

// Connect to Gazebo via ROS
func initialize_gazebo() -> void {
    ros.init("astra_gazebo_controller");
}

// Spawn model in Gazebo
func spawn_model(model_name: string, model_xml: string, namespace: string,
                initial_pose: geometry_msgs.Pose, reference_frame: string) -> bool {
    var spawn_client = ros.create_service_client<gazebo_msgs.SpawnModel>("/gazebo/spawn_urdf_model");
    
    var request = gazebo_msgs.SpawnModel.Request();
    request.model_name = model_name;
    request.model_xml = model_xml;
    request.robot_namespace = namespace;
    request.initial_pose = initial_pose;
    request.reference_frame = reference_frame;
    
    var response = spawn_client.call(request);
    
    if (response != null) {
        return response.success;
    }
    
    return false;
}

// Delete model from Gazebo
func delete_model(model_name: string) -> bool {
    var delete_client = ros.create_service_client<gazebo_msgs.DeleteModel>("/gazebo/delete_model");
    
    var request = gazebo_msgs.DeleteModel.Request();
    request.model_name = model_name;
    
    var response = delete_client.call(request);
    
    if (response != null) {
        return response.success;
    }
    
    return false;
}

// Set model state in Gazebo
func set_model_state(model_name: string, pose: geometry_msgs.Pose, twist: geometry_msgs.Twist,
                    reference_frame: string) -> bool {
    var state_client = ros.create_service_client<gazebo_msgs.SetModelState>("/gazebo/set_model_state");
    
    var request = gazebo_msgs.SetModelState.Request();
    request.model_state.model_name = model_name;
    request.model_state.pose = pose;
    request.model_state.twist = twist;
    request.model_state.reference_frame = reference_frame;
    
    var response = state_client.call(request);
    
    if (response != null) {
        return response.success;
    }
    
    return false;
}

// Get model state from Gazebo
func get_model_state(model_name: string, reference_frame: string) -> gazebo_msgs.GetModelState.Response? {
    var state_client = ros.create_service_client<gazebo_msgs.GetModelState>("/gazebo/get_model_state");
    
    var request = gazebo_msgs.GetModelState.Request();
    request.model_name = model_name;
    request.relative_entity_name = reference_frame;
    
    return state_client.call(request);
}

// Apply force to model in Gazebo
func apply_body_wrench(body_name: string, reference_frame: string, reference_point: geometry_msgs.Point,
                      wrench: geometry_msgs.Wrench, duration: ros.Duration) -> bool {
    var wrench_client = ros.create_service_client<gazebo_msgs.ApplyBodyWrench>("/gazebo/apply_body_wrench");
    
    var request = gazebo_msgs.ApplyBodyWrench.Request();
    request.body_name = body_name;
    request.reference_frame = reference_frame;
    request.reference_point = reference_point;
    request.wrench = wrench;
    request.start_time = ros.Time(0);  // Apply immediately
    request.duration = duration;
    
    var response = wrench_client.call(request);
    
    if (response != null) {
        return response.success;
    }
    
    return false;
}

// Simulate drone in Gazebo
func simulate_drone() -> void {
    // Initialize ROS and Gazebo connection
    initialize_gazebo();
    
    // Spawn drone model
    var drone_urdf = file.read_text("drone.urdf");
    var initial_pose = geometry_msgs.Pose();
    initial_pose.position.x = 0.0;
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 1.0;
    initial_pose.orientation.w = 1.0;
    
    var success = spawn_model("drone", drone_urdf, "", initial_pose, "world");
    
    if (!success) {
        io.println("Failed to spawn drone model");
        return;
    }
    
    io.println("Drone model spawned successfully");
    
    // Create publishers for control
    var cmd_vel_pub = ros.create_publisher<geometry_msgs.Twist>("/drone/cmd_vel", 10);
    
    // Create subscribers for feedback
    var pose_sub = ros.create_subscriber<gazebo_msgs.ModelStates>("/gazebo/model_states", 10, on_model_states);
    
    // Control loop
    var rate = ros.Rate(50);  // 50 Hz
    var start_time = ros.time.now();
    
    while (ros.ok() && (ros.time.now() - start_time).to_sec() < 60.0) {  // Run for 60 seconds
        // Create control command
        var cmd_vel = geometry_msgs.Twist();
        
        // Circular flight pattern
        var t = (ros.time.now() - start_time).to_sec();
        cmd_vel.linear.x = Math.cos(t * 0.5) * 0.5;
        cmd_vel.linear.y = Math.sin(t * 0.5) * 0.5;
        cmd_vel.linear.z = Math.sin(t * 0.2) * 0.2;
        cmd_vel.angular.z = 0.5;
        
        // Publish command
        cmd_vel_pub.publish(cmd_vel);
        
        // Sleep to maintain loop rate
        rate.sleep();
    }
    
    // Land the drone
    io.println("Landing drone...");
    
    for (var i = 0; i < 50; i++) {
        var cmd_vel = geometry_msgs.Twist();
        cmd_vel.linear.z = -0.1;
        cmd_vel_pub.publish(cmd_vel);
        rate.sleep();
    }
    
    // Delete the model
    delete_model("drone");
    
    io.println("Simulation complete");
}

// Callback for model states
func on_model_states(msg: gazebo_msgs.ModelStates) -> void {
    // Find drone model
    var drone_index = -1;
    for (var i = 0; i < msg.name.length; i++) {
        if (msg.name[i] == "drone") {
            drone_index = i;
            break;
        }
    }
    
    if (drone_index >= 0) {
        var pose = msg.pose[drone_index];
        var twist = msg.twist[drone_index];
        
        io.println("Drone position: (" + 
                  pose.position.x.toString() + ", " + 
                  pose.position.y.toString() + ", " + 
                  pose.position.z.toString() + ")");
    }
}
```

### SITL (Software In The Loop) Integration

```astra
import mavlink;
import sitl;

// Start SITL simulation
func start_sitl_simulation() -> sitl.Simulation {
    // Configure simulation parameters
    var config = sitl.SimulationConfig();
    config.vehicle_type = sitl.VEHICLE_QUADCOPTER;
    config.start_latitude = 47.3977419;
    config.start_longitude = 8.5455938;
    config.start_altitude = 488.0;
    config.frame_type = sitl.FRAME_QUAD_X;
    
    // Start simulation
    var sim = sitl.Simulation(config);
    sim.start();
    
    return sim;
}

// Connect to SITL via MAVLink
func connect_to_sitl() -> mavlink.Connection {
    // Connect to SITL
    var connection = mavlink.Connection("tcp:localhost:5760");
    
    // Wait for heartbeat
    var heartbeat_received = false;
    var timeout = time.now() + 10.0;  // 10 second timeout
    
    while (!heartbeat_received && time.now() < timeout) {
        connection.update();
        
        if (connection.get_heartbeat() != null) {
            heartbeat_received = true;
        }
        
        time.sleep(100ms);
    }
    
    if (!heartbeat_received) {
        io.println("Failed to receive heartbeat from SITL");
        return null;
    }
    
    io.println("Connected to SITL");
    return connection;
}

// Run mission in SITL
func run_sitl_mission() -> void {
    // Start SITL simulation
    var sim = start_sitl_simulation();
    
    // Connect to SITL
    var connection = connect_to_sitl();
    if (connection == null) {
        sim.stop();
        return;
    }
    
    // Set up mission
    var mission = [
        (47.3977419, 8.5455938, 10.0),  // Takeoff point
        (47.3977419, 8.5456938, 15.0),  // Waypoint 1
        (47.3978419, 8.5456938, 15.0),  // Waypoint 2
        (47.3978419, 8.5455938, 10.0),  // Waypoint 3
        (47.3977419, 8.5455938, 0.0)    // Landing point
    ];
    
    // Upload mission
    upload_mission(connection, mission);
    
    // Arm and takeoff
    arm_and_takeoff(connection, 10.0);
    
    // Start mission
    start_mission(connection);
    
    // Monitor mission progress
    monitor_mission(connection);
    
    // Stop simulation
    sim.stop();
    
    io.println("SITL mission complete");
}

// Upload mission to SITL
func upload_mission(connection: mavlink.Connection, waypoints: array<(float, float, float)>) -> void {
    // Clear any existing mission
    var clear_msg = mavlink.Message();
    clear_msg.id = mavlink.MISSION_CLEAR_ALL;
    clear_msg.target_system = 1;
    clear_msg.target_component = 1;
    connection.send_message(clear_msg);
    
    // Wait for acknowledgement
    wait_for_mission_ack(connection);
    
    // Send mission count
    var count_msg = mavlink.Message();
    count_msg.id = mavlink.MISSION_COUNT;
    count_msg.target_system = 1;
    count_msg.target_component = 1;
    count_msg.count = waypoints.length as uint16;
    connection.send_message(count_msg);
    
    // Wait for mission request
    wait_for_mission_request(connection, 0);
    
    // Send mission items
    for (var i = 0; i < waypoints.length; i++) {
        var waypoint = waypoints[i];
        
        var item_msg = mavlink.Message();
        item_msg.id = mavlink.MISSION_ITEM;
        item_msg.target_system = 1;
        item_msg.target_component = 1;
        item_msg.seq = i as uint16;
        item_msg.frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT;
        
        if (i == 0) {
            item_msg.command = mavlink.MAV_CMD_NAV_TAKEOFF;
        } else if (i == waypoints.length - 1) {
            item_msg.command = mavlink.MAV_CMD_NAV_LAND;
        } else {
            item_msg.command = mavlink.MAV_CMD_NAV_WAYPOINT;
        }
        
        item_msg.current = (i == 0) ? 1 : 0;
        item_msg.autocontinue = 1;
        item_msg.param1 = 0.0;  // Hold time (seconds)
        item_msg.param2 = 0.0;  // Acceptance radius (meters)
        item_msg.param3 = 0.0;  // Pass through waypoint
        item_msg.param4 = 0.0;  // Desired yaw angle
        item_msg.x = waypoint.0;  // Latitude
        item_msg.y = waypoint.1;  // Longitude
        item_msg.z = waypoint.2;  // Altitude
        
        connection.send_message(item_msg);
        
        // Wait for next mission request or ack
        if (i < waypoints.length - 1) {
            wait_for_mission_request(connection, i + 1);
        } else {
            wait_for_mission_ack(connection);
        }
    }
    
    io.println("Mission uploaded successfully");
}

// Wait for mission request
func wait_for_mission_request(connection: mavlink.Connection, seq: int) -> void {
    var timeout = time.now() + 5.0;  // 5 second timeout
    var request_received = false;
    
    while (!request_received && time.now() < timeout) {
        connection.update();
        
        var msg = connection.get_message(mavlink.MISSION_REQUEST);
        if (msg != null && msg.seq == seq) {
            request_received = true;
        }
        
        time.sleep(10ms);
    }
    
    if (!request_received) {
        io.println("Timeout waiting for mission request " + seq.toString());
    }
}

// Wait for mission acknowledgement
func wait_for_mission_ack(connection: mavlink.Connection) -> void {
    var timeout = time.now() + 5.0;  // 5 second timeout
    var ack_received = false;
    
    while (!ack_received && time.now() < timeout) {
        connection.update();
        
        var msg = connection.get_message(mavlink.MISSION_ACK);
        if (msg != null) {
            ack_received = true;
            
            if (msg.type != mavlink.MAV_MISSION_ACCEPTED) {
                io.println("Mission not accepted: " + msg.type.toString());
            }
        }
        
        time.sleep(10ms);
    }
    
    if (!ack_received) {
        io.println("Timeout waiting for mission acknowledgement");
    }
}

// Arm and takeoff
func arm_and_takeoff(connection: mavlink.Connection, altitude: float) -> void {
    // Set mode to GUIDED
    var mode_msg = mavlink.Message();
    mode_msg.id = mavlink.SET_MODE;
    mode_msg.target_system = 1;
    mode_msg.base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mode_msg.custom_mode = 4;  // GUIDED mode
    connection.send_message(mode_msg);
    
    time.sleep(100ms);
    
    // Arm vehicle
    var arm_msg = mavlink.Message();
    arm_msg.id = mavlink.COMMAND_LONG;
    arm_msg.target_system = 1;
    arm_msg.target_component = 1;
    arm_msg.command = mavlink.MAV_CMD_COMPONENT_ARM_DISARM;
    arm_msg.param1 = 1.0;  // 1 = arm
    connection.send_message(arm_msg);
    
    // Wait for arming
    var armed = false;
    var timeout = time.now() + 10.0;  // 10 second timeout
    
    while (!armed && time.now() < timeout) {
        connection.update();
        
        var heartbeat = connection.get_heartbeat();
        if (heartbeat != null) {
            armed = (heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        }
        
        time.sleep(100ms);
    }
    
    if (!armed) {
        io.println("Failed to arm vehicle");
        return;
    }
    
    io.println("Vehicle armed");
    
    // Takeoff
    var takeoff_msg = mavlink.Message();
    takeoff_msg.id = mavlink.COMMAND_LONG;
    takeoff_msg.target_system = 1;
    takeoff_msg.target_component = 1;
    takeoff_msg.command = mavlink.MAV_CMD_NAV_TAKEOFF;
    takeoff_msg.param7 = altitude;
    connection.send_message(takeoff_msg);
    
    io.println("Takeoff command sent");
}

// Start mission
func start_mission(connection: mavlink.Connection) -> void {
    var mission_start_msg = mavlink.Message();
    mission_start_msg.id = mavlink.MISSION_START;
    mission_start_msg.target_system = 1;
    mission_start_msg.target_component = 1;
    mission_start_msg.first_item = 0;
    mission_start_msg.last_item = 0;  // 0 means all items
    connection.send_message(mission_start_msg);
    
    io.println("Mission started");
}

// Monitor mission progress
func monitor_mission(connection: mavlink.Connection) -> void {
    var mission_complete = false;
    var timeout = time.now() + 300.0;  // 5 minute timeout
    
    while (!mission_complete && time.now() < timeout) {
        connection.update();
        
        // Check mission current
        var mission_current = connection.get_message(mavlink.MISSION_CURRENT);
        if (mission_current != null) {
            io.println("Current mission item: " + mission_current.seq.toString());
        }
        
        // Check global position
        var global_pos = connection.get_message(mavlink.GLOBAL_POSITION_INT);
        if (global_pos != null) {
            var lat = global_pos.lat / 1e7;
            var lon = global_pos.lon / 1e7;
            var alt = global_pos.relative_alt / 1000.0;
            
            io.println("Position: Lat=" + lat.toString() + ", Lon=" + lon.toString() + ", Alt=" + alt.toString() + "m");
        }
        
        // Check if landed (mission complete)
        var heartbeat = connection.get_heartbeat();
        if (heartbeat != null) {
            var armed = (heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            
            if (!armed) {
                io.println("Vehicle disarmed, mission complete");
                mission_complete = true;
            }
        }
        
        time.sleep(1s);
    }
    
    if (!mission_complete) {
        io.println("Mission timeout");
    }
}
```

### AirSim Integration

```astra
import airsim;

// Connect to AirSim
func connect_to_airsim() -> airsim.Client {
    var client = airsim.Client();
    client.connect();
    
    // Enable API control
    client.enableApiControl(true);
    
    return client;
}

// Reset AirSim simulation
func reset_simulation(client: airsim.Client) -> void {
    client.reset();
    
    // Wait for reset to complete
    time.sleep(1s);
}

// Arm the vehicle
func arm_vehicle(client: airsim.Client) -> bool {
    return client.armDisarm(true);
}

// Takeoff
func takeoff(client: airsim.Client, altitude: float) -> bool {
    var result = client.takeoff(altitude);
    
    // Wait for takeoff to complete
    time.sleep(5s);
    
    return result;
}

// Move to position
func move_to_position(client: airsim.Client, x: float, y: float, z: float, velocity: float) -> bool {
    return client.moveToPosition(x, y, z, velocity);
}

// Move by velocity
func move_by_velocity(client: airsim.Client, vx: float, vy: float, vz: float, duration: float) -> bool {
    return client.moveByVelocity(vx, vy, vz, duration);
}

// Rotate to yaw
func rotate_to_yaw(client: airsim.Client, yaw: float) -> bool {
    return client.rotateToYaw(yaw);
}

// Land
func land(client: airsim.Client) -> bool {
    return client.land();
}

// Get vehicle state
func get_vehicle_state(client: airsim.Client) -> airsim.MultirotorState {
    return client.getMultirotorState();
}

// Get camera image
func get_camera_image(client: airsim.Client, camera_name: string) -> airsim.Image {
    return client.getImage(camera_name);
}

// Run AirSim mission
func run_airsim_mission() -> void {
    // Connect to AirSim
    var client = connect_to_airsim();
    
    // Reset simulation
    reset_simulation(client);
    
    // Arm vehicle
    if (!arm_vehicle(client)) {
        io.println("Failed to arm vehicle");
        return;
    }
    
    io.println("Vehicle armed");
    
    // Takeoff
    if (!takeoff(client, 10.0)) {
        io.println("Failed to takeoff");
        return;
    }
    
    io.println("Takeoff complete");
    
    // Fly a square pattern
    var waypoints = [
        (0.0, 0.0, -10.0),    // Starting point (10m altitude)
        (20.0, 0.0, -10.0),   // Forward 20m
        (20.0, 20.0, -10.0),  // Right 20m
        (0.0, 20.0, -10.0),   // Backward 20m
        (0.0, 0.0, -10.0)     // Left 20m
    ];
    
    for (var i = 0; i < waypoints.length; i++) {
        var waypoint = waypoints[i];
        
        io.println("Flying to waypoint " + (i + 1).toString());
        
        if (!move_to_position(client, waypoint.0, waypoint.1, waypoint.2, 5.0)) {
            io.println("Failed to move to waypoint");
            break;
        }
        
        // Wait for movement to complete
        time.sleep(5s);
        
        // Get current state
        var state = get_vehicle_state(client);
        io.println("Position: (" + 
                  state.position.x.toString() + ", " + 
                  state.position.y.toString() + ", " + 
                  state.position.z.toString() + ")");
        
        // Capture image from camera
        var image = get_camera_image(client, "0");
        io.println("Captured image: " + image.width.toString() + "x" + image.height.toString());
    }
    
    // Land
    io.println("Landing");
    if (!land(client)) {
        io.println("Failed to land");
        return;
    }
    
    io.println("Landing complete");
    
    // Disarm
    client.armDisarm(false);
    
    io.println("Mission complete");
}
```

## Integration with Data Processing Pipelines

### OpenCV Integration

```astra
import opencv;

// Load image
func load_image(path: string) -> opencv.Mat {
    return opencv.imread(path);
}

// Save image
func save_image(image: opencv.Mat, path: string) -> bool {
    return opencv.imwrite(path, image);
}

// Process image for object detection
func process_image_for_detection(image: opencv.Mat) -> opencv.Mat {
    // Convert to grayscale
    var gray = opencv.cvtColor(image, opencv.COLOR_BGR2GRAY);
    
    // Apply Gaussian blur
    var blurred = opencv.GaussianBlur(gray, opencv.Size(5, 5), 0);
    
    // Apply Canny edge detection
    var edges = opencv.Canny(blurred, 50, 150);
    
    return edges;
}

// Detect contours
func detect_contours(image: opencv.Mat) -> array<array<opencv.Point>> {
    var contours = array<array<opencv.Point>>[];
    var hierarchy = opencv.Mat();
    
    opencv.findContours(image.clone(), contours, hierarchy, opencv.RETR_EXTERNAL, opencv.CHAIN_APPROX_SIMPLE);
    
    return contours;
}

// Filter contours by size
func filter_contours_by_size(contours: array<array<opencv.Point>>, min_area: float) -> array<array<opencv.Point>> {
    var filtered = array<array<opencv.Point>>[];
    
    for (var i = 0; i < contours.length; i++) {
        var area = opencv.contourArea(contours[i]);
        if (area >= min_area) {
            filtered.push(contours[i]);
        }
    }
    
    return filtered;
}

// Draw contours on image
func draw_contours(image: opencv.Mat, contours: array<array<opencv.Point>>) -> opencv.Mat {
    var result = image.clone();
    opencv.drawContours(result, contours, -1, opencv.Scalar(0, 255, 0), 2);
    return result;
}

// Detect objects in image
func detect_objects(image_path: string, output_path: string) -> int {
    // Load image
    var image = load_image(image_path);
    if (image.empty()) {
        io.println("Failed to load image: " + image_path);
        return 0;
    }
    
    // Process image
    var processed = process_image_for_detection(image);
    
    // Detect contours
    var contours = detect_contours(processed);
    
    // Filter contours by size
    var filtered_contours = filter_contours_by_size(contours, 100.0);
    
    // Draw contours on original image
    var result = draw_contours(image, filtered_contours);
    
    // Save result
    if (!save_image(result, output_path)) {
        io.println("Failed to save result image: " + output_path);
    }
    
    io.println("Detected " + filtered_contours.length.toString() + " objects");
    
    return filtered_contours.length;
}

// Process video for object tracking
func process_video(video_path: string, output_path: string) -> void {
    // Open video
    var video = opencv.VideoCapture(video_path);
    if (!video.isOpened()) {
        io.println("Failed to open video: " + video_path);
        return;
    }
    
    // Get video properties
    var width = video.get(opencv.CAP_PROP_FRAME_WIDTH) as int;
    var height = video.get(opencv.CAP_PROP_FRAME_HEIGHT) as int;
    var fps = video.get(opencv.CAP_PROP_FPS);
    var frame_count = video.get(opencv.CAP_PROP_FRAME_COUNT) as int;
    
    io.println("Video properties: " + width.toString() + "x" + height.toString() + 
              ", " + fps.toString() + " fps, " + frame_count.toString() + " frames");
    
    // Create video writer
    var writer = opencv.VideoWriter(output_path, opencv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 
                                   fps, opencv.Size(width, height));
    
    // Create background subtractor
    var bg_subtractor = opencv.createBackgroundSubtractorMOG2();
    
    // Process frames
    var frame_number = 0;
    
    while (true) {
        // Read frame
        var frame = opencv.Mat();
        if (!video.read(frame) || frame.empty()) {
            break;
        }
        
        // Apply background subtraction
        var fg_mask = opencv.Mat();
        bg_subtractor.apply(frame, fg_mask);
        
        // Apply morphological operations to remove noise
        var kernel = opencv.getStructuringElement(opencv.MORPH_ELLIPSE, opencv.Size(5, 5));
        opencv.morphologyEx(fg_mask, fg_mask, opencv.MORPH_OPEN, kernel);
        
        // Find contours
        var contours = detect_contours(fg_mask);
        
        // Filter contours by size
        var filtered_contours = filter_contours_by_size(contours, 500.0);
        
        // Draw bounding boxes around detected objects
        for (var i = 0; i < filtered_contours.length; i++) {
            var rect = opencv.boundingRect(filtered_contours[i]);
            opencv.rectangle(frame, opencv.Point(rect.x, rect.y), 
                           opencv.Point(rect.x + rect.width, rect.y + rect.height), 
                           opencv.Scalar(0, 255, 0), 2);
        }
        
        // Write frame to output video
        writer.write(frame);
        
        // Update progress
        frame_number++;
        if (frame_number % 100 == 0) {
            io.println("Processed " + frame_number.toString() + " frames");
        }
    }
    
    // Release resources
    video.release();
    writer.release();
    
    io.println("Video processing complete");
}
```

### TensorFlow Integration

```astra
import tensorflow as tf;

// Load TensorFlow model
func load_model(model_path: string) -> tf.Model {
    return tf.keras.models.load_model(model_path);
}

// Preprocess image for model input
func preprocess_image(image_path: string, target_size: (int, int)) -> tf.Tensor {
    // Load image
    var img = tf.keras.preprocessing.image.load_img(image_path, target_size=target_size);
    
    // Convert to array
    var img_array = tf.keras.preprocessing.image.img_to_array(img);
    
    // Expand dimensions to create batch
    var img_batch = tf.expand_dims(img_array, 0);
    
    // Preprocess for model
    var preprocessed = tf.keras.applications.mobilenet_v2.preprocess_input(img_batch);
    
    return preprocessed;
}

// Perform object detection
func detect_objects_tf(model: tf.Model, image_path: string, class_names: array<string>) -> array<(string, float, (float, float, float, float))> {
    // Preprocess image
    var input_tensor = preprocess_image(image_path, (224, 224));
    
    // Run inference
    var predictions = model.predict(input_tensor);
    
    // Parse predictions
    var boxes = predictions[0];
    var scores = predictions[1];
    var classes = predictions[2];
    var num_detections = predictions[3];
    
    // Convert to numpy arrays
    boxes = boxes.numpy()[0];
    scores = scores.numpy()[0];
    classes = classes.numpy()[0].astype(np.int32);
    num_detections = int(num_detections.numpy()[0]);
    
    // Create results array
    var results = array<(string, float, (float, float, float, float))>[];
    
    // Add detections with score > 0.5
    for (var i = 0; i < num_detections; i++) {
        if (scores[i] > 0.5) {
            var class_id = classes[i];
            var class_name = class_names[class_id];
            var score = scores[i];
            var box = (boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]);  // [y1, x1, y2, x2]
            
            results.push((class_name, score, box));
        }
    }
    
    return results;
}

// Train a simple model
func train_simple_model(x_train: tf.Tensor, y_train: tf.Tensor, x_val: tf.Tensor, y_val: tf.Tensor) -> tf.Model {
    // Create sequential model
    var model = tf.keras.Sequential();
    
    // Add layers
    model.add(tf.keras.layers.Conv2D(32, (3, 3), activation='relu', input_shape=(32, 32, 3)));
    model.add(tf.keras.layers.MaxPooling2D((2, 2)));
    model.add(tf.keras.layers.Conv2D(64, (3, 3), activation='relu'));
    model.add(tf.keras.layers.MaxPooling2D((2, 2)));
    model.add(tf.keras.layers.Conv2D(64, (3, 3), activation='relu'));
    model.add(tf.keras.layers.Flatten());
    model.add(tf.keras.layers.Dense(64, activation='relu'));
    model.add(tf.keras.layers.Dense(10, activation='softmax'));
    
    // Compile model
    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    );
    
    // Train model
    model.fit(
        x_train, y_train,
        epochs=10,
        validation_data=(x_val, y_val)
    );
    
    return model;
}

// Save model
func save_model(model: tf.Model, path: string) -> void {
    model.save(path);
}

// Evaluate model
func evaluate_model(model: tf.Model, x_test: tf.Tensor, y_test: tf.Tensor) -> (float, float) {
    var result = model.evaluate(x_test, y_test);
    return (result[0], result[1]);  // loss, accuracy
}

// Use model for prediction
func predict(model: tf.Model, input_data: tf.Tensor) -> tf.Tensor {
    return model.predict(input_data);
}

// Example: Train and use a model for image classification
func image_classification_example() -> void {
    // Load CIFAR-10 dataset
    var (x_train, y_train), (x_test, y_test) = tf.keras.datasets.cifar10.load_data();
    
    // Normalize pixel values
    x_train = x_train / 255.0;
    x_test = x_test / 255.0;
    
    // Split test set into validation and test
    var x_val = x_test[:5000];
    var y_val = y_test[:5000];
    x_test = x_test[5000:];
    y_test = y_test[5000:];
    
    // Train model
    var model = train_simple_model(x_train, y_train, x_val, y_val);
    
    // Evaluate model
    var (loss, accuracy) = evaluate_model(model, x_test, y_test);
    io.println("Test loss: " + loss.toString());
    io.println("Test accuracy: " + accuracy.toString());
    
    // Save model
    save_model(model, "cifar10_model");
    
    // Class names
    var class_names = ["airplane", "automobile", "bird", "cat", "deer", "dog", "frog", "horse", "ship", "truck"];
    
    // Make predictions on a few test images
    var predictions = predict(model, x_test[:5]);
    
    for (var i = 0; i < 5; i++) {
        var predicted_class = tf.argmax(predictions[i]).numpy();
        var true_class = y_test[i][0];
        
        io.println("Image " + i.toString() + ":");
        io.println("  Predicted: " + class_names[predicted_class]);
        io.println("  Actual: " + class_names[true_class]);
    }
}
```

### Point Cloud Processing

```astra
import pcl;

// Load point cloud from file
func load_point_cloud(path: string) -> pcl.PointCloud<pcl.PointXYZ> {
    var cloud = pcl.PointCloud<pcl.PointXYZ>();
    pcl.io.loadPCDFile(path, cloud);
    return cloud;
}

// Save point cloud to file
func save_point_cloud(cloud: pcl.PointCloud<pcl.PointXYZ>, path: string) -> void {
    pcl.io.savePCDFile(path, cloud);
}

// Filter point cloud using voxel grid
func voxel_grid_filter(cloud: pcl.PointCloud<pcl.PointXYZ>, leaf_size: float) -> pcl.PointCloud<pcl.PointXYZ> {
    var filtered = pcl.PointCloud<pcl.PointXYZ>();
    var filter = pcl.VoxelGrid<pcl.PointXYZ>();
    
    filter.setInputCloud(cloud);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(filtered);
    
    return filtered;
}

// Remove outliers using statistical outlier removal
func remove_outliers(cloud: pcl.PointCloud<pcl.PointXYZ>, mean_k: int, std_dev_mul: float) -> pcl.PointCloud<pcl.PointXYZ> {
    var filtered = pcl.PointCloud<pcl.PointXYZ>();
    var filter = pcl.StatisticalOutlierRemoval<pcl.PointXYZ>();
    
    filter.setInputCloud(cloud);
    filter.setMeanK(mean_k);
    filter.setStddevMulThresh(std_dev_mul);
    filter.filter(filtered);
    
    return filtered;
}

// Segment plane from point cloud
func segment_plane(cloud: pcl.PointCloud<pcl.PointXYZ>, distance_threshold: float) -> (pcl.PointCloud<pcl.PointXYZ>, pcl.PointCloud<pcl.PointXYZ>, pcl.ModelCoefficients) {
    var inliers = pcl.PointIndices();
    var coefficients = pcl.ModelCoefficients();
    var plane_cloud = pcl.PointCloud<pcl.PointXYZ>();
    var non_plane_cloud = pcl.PointCloud<pcl.PointXYZ>();
    
    // Create segmentation object
    var seg = pcl.SACSegmentation<pcl.PointXYZ>();
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl.SACMODEL_PLANE);
    seg.setMethodType(pcl.SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    
    // Segment the largest planar component
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);
    
    // Extract the plane
    var extract = pcl.ExtractIndices<pcl.PointXYZ>();
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    
    // Get points in the plane
    extract.setNegative(false);
    extract.filter(plane_cloud);
    
    // Get points not in the plane
    extract.setNegative(true);
    extract.filter(non_plane_cloud);
    
    return (plane_cloud, non_plane_cloud, coefficients);
}

// Cluster point cloud
func cluster_point_cloud(cloud: pcl.PointCloud<pcl.PointXYZ>, cluster_tolerance: float, min_cluster_size: int, max_cluster_size: int) -> array<pcl.PointCloud<pcl.PointXYZ>> {
    var clusters = array<pcl.PointCloud<pcl.PointXYZ>>[];
    
    // Create KdTree for search
    var tree = pcl.search.KdTree<pcl.PointXYZ>();
    tree.setInputCloud(cloud);
    
    // Create cluster extraction object
    var ec = pcl.EuclideanClusterExtraction<pcl.PointXYZ>();
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    
    // Extract clusters
    var cluster_indices = array<pcl.PointIndices>[];
    ec.extract(cluster_indices);
    
    // Create point cloud for each cluster
    for (var i = 0; i < cluster_indices.length; i++) {
        var cluster = pcl.PointCloud<pcl.PointXYZ>();
        
        for (var j = 0; j < cluster_indices[i].indices.length; j++) {
            cluster.push_back(cloud.points[cluster_indices[i].indices[j]]);
        }
        
        cluster.width = cluster.points.length;
        cluster.height = 1;
        cluster.is_dense = true;
        
        clusters.push(cluster);
    }
    
    return clusters;
}

// Compute normals for point cloud
func compute_normals(cloud: pcl.PointCloud<pcl.PointXYZ>, search_radius: float) -> pcl.PointCloud<pcl.Normal> {
    var normals = pcl.PointCloud<pcl.Normal>();
    
    // Create normal estimation object
    var ne = pcl.NormalEstimation<pcl.PointXYZ, pcl.Normal>();
    ne.setInputCloud(cloud);
    
    // Create KdTree for search
    var tree = pcl.search.KdTree<pcl.PointXYZ>();
    ne.setSearchMethod(tree);
    
    // Set search radius
    ne.setRadiusSearch(search_radius);
    
    // Compute normals
    ne.compute(normals);
    
    return normals;
}

// Example: Process LiDAR point cloud
func process_lidar_point_cloud(input_path: string, output_path: string) -> void {
    // Load point cloud
    var cloud = load_point_cloud(input_path);
    io.println("Loaded point cloud with " + cloud.points.length.toString() + " points");
    
    // Downsample using voxel grid
    var filtered_cloud = voxel_grid_filter(cloud, 0.01);  // 1cm leaf size
    io.println("Filtered point cloud has " + filtered_cloud.points.length.toString() + " points");
    
    // Remove outliers
    var clean_cloud = remove_outliers(filtered_cloud, 50, 1.0);
    io.println("Clean point cloud has " + clean_cloud.points.length.toString() + " points");
    
    // Segment ground plane
    var (ground_cloud, objects_cloud, plane_coefficients) = segment_plane(clean_cloud, 0.02);  // 2cm threshold
    io.println("Ground plane has " + ground_cloud.points.length.toString() + " points");
    io.println("Objects have " + objects_cloud.points.length.toString() + " points");
    
    // Cluster objects
    var clusters = cluster_point_cloud(objects_cloud, 0.05, 100, 25000);  // 5cm cluster tolerance, min 100 points, max 25000 points
    io.println("Found " + clusters.length.toString() + " clusters");
    
    // Save processed point cloud
    save_point_cloud(objects_cloud, output_path);
    io.println("Saved processed point cloud to " + output_path);
    
    // Save individual clusters
    for (var i = 0; i < clusters.length; i++) {
        var cluster_path = output_path.replace(".pcd", "_cluster_" + i.toString() + ".pcd");
        save_point_cloud(clusters[i], cluster_path);
        io.println("Saved cluster " + i.toString() + " with " + clusters[i].points.length.toString() + " points to " + cluster_path);
    }
}
```

## Integration with Cloud Services

### AWS Integration

```astra
import aws;

// Initialize AWS credentials
func initialize_aws() -> void {
    aws.init();
    
    // Set credentials (from environment variables or config file)
    aws.set_region("us-west-2");
}

// Upload file to S3
func upload_to_s3(local_path: string, bucket: string, key: string) -> bool {
    var s3 = aws.S3();
    
    io.println("Uploading " + local_path + " to s3://" + bucket + "/" + key);
    
    var result = s3.upload_file(local_path, bucket, key);
    
    if (result) {
        io.println("Upload successful");
    } else {
        io.println("Upload failed: " + s3.get_last_error());
    }
    
    return result;
}

// Download file from S3
func download_from_s3(bucket: string, key: string, local_path: string) -> bool {
    var s3 = aws.S3();
    
    io.println("Downloading s3://" + bucket + "/" + key + " to " + local_path);
    
    var result = s3.download_file(bucket, key, local_path);
    
    if (result) {
        io.println("Download successful");
    } else {
        io.println("Download failed: " + s3.get_last_error());
    }
    
    return result;
}

// List objects in S3 bucket
func list_s3_objects(bucket: string, prefix: string = "") -> array<string> {
    var s3 = aws.S3();
    
    io.println("Listing objects in s3://" + bucket + "/" + prefix);
    
    var objects = s3.list_objects(bucket, prefix);
    
    for (var i = 0; i < objects.length; i++) {
        io.println("  " + objects[i]);
    }
    
    return objects;
}

// Send message to SQS queue
func send_to_sqs(queue_url: string, message: string) -> bool {
    var sqs = aws.SQS();
    
    io.println("Sending message to " + queue_url);
    
    var result = sqs.send_message(queue_url, message);
    
    if (result) {
        io.println("Message sent successfully");
    } else {
        io.println("Failed to send message: " + sqs.get_last_error());
    }
    
    return result;
}

// Receive messages from SQS queue
func receive_from_sqs(queue_url: string, max_messages: int = 10) -> array<aws.SQSMessage> {
    var sqs = aws.SQS();
    
    io.println("Receiving messages from " + queue_url);
    
    var messages = sqs.receive_messages(queue_url, max_messages);
    
    io.println("Received " + messages.length.toString() + " messages");
    
    return messages;
}

// Delete message from SQS queue
func delete_from_sqs(queue_url: string, receipt_handle: string) -> bool {
    var sqs = aws.SQS();
    
    io.println("Deleting message from " + queue_url);
    
    var result = sqs.delete_message(queue_url, receipt_handle);
    
    if (result) {
        io.println("Message deleted successfully");
    } else {
        io.println("Failed to delete message: " + sqs.get_last_error());
    }
    
    return result;
}

// Invoke Lambda function
func invoke_lambda(function_name: string, payload: string) -> string {
    var lambda = aws.Lambda();
    
    io.println("Invoking Lambda function: " + function_name);
    
    var response = lambda.invoke(function_name, payload);
    
    if (response != null) {
        io.println("Lambda invocation successful");
        return response;
    } else {
        io.println("Lambda invocation failed: " + lambda.get_last_error());
        return "";
    }
}

// Example: Process and upload telemetry data
func process_and_upload_telemetry(telemetry_file: string) -> void {
    // Initialize AWS
    initialize_aws();
    
    // Read telemetry data
    var telemetry_data = file.read_text(telemetry_file);
    
    // Process telemetry data
    var processed_data = process_telemetry(telemetry_data);
    
    // Save processed data to temporary file
    var temp_file = system.get_temp_directory() + "/processed_telemetry.json";
    file.write_text(temp_file, processed_data);
    
    // Upload to S3
    var timestamp = time.now().format("%Y%m%d_%H%M%S");
    var s3_key = "telemetry/" + timestamp + ".json";
    upload_to_s3(temp_file, "drone-telemetry-bucket", s3_key);
    
    // Send notification to SQS
    var message = json.stringify({
        "type": "telemetry",
        "timestamp": timestamp,
        "s3_key": s3_key
    });
    send_to_sqs("https://sqs.us-west-2.amazonaws.com/123456789012/telemetry-queue", message);
    
    // Clean up temporary file
    file.delete(temp_file);
}

// Process telemetry data
func process_telemetry(telemetry_data: string) -> string {
    // Parse JSON
    var data = json.parse(telemetry_data);
    
    // Extract relevant fields
    var processed = Map<string, any>();
    processed.set("timestamp", data.get("timestamp"));
    processed.set("device_id", data.get("device_id"));
    
    // Extract position
    var position = Map<string, any>();
    position.set("latitude", data.get("latitude"));
    position.set("longitude", data.get("longitude"));
    position.set("altitude", data.get("altitude"));
    processed.set("position", position);
    
    // Extract attitude
    var attitude = Map<string, any>();
    attitude.set("roll", data.get("roll"));
    attitude.set("pitch", data.get("pitch"));
    attitude.set("yaw", data.get("yaw"));
    processed.set("attitude", attitude);
    
    // Extract battery
    var battery = Map<string, any>();
    battery.set("voltage", data.get("battery_voltage"));
    battery.set("current", data.get("battery_current"));
    battery.set("level", data.get("battery_level"));
    processed.set("battery", battery);
    
    // Convert back to JSON
    return json.stringify(processed);
}
```

### Azure Integration

```astra
import azure;

// Initialize Azure credentials
func initialize_azure() -> void {
    azure.init();
    
    // Set credentials (from environment variables or config file)
    azure.set_subscription_id(system.get_env("AZURE_SUBSCRIPTION_ID"));
}

// Upload blob to Azure Storage
func upload_to_blob_storage(local_path: string, container: string, blob_name: string) -> bool {
    var storage = azure.BlobStorage();
    
    io.println("Uploading " + local_path + " to " + container + "/" + blob_name);
    
    var result = storage.upload_blob(local_path, container, blob_name);
    
    if (result) {
        io.println("Upload successful");
    } else {
        io.println("Upload failed: " + storage.get_last_error());
    }
    
    return result;
}

// Download blob from Azure Storage
func download_from_blob_storage(container: string, blob_name: string, local_path: string) -> bool {
    var storage = azure.BlobStorage();
    
    io.println("Downloading " + container + "/" + blob_name + " to " + local_path);
    
    var result = storage.download_blob(container, blob_name, local_path);
    
    if (result) {
        io.println("Download successful");
    } else {
        io.println("Download failed: " + storage.get_last_error());
    }
    
    return result;
}

// List blobs in container
func list_blobs(container: string, prefix: string = "") -> array<string> {
    var storage = azure.BlobStorage();
    
    io.println("Listing blobs in " + container + "/" + prefix);
    
    var blobs = storage.list_blobs(container, prefix);
    
    for (var i = 0; i < blobs.length; i++) {
        io.println("  " + blobs[i]);
    }
    
    return blobs;
}

// Send message to Azure Service Bus
func send_to_service_bus(queue_name: string, message: string) -> bool {
    var service_bus = azure.ServiceBus();
    
    io.println("Sending message to " + queue_name);
    
    var result = service_bus.send_message(queue_name, message);
    
    if (result) {
        io.println("Message sent successfully");
    } else {
        io.println("Failed to send message: " + service_bus.get_last_error());
    }
    
    return result;
}

// Receive messages from Azure Service Bus
func receive_from_service_bus(queue_name: string, max_messages: int = 10) -> array<azure.ServiceBusMessage> {
    var service_bus = azure.ServiceBus();
    
    io.println("Receiving messages from " + queue_name);
    
    var messages = service_bus.receive_messages(queue_name, max_messages);
    
    io.println("Received " + messages.length.toString() + " messages");
    
    return messages;
}

// Complete message from Azure Service Bus
func complete_service_bus_message(message: azure.ServiceBusMessage) -> bool {
    var service_bus = azure.ServiceBus();
    
    io.println("Completing message");
    
    var result = service_bus.complete_message(message);
    
    if (result) {
        io.println("Message completed successfully");
    } else {
        io.println("Failed to complete message: " + service_bus.get_last_error());
    }
    
    return result;
}

// Invoke Azure Function
func invoke_azure_function(function_url: string, payload: string) -> string {
    var functions = azure.Functions();
    
    io.println("Invoking Azure Function: " + function_url);
    
    var response = functions.invoke(function_url, payload);
    
    if (response != null) {
        io.println("Function invocation successful");
        return response;
    } else {
        io.println("Function invocation failed: " + functions.get_last_error());
        return "";
    }
}

// Example: Process and upload image data
func process_and_upload_image(image_file: string) -> void {
    // Initialize Azure
    initialize_azure();
    
    // Process image
    var processed_image = process_image(image_file);
    
    // Save processed image to temporary file
    var temp_file = system.get_temp_directory() + "/processed_image.jpg";
    file.write_binary(temp_file, processed_image);
    
    // Upload to Blob Storage
    var timestamp = time.now().format("%Y%m%d_%H%M%S");
    var blob_name = "images/" + timestamp + ".jpg";
    upload_to_blob_storage(temp_file, "drone-images", blob_name);
    
    // Send notification to Service Bus
    var message = json.stringify({
        "type": "image",
        "timestamp": timestamp,
        "blob_name": blob_name
    });
    send_to_service_bus("image-processing-queue", message);
    
    // Clean up temporary file
    file.delete(temp_file);
}

// Process image
func process_image(image_file: string) -> array<byte> {
    // Load image
    var image_data = file.read_binary(image_file);
    
    // Process image (placeholder for actual image processing)
    // In a real implementation, this would use image processing libraries
    
    return image_data;
}
```

## Case Studies

### Case Study 1: Integrating ASTRA with ROS and OpenCV for Drone Vision

```astra
import ros;
import opencv;
import geometry_msgs;
import sensor_msgs;
import std_msgs;

// Global variables
var image_pub: ros.Publisher<sensor_msgs.Image>;
var detection_pub: ros.Publisher<geometry_msgs.PoseArray>;
var bridge = opencv.CvBridge();

// Initialize ROS node
func initialize_ros() -> void {
    ros.init("astra_vision_node");
    
    // Create publishers
    image_pub = ros.create_publisher<sensor_msgs.Image>("/vision/processed_image", 1);
    detection_pub = ros.create_publisher<geometry_msgs.PoseArray>("/vision/detections", 1);
    
    // Create subscribers
    ros.create_subscriber<sensor_msgs.Image>("/camera/image_raw", 1, image_callback);
    
    io.println("ROS node initialized");
}

// Image callback
func image_callback(msg: sensor_msgs.Image) -> void {
    try {
        // Convert ROS image to OpenCV image
        var cv_image = bridge.imgmsg_to_cv2(msg, "bgr8");
        
        // Process image
        var processed_image = process_image(cv_image);
        
        // Detect objects
        var detections = detect_objects(processed_image);
        
        // Publish processed image
        var processed_msg = bridge.cv2_to_imgmsg(processed_image, "bgr8");
        processed_msg.header = msg.header;
        image_pub.publish(processed_msg);
        
        // Publish detections
        var detection_msg = geometry_msgs.PoseArray();
        detection_msg.header = msg.header;
        
        for (var i = 0; i < detections.length; i++) {
            var pose = geometry_msgs.Pose();
            pose.position.x = detections[i].x;
            pose.position.y = detections[i].y;
            pose.position.z = detections[i].z;
            pose.orientation.w = 1.0;
            
            detection_msg.poses.push(pose);
        }
        
        detection_pub.publish(detection_msg);
    } catch (e) {
        io.println("Error processing image: " + e.toString());
    }
}

// Process image
func process_image(image: opencv.Mat) -> opencv.Mat {
    // Convert to HSV
    var hsv = opencv.cvtColor(image, opencv.COLOR_BGR2HSV);
    
    // Define range for red color
    var lower_red1 = opencv.Scalar(0, 100, 100);
    var upper_red1 = opencv.Scalar(10, 255, 255);
    var lower_red2 = opencv.Scalar(160, 100, 100);
    var upper_red2 = opencv.Scalar(179, 255, 255);
    
    // Threshold the HSV image to get only red colors
    var mask1 = opencv.inRange(hsv, lower_red1, upper_red1);
    var mask2 = opencv.inRange(hsv, lower_red2, upper_red2);
    var mask = opencv.add(mask1, mask2);
    
    // Bitwise-AND mask and original image
    var result = opencv.Mat();
    opencv.bitwise_and(image, image, result, mask);
    
    return result;
}

// Detect objects
func detect_objects(image: opencv.Mat) -> array<opencv.Point3f> {
    var detections = array<opencv.Point3f>[];
    
    // Convert to grayscale
    var gray = opencv.cvtColor(image, opencv.COLOR_BGR2GRAY);
    
    // Apply Gaussian blur
    var blurred = opencv.GaussianBlur(gray, opencv.Size(5, 5), 0);
    
    // Apply Canny edge detection
    var edges = opencv.Canny(blurred, 50, 150);
    
    // Find contours
    var contours = array<array<opencv.Point>>();
    var hierarchy = opencv.Mat();
    opencv.findContours(edges, contours, hierarchy, opencv.RETR_EXTERNAL, opencv.CHAIN_APPROX_SIMPLE);
    
    // Process contours
    for (var i = 0; i < contours.length; i++) {
        var area = opencv.contourArea(contours[i]);
        
        // Filter small contours
        if (area > 100) {
            // Get bounding rectangle
            var rect = opencv.boundingRect(contours[i]);
            
            // Get center of contour
            var moments = opencv.moments(contours[i]);
            var cx = moments.m10 / moments.m00;
            var cy = moments.m01 / moments.m00;
            
            // Add detection (x, y, area)
            detections.push(opencv.Point3f(cx, cy, area));
            
            // Draw rectangle on image
            opencv.rectangle(image, opencv.Point(rect.x, rect.y), 
                           opencv.Point(rect.x + rect.width, rect.y + rect.height), 
                           opencv.Scalar(0, 255, 0), 2);
        }
    }
    
    return detections;
}

// Main function
func main() {
    // Initialize ROS
    initialize_ros();
    
    // Spin
    ros.spin();
}
```

### Case Study 2: Integrating ASTRA with MAVLink and TensorFlow for Autonomous Navigation

```astra
import mavlink;
import tensorflow as tf;
import time;
import io;
import file;

// Global variables
var connection: mavlink.Connection;
var model: tf.Model;
var running: bool = true;

// Initialize MAVLink connection
func initialize_mavlink() -> bool {
    // Connect to flight controller
    connection = mavlink.Connection("udp:localhost:14550");
    
    // Wait for heartbeat
    var heartbeat_received = false;
    var timeout = time.now() + 10.0;  // 10 second timeout
    
    while (!heartbeat_received && time.now() < timeout) {
        connection.update();
        
        if (connection.get_heartbeat() != null) {
            heartbeat_received = true;
        }
        
        time.sleep(100ms);
    }
    
    if (!heartbeat_received) {
        io.println("Failed to receive heartbeat");
        return false;
    }
    
    io.println("MAVLink connection established");
    return true;
}

// Load TensorFlow model
func load_model(model_path: string) -> bool {
    try {
        model = tf.keras.models.load_model(model_path);
        io.println("Model loaded successfully");
        return true;
    } catch (e) {
        io.println("Failed to load model: " + e.toString());
        return false;
    }
}

// Process image for obstacle detection
func process_image(image_data: array<byte>) -> array<(string, float, (float, float, float, float))> {
    // Convert image data to tensor
    var image = tf.image.decode_jpeg(image_data, channels=3);
    var input_tensor = tf.image.resize(image, [224, 224]);
    input_tensor = tf.expand_dims(input_tensor, 0);
    input_tensor = tf.keras.applications.mobilenet_v2.preprocess_input(input_tensor);
    
    // Run inference
    var predictions = model.predict(input_tensor);
    
    // Parse predictions
    var boxes = predictions[0].numpy()[0];
    var scores = predictions[1].numpy()[0];
    var classes = predictions[2].numpy()[0].astype(np.int32);
    var num_detections = int(predictions[3].numpy()[0]);
    
    // Create results array
    var results = array<(string, float, (float, float, float, float))>[];
    
    // Class names (simplified)
    var class_names = ["background", "person", "car", "tree", "building"];
    
    // Add detections with score > 0.5
    for (var i = 0; i < num_detections; i++) {
        if (scores[i] > 0.5) {
            var class_id = classes[i];
            var class_name = class_id < class_names.length ? class_names[class_id] : "unknown";
            var score = scores[i];
            var box = (boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]);  // [y1, x1, y2, x2]
            
            results.push((class_name, score, box));
        }
    }
    
    return results;
}

// Plan path around obstacles
func plan_path(current_position: Vector3, target_position: Vector3, obstacles: array<(string, float, (float, float, float, float))>) -> array<Vector3> {
    // Simple path planning: if obstacles detected, go higher
    var path = array<Vector3>[];
    
    // Add current position
    path.push(current_position);
    
    // Check if obstacles are in the way
    var has_obstacles = false;
    for (var i = 0; i < obstacles.length; i++) {
        var obstacle = obstacles[i];
        var box = obstacle.2;
        var box_center_x = (box.1 + box.3) / 2.0;
        var box_center_y = (box.0 + box.2) / 2.0;
        
        // Check if obstacle is in the path (simplified)
        if (box_center_x > 0.3 && box_center_x < 0.7 && box_center_y > 0.3 && box_center_y < 0.7) {
            has_obstacles = true;
            break;
        }
    }
    
    // If obstacles detected, add waypoint with higher altitude
    if (has_obstacles) {
        var midpoint = Vector3(
            (current_position.x + target_position.x) / 2.0,
            (current_position.y + target_position.y) / 2.0,
            current_position.z + 5.0  // Go 5 meters higher
        );
        path.push(midpoint);
    }
    
    // Add target position
    path.push(target_position);
    
    return path;
}

// Send position target
func send_position_target(position: Vector3) -> void {
    var msg = mavlink.Message();
    msg.id = mavlink.SET_POSITION_TARGET_LOCAL_NED;
    msg.time_boot_ms = time.millis();
    msg.target_system = 1;
    msg.target_component = 1;
    msg.coordinate_frame = mavlink.MAV_FRAME_LOCAL_NED;
    msg.type_mask = 0b0000111111111000;  // Use position
    msg.x = position.x;
    msg.y = position.y;
    msg.z = position.z;
    msg.vx = 0.0;
    msg.vy = 0.0;
    msg.vz = 0.0;
    msg.afx = 0.0;
    msg.afy = 0.0;
    msg.afz = 0.0;
    msg.yaw = 0.0;
    msg.yaw_rate = 0.0;
    
    connection.send_message(msg);
}

// Get current position
func get_current_position() -> Vector3 {
    var local_position = connection.get_message(mavlink.LOCAL_POSITION_NED);
    
    if (local_position != null) {
        return Vector3(local_position.x, local_position.y, -local_position.z);  // Note: z is inverted in NED
    }
    
    return Vector3(0.0, 0.0, 0.0);
}

// Get camera image
func get_camera_image() -> array<byte> {
    // In a real system, this would get the image from a camera
    // For this example, we'll load a test image
    return file.read_binary("test_image.jpg");
}

// Main control loop
func main() {
    // Initialize MAVLink
    if (!initialize_mavlink()) {
        io.println("Failed to initialize MAVLink");
        return;
    }
    
    // Load model
    if (!load_model("obstacle_detection_model")) {
        io.println("Failed to load model");
        return;
    }
    
    // Set target position
    var target_position = Vector3(100.0, 0.0, 20.0);  // 100m north, 20m altitude
    
    // Main loop
    while (running) {
        // Update MAVLink
        connection.update();
        
        // Get current position
        var current_position = get_current_position();
        
        // Get camera image
        var image_data = get_camera_image();
        
        // Process image for obstacle detection
        var obstacles = process_image(image_data);
        
        // Plan path around obstacles
        var path = plan_path(current_position, target_position, obstacles);
        
        // Send position target (next waypoint in path)
        if (path.length > 1) {
            send_position_target(path[1]);
        }
        
        // Check if reached target
        var distance_to_target = current_position.distance_to(target_position);
        if (distance_to_target < 1.0) {
            io.println("Reached target position");
            running = false;
        }
        
        // Sleep
        time.sleep(100ms);
    }
}
```

## Conclusion

This guide provides a comprehensive overview of integrating ASTRA with various systems, frameworks, and programming languages commonly used in aerospace and UAV applications. By leveraging ASTRA's foreign function interface, interoperability features, and standardized communication protocols, developers can create seamless integrations between ASTRA code and existing systems.

For platform-specific details and advanced topics, refer to the following resources:

- [ASTRA Foreign Function Interface Reference](ffi_reference.md)
- [ASTRA ROS Integration Guide](ros_integration.md)
- [ASTRA MAVLink Integration Guide](mavlink_integration.md)
- [ASTRA Cloud Integration Guide](cloud_integration.md)