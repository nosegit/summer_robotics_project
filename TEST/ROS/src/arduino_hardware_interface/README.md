# Arduino Hardware Interface for ROS 2 Control (`arduino_hardware_interface`)

A robust, production-grade ROS 2 Control hardware interface plugin that bridges ROS 2 and an Arduino microcontroller via USB serial communication using `libserial`. It supports differential drive motors (wheels), a servo motor, an IMU (orientation, angular velocity, linear acceleration), and infrared (IR) distance sensors.

---

## Features

- **System Interface Plugin**: Fully compatible with `ros2_control` lifecycle management (`on_init`, `on_configure`, `on_cleanup`, `read`, `write`).
- **Multi-Sensor Support**:
  - Wheels & Servo (Velocity/Position commands and feedback via `joint_state_broadcaster`).
  - 9-DoF IMU sensor feedback via `imu_sensor_broadcaster`.
  - Dual IR Distance sensors via `range_sensor_broadcaster`.
- **Threaded Serial Architecture**: Asynchronous background thread for non-blocking serial communication, ensuring real-time loop stability at 50 Hz.

---

## Prerequisites

- **ROS 2 Distribution**: Jazzy Jalisco (or compatible versions like Humble/Iron with minor adjustments).
- **Dependencies**:
  - `rclcpp`
  - `rclcpp_lifecycle`
  - `hardware_interface`
  - `pluginlib`
  - `libserial` (System package via `pkg-config`)

To install `libserial` on Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install libserial-dev

```

---

## Package Directory Structure

```text
arduino_hardware_interface/
├── CMakeLists.txt
├── package.xml
├── arduino_hardware_interface.xml
├── test_file
│   ├── test_arduino.urdf
│   └── test_controllers.yaml
├── include/
│   └── arduino_hardware_interface/
│       ├── arduino_node.hpp
│       └── arduino_hardware_interface.hpp
└── src/
    ├── arduino_node.cpp
    └── arduino_hardware_interface.cpp

```

---

## Installation & Building

1. Navigate to your ROS 2 workspace source directory:
```bash
cd ~/ros2_ws/src

```


2. Clone or create your package following standard layout (`ros2 pkg create arduino_hardware_interface --build-type ament_cmake --dependencies rclcpp rclcpp_lifecycle hardware_interface pluginlib`).
3. Populate the source files, headers, `CMakeLists.txt`, `package.xml`, and the plugin XML definition (`arduino_hardware_interface.xml`).
4. Build the package using `colcon`:
```bash
cd ~/ros2_ws
colcon build --packages-select arduino_hardware_interface
source install/setup.bash

```



---

## Testing Guide

Follow these steps to verify that the hardware interface plugin loads, connects to your Arduino, and streams sensor and actuator feedback.

### Step 1: Create the Test URDF (`test_arduino.urdf`)

Create a test URDF file in your workspace directory that defines the robot joints and maps them to the plugin parameters:

```xml
<?xml version="1.0"?>
<robot name="test_arduino_robot">
  <!-- Dummy Links -->
  <link name="base_link"/>
  <link name="wheel1_link"/>
  <link name="wheel2_link"/>
  <link name="servo_link"/>

  <!-- Physical URDF Joints -->
  <joint name="wheel1_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel1_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel2_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel2_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="servo_joint" type="revolute">
    <parent link="base_link"/>
    <child link="servo_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>

  <!-- ros2_control Hardware Mapping -->
  <ros2_control name="ArduinoHardware" type="system">
    <hardware>
      <plugin>arduino_hardware_interface/ArduinoHardwareInterface</plugin>
      <param name="serial_port">/dev/ttyACM0</param>
      <param name="baudrate">115200</param>
    </hardware>

    <!-- Wheels -->
    <joint name="wheel1_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="wheel2_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <!-- Servo -->
    <joint name="servo_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>

    <!-- Sensors -->
    <sensor name="imu_sensor">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
    </sensor>

    <sensor name="ir_sensor_1">
      <state_interface name="range"/>
    </sensor>

    <sensor name="ir_sensor_2">
      <state_interface name="range"/>
    </sensor>
  </ros2_control>
</robot>

```

### Step 2: Create Controller Configuration (`test_controllers.yaml`)

Create a configuration file to match your Arduino's 50 Hz update rate and declare all required broadcasters:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    imu_sensor_broadcaster:
      type: imu_sensor_broadcaster/IMUSensorBroadcaster

    ir_sensor_1_broadcaster:
      type: range_sensor_broadcaster/RangeSensorBroadcaster

    ir_sensor_2_broadcaster:
      type: range_sensor_broadcaster/RangeSensorBroadcaster

imu_sensor_broadcaster:
  ros__parameters:
    sensor_name: "imu_sensor"
    frame_id: "imu_sensor"

ir_sensor_1_broadcaster:
  ros__parameters:
    sensor_name: "ir_sensor_1"
    frame_id: "ir_sensor_1"
    radiation_type: 1 # 1 = Infrared

ir_sensor_2_broadcaster:
  ros__parameters:
    sensor_name: "ir_sensor_2"
    frame_id: "ir_sensor_2"
    radiation_type: 1 # 1 = Infrared

```

---

### Step 3: Run the Test Sequence

You will need **3 separate terminals** (make sure to run `source ~/ros2_ws/install/setup.bash` in each).

#### Terminal 1: Run Robot State Publisher

Broadcasts the robot description to the ROS 2 network:

```bash
ros2 run robot_state_publisher robot_state_publisher test_arduino.urdf

```

#### Terminal 2: Run Controller Manager

Launches the control loop and opens the serial port connection to the Arduino:

```bash
ros2 run controller_manager ros2_control_node --ros-args --params-file test_controllers.yaml

```

*(Look for confirmation logs like `[INFO] [ArduinoHardwareInterface]: Opening serial connection on /dev/ttyACM0...`)*

#### Terminal 3: Load and Activate Broadcasters

Activate all sensors and state broadcasters:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active imu_sensor_broadcaster
ros2 control load_controller --set-state active ir_sensor_1_broadcaster
ros2 control load_controller --set-state active ir_sensor_2_broadcaster

```

---

### Step 4: Verify Data Streams

Echo any of the active sensor topics to view live data streaming from your hardware:

* **Joint States (Wheels & Servo):**
```bash
ros2 topic echo /joint_states

```


* **IMU Data:**
```bash
ros2 topic echo /imu_sensor_broadcaster/imu

```


* **IR Sensor 1 Range:**
```bash
ros2 topic echo /ir_sensor_1_broadcaster/range

```


* **IR Sensor 2 Range:**
```bash
ros2 topic echo /ir_sensor_2_broadcaster/range

```



---

## Troubleshooting

* **Permission Denied on Serial Port (`/dev/ttyACM0`):**
Grant user access to the dialout group:
```bash
sudo usermod -aG dialout $USER

```


*(Log out and log back in for changes to take effect).*
* **Overrun Warnings in Controller Manager:**
Ensure blocking operations like `serial_.DrainWriteBuffer()` are removed from `arduino_node.cpp` to prevent USB latency from delaying the control loop.
"""