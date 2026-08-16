# Wheel position test

This utility drives forward until one wheel's `/joint_states` position changes by a target angle.

Build and start the hardware:

```bash
cd /root/ros2_ws
colcon build --packages-select my_robot_bringup my_robot_description
source install/setup.bash
ros2 launch my_robot_description hardware_bringup.launch.py
```

In another sourced terminal, rotate the front-left wheel by π radians (half a revolution):

```bash
ros2 run my_robot_description rotate_wheel_to_position
```

Override the wheel, angle, speed, or timeout:

```bash
ros2 run my_robot_description rotate_wheel_to_position --ros-args \
  -p joint_name:=rear_left_wheel_joint \
  -p target_angle:=6.2831853 \
  -p linear_speed:=0.02 \
  -p timeout:=20.0
```

Test with the wheels lifted first. Press `Ctrl+C` to stop. Positive `linear_speed` drives forward; use a negative value to reverse.

## 10 cm odometry test

Drive forward until odometry X changes by 10 cm:

```bash
ros2 run my_robot_description move_to_odom_distance.py
```

Custom distance, speed, and timeout:

```bash
ros2 run my_robot_description move_to_odom_distance.py --ros-args \
  -p target_distance:=0.10 -p linear_speed:=0.03 -p timeout:=15.0
```

Measure the real floor distance from a marked starting point. Press `Ctrl+C` to stop early.

## One-revolution odometry test

Rotate counterclockwise in place until accumulated odometry yaw reaches one revolution:

```bash
ros2 run my_robot_description rotate_to_odom_angle.py
```

Custom angle, speed, and timeout:

```bash
ros2 run my_robot_description rotate_to_odom_angle.py --ros-args \
  -p target_angle:=6.2831853 -p angular_speed:=0.20 -p timeout:=45.0
```

Use a negative `angular_speed` to rotate clockwise. Mark the robot's initial heading and measure its real final heading.

## Single-wheel velocity test

Lift the robot, stop the mecanum controller, and start the test controller:

```bash
ros2 control switch_controllers --deactivate mecanum_drive_controller
ros2 run controller_manager spawner front_left_wheel_test_controller
```

Command the front-left wheel at `1 rad/s` and inspect its measured velocity:

```bash
ros2 topic pub -r 20 /front_left_wheel_test_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic echo /joint_states
```

Press `Ctrl+C`, send zero, unload the test controller, and restore mecanum control:

```bash
ros2 topic pub --once /front_left_wheel_test_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
ros2 control unload_controller front_left_wheel_test_controller
ros2 control switch_controllers --activate mecanum_drive_controller
```
