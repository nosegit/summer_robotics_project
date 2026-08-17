# Localization tuning guide

Use this procedure to tune the mecanum robot EKF incrementally. Do not enable
every IMU measurement at once; validate each input before adding another.

## EKF configuration vector

Every `*_config` vector contains 15 booleans in this order:

```text
x, y, z,
roll, pitch, yaw,
vx, vy, vz,
vroll, vpitch, vyaw,
ax, ay, az
```

The initial wheel configuration fuses planar forward, lateral, and yaw
velocities:

```yaml
odom0_config: [
  false, false, false,
  false, false, false,
  true,  true,  false,
  false, false, true,
  false, false, false
]
```

Each IMU initially contributes only its Z angular velocity:

```yaml
imu0_config: [
  false, false, false,
  false, false, false,
  false, false, false,
  false, false, true,
  false, false, false
]
```

This is a suitable starting configuration for a planar mecanum robot.

## 1. Inspect message covariances

```bash
ros2 topic echo /imu_sensor_L_broadcaster/imu --once
ros2 topic echo /imu_sensor_R_broadcaster/imu --once
ros2 topic echo /odometry/wheel --once
```

Inspect these fields:

```text
orientation_covariance
angular_velocity_covariance
linear_acceleration_covariance
pose.covariance
twist.covariance
```

Zero matrices do not provide the EKF with meaningful uncertainty information.
Check what `ros_gz_bridge` publishes before adding covariance elsewhere.

The simulated IMUs currently use these Gaussian noise standard deviations:

```text
Gyroscope:     0.002 rad/s
Accelerometer: 0.02 m/s²
```

Covariance is variance (`standard_deviation²`), giving:

```text
Gyroscope:     0.002² = 0.000004
Accelerometer: 0.02²  = 0.0004
```

Corresponding diagonal covariance matrices are:

```yaml
angular_velocity_covariance: [
  0.000004, 0.0,      0.0,
  0.0,      0.000004, 0.0,
  0.0,      0.0,      0.000004
]

linear_acceleration_covariance: [
  0.0004, 0.0,    0.0,
  0.0,    0.0004, 0.0,
  0.0,    0.0,    0.0004
]
```

## 2. Leave orientation disabled initially

A physical 6-axis IMU has no absolute magnetic heading. Yaw derived by
integrating its gyroscope will drift. Initially keep roll, pitch, and yaw
orientation disabled and fuse only `vyaw`.

Keep these settings while only angular velocity is fused:

```yaml
imu0_relative: false
imu1_relative: false
imu0_remove_gravitational_acceleration: false
imu1_remove_gravitational_acceleration: false
```

The relative and gravity-removal parameters have no effect on an input that
only fuses angular velocity.

## 3. Test stationary behaviour

Leave the robot stationary for one or two minutes:

```bash
ros2 topic hz /odom
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /diagnostics
```

Check that:

- `/odom` publishes close to 50 Hz.
- X and Y remain nearly constant.
- Yaw changes only slightly.
- No sensor timeout or transform warnings appear.
- Covariance grows gradually and does not become `NaN` or remain
  unrealistically zero.

Record a bag for repeatable analysis:

```bash
ros2 bag record \
  /odometry/wheel \
  /imu_sensor_L_broadcaster/imu \
  /imu_sensor_R_broadcaster/imu \
  /odom \
  /tf \
  /diagnostics
```

## 4. Test each motion independently

Test these motions separately:

1. Forward and backward.
2. Strafe left and right.
3. Rotate counterclockwise and clockwise.
4. Combine translation and rotation.

Compare raw and filtered odometry:

```bash
ros2 topic echo /odometry/wheel
ros2 topic echo /odom
```

During rotation, verify that wheel odometry and both IMUs report the same yaw
rate sign:

```bash
ros2 topic echo /odometry/wheel --field twist.twist.angular.z
ros2 topic echo /imu_sensor_L_broadcaster/imu --field angular_velocity.z
ros2 topic echo /imu_sensor_R_broadcaster/imu --field angular_velocity.z
```

If an IMU has the opposite sign, fix its frame or mounting transform. Do not
hide a frame error by changing its covariance.

## 5. Add acceleration later

After the angular-velocity filter is stable, test X and Y acceleration on one
IMU first:

```yaml
imu0_config: [
  false, false, false,
  false, false, false,
  false, false, false,
  false, false, true,
  true,  true,  false
]

imu0_remove_gravitational_acceleration: true
```

Gazebo IMUs contain orientation, allowing `robot_localization` to remove
gravity. Gravity removal requires valid orientation in the IMU message even
when orientation is not selected for state fusion.

If the first IMU improves the estimate, apply the same configuration to
`imu1`. Accelerometers can degrade planar odometry when their bias or
covariance is inaccurate.

For real hardware, select gravity removal based on the firmware output:

- Raw acceleration containing gravity: `true`.
- Already gravity-compensated linear acceleration: `false`.

## 6. Tune process and measurement noise

Keep the default `process_noise_covariance` until all inputs, timestamps,
frames, signs, and measurement covariances are correct.

General effects:

- Larger process noise makes the EKF follow new measurements faster.
- Smaller process noise makes the estimate smoother but slower.
- Larger sensor covariance reduces that sensor's influence.
- Smaller sensor covariance increases that sensor's influence.

Tune one state at a time:

- Slow strafing response: slightly increase process noise for `vy`.
- Slow rotational response: slightly increase process noise for `vyaw`.
- Noisy yaw: increase IMU angular-velocity covariance.
- Wheel slip dominates yaw: increase wheel odometry yaw-rate covariance.
- IMU noise dominates yaw: increase IMU yaw-rate covariance.

Do not copy a large covariance matrix from another robot. Sensor noise, wheel
geometry, surface friction, and update rates will differ.

## 7. Understand the simulation limitation

`/odometry/wheel` currently comes from Gazebo's ground-truth odometry
publisher. It is not calculated from noisy wheel encoders, so simulation will
usually look much better than real hardware.

A more realistic future pipeline is:

```text
wheel joint states
    -> mecanum wheel odometry with encoder noise and slip
    -> /odometry/wheel
    -> EKF
```

Until then, use simulation to verify topic connections, frames, TF ownership,
sensor signs, EKF configuration, and Nav2 integration. Use stationary and
motion data from the physical robot later to calculate real IMU and wheel
odometry covariances.

## References

- [robot_localization EKF template](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml)
- [robot_localization state-estimation parameters](https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html)
