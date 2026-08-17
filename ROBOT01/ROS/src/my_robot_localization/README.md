# my_robot_localization

Fuses simulated mecanum odometry and two 6-axis Gazebo IMUs with the
`robot_localization` EKF. The EKF publishes `/odom` and owns the
`odom -> base_link` transform.

```bash
cd /root/ros2_ws
colcon build --packages-select my_robot_localization my_robot_description my_robot_sim my_robot_bringup
source install/setup.bash
ros2 launch my_robot_bringup simulation.launch.py mode:=localization
```

Inspect the inputs and output:

```bash
ros2 topic echo /imu_sensor_L_broadcaster/imu --once
ros2 topic echo /imu_sensor_R_broadcaster/imu --once
ros2 topic echo /odometry/wheel --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /diagnostics
```
