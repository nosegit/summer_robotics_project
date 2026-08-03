# my_robot_bringup

Top-level launches for a navigation-ready mecanum robot. Build from the workspace root:

```bash
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

Start simulation with the saved map and Nav2:

```bash
ros2 launch my_robot_bringup simulation.launch.py mode:=localization
```

To create a map, start SLAM and save it after driving the robot:

```bash
ros2 launch my_robot_bringup simulation.launch.py mode:=mapping
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Use the saved map with `map:=/absolute/path/to/my_map.yaml`. The hardware entry point is a template:

```bash
ros2 launch my_robot_bringup hardware.launch.py mode:=localization lidar_port:=/dev/ttyUSB0
```

The hardware launch uses `config/mecanum_controllers.yaml`, copied from the tested configuration in `arduino_hardware_interface/test_file/`. Before hardware use, confirm the Arduino ports embedded in the robot Xacro, RPLIDAR device, and that `laser_frame`, `odom`, and `base_link` transforms are correct.
