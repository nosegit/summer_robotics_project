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

This launch also starts parcel vision and acquisition by default. Nav2 writes
to `/cmd_vel_navigation`, parcel acquisition writes to
`/cmd_vel_acquisition`, and the priority arbiter is the only publisher to
`/cmd_vel`. A fresh acquisition command overrides navigation; stale inputs are
stopped automatically. Cancel or pause an active Nav2 goal before starting an
acquisition action so that navigation does not resume when acquisition ends.

Disable the parcel nodes, or enable vision debug images, with:

```bash
ros2 launch my_robot_bringup simulation.launch.py parcel_acquisition:=false
ros2 launch my_robot_bringup simulation.launch.py debug_image:=true
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

The hardware launch includes `my_robot_description/launch/hardware_bringup.launch.py`, which owns the Arduino hardware interface and uses `my_robot_description/config/mecanum_controllers.yaml`. It also starts `my_robot_gripper`, making the idle, pickup, and open-gripper sequence services available to missions. Before hardware use, confirm the Arduino ports embedded in the robot Xacro, RPLIDAR device, and that `laser_frame`, `odom`, and `base_link` transforms are correct.

## CLI - note
### SIMULATION
ISA map simulation launch script
``` bash
ros2 launch my_robot_bringup simulation.launch.py mode:=localization map:=/root/ros2_ws/src/my_robot_navigation/maps/isa_map.yaml
```
Rviz gui
``` bash
rviz2 -d /root/ros2_ws/src/my_robot_description/rviz/rviz_view_nav.rviz --ros-args -p use_sim_time:=true
```
### REAL HARDWARE
ISA map realhardware launch script
``` bash
ros2 launch my_robot_bringup hardware.launch.py mode:=localization lidar_port:=/dev/ttyUSB0 map:=/root/ros2_ws/src/my_robot_navigation/maps/isa_map.yaml
ros2 launch my_robot_bringup hardware.launch.py mode:=localization lidar_port:=/dev/ttyUSB0 map:=/root/ros2_ws/src/my_map2.yaml
```
Rviz gui
``` bash
rviz2 -d /root/ros2_ws/src/my_robot_description/rviz/rviz_view_nav.rviz
```
