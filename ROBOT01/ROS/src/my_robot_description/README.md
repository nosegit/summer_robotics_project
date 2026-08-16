# my_robot_description

ROS 2 URDF/Xacro, `ros2_control` hardware configuration, and simulation plugins for a mecanum-wheel mobile robot. In the multi-container Docker setup, the headless server publishes the robot description and TF tree, while the GUI container runs RViz 2.

## Package structure

```text
my_robot_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── robot.launch.py     # Headless description and joint-state publisher
│   └── rviz.launch.py      # RViz 2 with the saved configuration
├── urdf/
│   └── mecanum_robot.urdf.xacro
└── rviz/
    └── rviz_view.rviz
```

## Build and setup

Build in the workspace, then source it:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

Source it in every terminal, or add this command to `~/.bashrc`:

```bash
source ~/ros2_ws/install/setup.bash
```

## Operating modes

Run the appropriate command inside the server container.

| Mode | `use_sim` | `publish_default_joints` | `/joint_states` source | Clock | Command |
| --- | :---: | :---: | --- | --- | --- |
| Watch URDF only (standalone TF/RViz test) | `false` | `true` | `joint_state_publisher`, with all joints at `0.0` radians | Server system clock | `ros2 launch my_robot_description robot.launch.py use_sim:=false publish_default_joints:=true` |
| Real hardware (Arduino motors) | `false` | `false` | Arduino encoders through the `ros2_control` hardware driver | Server system clock | `ros2 launch my_robot_description robot.launch.py use_sim:=false publish_default_joints:=false` |
| Gazebo simulation | `true` | `false` | Gazebo physics through the `gz-sim` plugin | Gazebo `/clock` | `ros2 launch my_robot_description robot.launch.py use_sim:=true publish_default_joints:=false` |

Launching `robot.launch.py` without arguments defaults to Watch URDF Only mode (`use_sim:=false publish_default_joints:=true`). This mode is useful for checking the TF tree and URDF links/visuals without hardware or simulation. Hardware mode disables dummy joint states so the Arduino interfaces publish wheel encoder positions. Gazebo mode enables the `gz-sim` plugins and sets `use_sim_time:=true`.

## Hardware motion tests

Launch `hardware_bringup.launch.py`. To travel approximately 10 cm, stop, and inspect odometry:

```bash
timeout --signal=INT 2s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic echo /odom --once
```

To rotate approximately 90 degrees counterclockwise in place and stop:

```bash
timeout --signal=INT 6.28s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.25}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## RViz 2

In the GUI container, use the launch file or run RViz 2 directly:

```bash
ros2 launch my_robot_description rviz.launch.py
rviz2 -d /root/ros2_ws/src/my_robot_description/rviz/rviz_view.rviz
```

## Troubleshooting

### `No transform from [front_left_wheel] to [base_link]`

RViz is not receiving `/joint_states` for continuous joints. During standalone testing, do not set `publish_default_joints:=false` or `use_sim:=true`. Verify the topic with:

```bash
ros2 topic echo /joint_states --once
```

### Transforms freeze with `use_sim:=true` and no Gazebo

Simulation time makes ROS 2 nodes wait for Gazebo's `/clock`. Without an active Gazebo instance, time remains at `0.0` and `/tf` broadcasts stop. Use `use_sim:=true` only while Gazebo is publishing `/clock`.

### Docker containers cannot communicate

Ensure the server and GUI containers use the host network (`--net=host` or `network_mode: "host"` in Docker Compose) and the same `ROS_DOMAIN_ID`.
