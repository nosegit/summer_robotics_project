# my_robot_navigation

Quick-start guide to mapping (SLAM) and autonomous navigation (Nav2) for the Mecanum robot in Gazebo Sim.

---

## 🛠️ Build & Setup

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_sim my_robot_description my_robot_navigation
source install/setup.bash

```

---

## 🗺️ Phase 1: SLAM (Mapping)

### 1. Launch Simulation (Server)

```bash
ros2 launch my_robot_sim gazebo.launch.py headless:=true

```

### 2. Launch SLAM Node

```bash
ros2 launch my_robot_navigation slam.launch.py

```

### 3. Launch RViz (GUI)

```bash
rviz2 -d /root/ros2_ws/src/my_robot_description/rviz/rviz_view_nav.rviz --ros-args -p use_sim_time:=true

```

> **Note:** Ensure **Global Options $\rightarrow$ Fixed Frame** in RViz is set to `map`.

### 4. Teleop (Drive to build map)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

### 5. Save Map & Rebuild

Once the map is complete, run:

```bash
cd ~/ros2_ws/src/my_robot_navigation/maps
ros2 run nav2_map_server map_saver_cli -f simple_room

# Rebuild so package installs the map files
cd ~/ros2_ws
colcon build --packages-select my_robot_navigation
source install/setup.bash

```

---

## 🚀 Phase 2: Autonomous Navigation (Nav2)

### 1. Launch Simulation (Server)

```bash
ros2 launch my_robot_sim gazebo.launch.py headless:=true

```

### 2. Launch Nav2 Stack

```bash
ros2 launch my_robot_navigation navigation.launch.py map:=$(ros2 pkg prefix my_robot_navigation)/share/my_robot_navigation/maps/simple_room.yaml

```

### 3. Launch RViz (GUI)

```bash
rviz2 -d /root/ros2_ws/src/my_robot_description/rviz/rviz_view_nav.rviz --ros-args -p use_sim_time:=true

```

### 4. Drive Autonomously

1. Verify **Fixed Frame** = `map`.
2. Click **2D Pose Estimate** in top toolbar $\rightarrow$ click & drag on robot's position in RViz.
3. Click **2D Goal Pose** / **Nav2 Goal** $\rightarrow$ click & drag anywhere open on the map to navigate.