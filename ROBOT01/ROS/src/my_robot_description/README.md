# my_robot_description

A ROS 2 package containing the URDF/Xacro description, hardware configuration (`ros2_control`), and simulation plugins for a **Mecanum Wheel Mobile Robot**. This package is structured for a multi-container Docker deployment where a headless server container publishes the robot description and TF tree, while a separate GUI container visualizes the robot using RViz 2.

---

## 📁 Package Structure

```text
my_robot_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── robot.launch.py     # Headless robot description & joint state publisher
│   └── rviz.launch.py      # Launches RViz2 with the saved configuration
├── urdf/
│   └── mecanum_robot.urdf.xacro # Main mecanum robot Xacro model
└── rviz/
    └── rviz_view.rviz      # Saved RViz2 display configuration
```

---

## 🛠️ Build & Workspace Setup

1. **Navigate to your workspace and build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_description
   source install/setup.bash
   ```

2. **Source the workspace in all terminal sessions (or add to `~/.bashrc`):**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## 🚀 Operational Modes & Launch Reference

The robot can be launched in **three distinct operational modes** depending on whether you are inspecting the model standalone, running physical Arduino motors, or running a full Gazebo simulation.

### Operational Mode Configuration & Command Matrix

| Operational Mode | `use_sim` | `publish_default_joints` | What Publishes `/joint_states`? | What Provides Clock Time? | Exact Command to Run |
| :--- | :---: | :---: | :--- | :--- | :--- |
| **1. Watch URDF Only**<br>*(Standalone TF / RViz Test)* | `false` | **`true`** | Dummy `joint_state_publisher`<br>*(Fakes **0.0** radians for all joints)* | Your Server's **System Clock** | `ros2 launch my_robot_description robot.launch.py use_sim:=false publish_default_joints:=true` |
| **2. Real Hardware**<br>*(Arduino Motors Connected)* | `false` | **`false`** | Your **Arduino Encoders** via<br>`ros2_control` hardware driver | Your Server's **System Clock** | `ros2 launch my_robot_description robot.launch.py use_sim:=false publish_default_joints:=false` |
| **3. Gazebo Simulation**<br>*(Full Simulation Running)* | **`true`** | **`false`** | **Gazebo Physics Engine**<br>*(via `gz-sim` plugin)* | **Gazebo Sim Clock**<br>*(via the `/clock` topic)* | `ros2 launch my_robot_description robot.launch.py use_sim:=true publish_default_joints:=false` |

> [!NOTE]
> Running `ros2 launch my_robot_description robot.launch.py` without arguments defaults to **Watch URDF Only** mode (`use_sim:=false publish_default_joints:=true`).

---

## 🖥️ Launching on the Server Docker (Headless)

Run the appropriate launch command inside your **Server Docker container** based on your target mode:

### 1. Watch URDF Only (No Hardware or Sim)
Useful for verifying the TF tree and inspecting URDF links/visuals in RViz without physical hardware:
```bash
ros2 launch my_robot_description robot.launch.py use_sim:=false publish_default_joints:=true
```

### 2. Real Hardware Mode (Arduino `ros2_control`)
Disables dummy joint states so that your real Arduino hardware interfaces publish encoder wheel positions:
```bash
ros2 launch my_robot_description robot.launch.py use_sim:=false publish_default_joints:=false
```

### 3. Gazebo Simulation Mode
Enables Gazebo `gz-sim` plugins and configures nodes to sync with simulation time (`use_sim_time:=true`):
```bash
ros2 launch my_robot_description robot.launch.py use_sim:=true publish_default_joints:=false
```

---

## 📊 Launching RViz 2 (GUI Docker Container)

From your **GUI Docker container**, use either of the following methods to launch RViz2 with your saved display configuration:

### Option A: Using the launch file (Recommended)
```bash
ros2 launch my_robot_description rviz.launch.py
```

### Option B: Using the direct one-line command
```bash
rviz2 -d /root/ros2_ws/src/my_robot_description/rviz/rviz_view.rviz
```

---

## ⚠️ Troubleshooting & FAQ

### 1. `No transform from [front_left_wheel] to [base_link]` in RViz
* **Cause:** RViz is not receiving `/joint_states` data for continuous joints.
* **Fix:** 
  * If testing standalone (without Arduino or Gazebo), ensure you did **not** set `publish_default_joints:=false` or `use_sim:=true`.
  * Verify `/joint_states` is publishing:
    ```bash
    ros2 topic echo /joint_states --once
    ```

### 2. Why does `use_sim:=true` freeze transforms when Gazebo isn't running?
* **Cause:** `use_sim:=true` tells all ROS 2 nodes to wait for the `/clock` topic published by Gazebo. If Gazebo is not actively running, node clocks freeze at `0.0`, stopping `/tf` broadcasts.
* **Fix:** Only pass `use_sim:=true` when an active Gazebo simulation instance is publishing `/clock`.

### 3. Docker Container Communication Issues
* Ensure both your **Server** and **GUI** containers are running on the host network (`--net=host` or `network_mode: "host"` in Docker Compose) and share the same `ROS_DOMAIN_ID`.