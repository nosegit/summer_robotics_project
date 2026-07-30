# my_robot_sim

Quick-start guide to build, launch, and drive the Mecanum robot simulation in Gazebo.

---

## 1. Build & Source

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_sim
source install/setup.bash

```

---

## 2. Launch Simulation (Server)

```bash
ros2 launch my_robot_sim gazebo.launch.py

```

---

## 3. Open Gazebo GUI (`gz sim -g`)

Run this inside your GUI container to connect to the headless server:

```bash
gz sim -g

```

---

## 4. Drive Robot (Teleop Keyboard)

Open a new terminal to move the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

* **Forward / Back:** `i` / `,`
* **Rotate Left / Right:** `j` / `l`
* **Strafe Sideways:** `J` / `L` *(Shift + Key)*
* **Stop:** `k`