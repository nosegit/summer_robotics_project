### 🐳 Launch Instructions (Docker Compose Setup)

This package is designed to run across two separate Docker containers managed by Docker Compose:

1. **`jazzy_server_dev`**: Runs the headless Gazebo physics engine, robot state publisher, SLAM toolbox, and the entire Nav2 stack.
2. **`jazzy_gui_dev`**: Handles the graphical rendering and user inputs via RViz.

#### Step 1: Spin up the Containers

From your host machine in the directory containing your `docker-compose.yml`, start both the server and GUI containers simultaneously:

```bash
docker compose --profile dev up

```

#### Step 2: Start the Server Stack

Open a new terminal and attach to the running **Server** container:

```bash
docker exec -it jazzy_server_dev bash

```

Once inside, source your workspace and launch the bringup file:

```bash
source install/setup.bash
ros2 launch my_robot_bringup bringup.launch.py

```

> **⏳ Wait for Boot:** Do not proceed to the next step until the Nav2 stack finishes loading. Watch the terminal output until you see this exact line: `[lifecycle_manager_navigation]: Managed nodes are active`

#### Step 3: Start the GUI (RViz)

Open a separate terminal and attach to the running **GUI** container:

```bash
docker exec -it jazzy_gui_dev bash

```

Once inside, source your workspace and launch RViz using your pre-configured layout and simulation time enabled:

```bash
source install/setup.bash
rviz2 -d src/nav2_view.rviz --ros-args -p use_sim_time:=true

```

#### Step 4: Send Omnidirectional Goals

1. In your **GUI** container (RViz2), click the **"2D Goal Pose"** button in the top toolbar.
2. Click and drag a destination on the map.
3. The GUI will send the action request across the Docker network to the Server container. The MPPI controller on the server will calculate the holonomic trajectories, and you will see the robot smoothly glide to its destination in RViz!