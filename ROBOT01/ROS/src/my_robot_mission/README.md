# Robot parcel mission

This package runs a YAML-defined command sequence, for example:

1. Publish the fixed robot start pose to AMCL.
2. Navigate to a safe parcel approach pose (normally about 1 m away).
3. Search for, dock with, and pick up the configured ArUco parcel.
4. Navigate to the destination pose and open the gripper.

Before using hardware, edit `config/mission.yaml` and change
`mission_configured` to `true`. Every entry in `poses` is `[x, y, yaw]` in the
map frame, in metres and radians. A parcel approach pose should normally be a
collision-free point about 1 m away with the camera facing the search direction.

Commands are executed from top to bottom:

- `set_initial_pose`
- `navigate_to_pose(N)` navigates to pose ID `N` from `poses`.
- `search_parcel(N)` searches for marker ID `N`.
- `servo_parcel(N)` docks with marker ID `N`.
- `rotate_blind` performs the configured pickup rotation.
- `idle`, `open_gripper`, and `pickup` run the matching gripper sequence.
- `acquire_parcel(N)` is a shortcut for search, docking, and blind rotation;
  it does not move the gripper.

Gripper movement only occurs when its command is explicitly present in the
mission YAML. Gripper operations wait for the smooth motion to finish before
the mission continues.

For example:

```yaml
poses:
  0: [1.0, 0.0, 0.0]
  10: [0.0, 1.0, 1.5708]
commands:
  - idle
  - set_initial_pose
  - navigate_to_pose(0)
  - search_parcel(2)
  - servo_parcel(2)
  - open_gripper
  - rotate_blind
  - pickup
  - navigate_to_pose(10)
  - open_gripper
```

Commands can be reordered, omitted, or repeated. The complete command list and
all referenced poses are validated before robot motion starts.

Build the mission package from the ROS workspace/container:

```bash
cd /root/ros2_ws
colcon build --packages-up-to my_robot_mission
source install/setup.bash
```

Start the robot separately and leave it running:

```bash
ros2 launch my_robot_bringup hardware.launch.py
```

In another sourced terminal, start the mission when the robot is physically at
its configured start pose:

```bash
ros2 run my_robot_mission parcel_mission
```

The executable automatically loads the installed `config/mission.yaml`. To use
a different configuration for one run:

```bash
ros2 run my_robot_mission parcel_mission --mission-file /path/to/mission.yaml
```

The default configuration is disabled so its example coordinates cannot move a
real robot accidentally. Once enabled, the initial pose is published
automatically on every mission start. Only use this on a robot that is
physically placed at the configured start pose. An incorrect fixed pose can
make Nav2 drive unsafely.
