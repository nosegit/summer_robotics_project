# Parcel acquisition

This package searches for an ArUco-labelled parcel, aligns a mecanum robot with
the parcel using visual servoing, and performs a timed blind rotation. Only one
action can own velocity control at a time, and every exit path publishes zero
velocity.

## Build and standalone simulation

```bash
cd /root/ros2_ws
colcon build --packages-up-to my_robot_parcel_aquisition
source install/setup.bash
ros2 launch my_robot_parcel_aquisition acquisition_sim.launch.py
```

The simulation intentionally does not start Nav2. It places the robot at
`(-5.55, -3.25)` facing global +Y, parallel to the parcel row. Call each action
manually (ID 0 is the first parcel encountered):

The launch uses `DICT_4X4_250` and a 0.09 m marker size to match the existing
Gazebo parcel models. Hardware keeps the vision package's configurable defaults;
set `marker_size` to the measured outer black-square dimension.

```bash
ros2 action send_goal /search_parcel my_robot_parcel_aquisition/action/SearchParcel "{marker_id: 0}" --feedback
ros2 action send_goal /servo_parcel my_robot_parcel_aquisition/action/ServoParcel "{marker_id: 0}" --feedback
ros2 action send_goal /rotate_blind my_robot_parcel_aquisition/action/RotateBlind "{angle: -1.5708}" --feedback
```

`SearchParcel` first drives forward until the requested parcel becomes visible,
then moves forward or backward until the parcel is near the horizontal centre
of the camera image. It succeeds after three consecutive centred observations.
The centering behavior is configured with `search_center_kp`,
`search_center_tolerance` (radians), and `search_center_settle_cycles`.
If the marker briefly disappears while visibility changes between cube faces,
the robot stops and waits up to `search_reacquire_timeout` seconds for the same
cube ID before aborting. The default is 2 seconds.

Or run all three stages:

```bash
ros2 run my_robot_parcel_aquisition acquisition_sequencer --marker-id 2 --angle -1.5708
```

Add `--feedback` to print live feedback from every stage:

```bash
ros2 run my_robot_parcel_aquisition acquisition_sequencer --marker-id 2 --angle 1.5708 --feedback
```

## Nav2 integration

Pause or cancel navigation before acquisition. Remap `cmd_vel_topic` to the
acquisition input of the robot's velocity arbiter; `/cmd_vel_acquisition` is the
recommended integrated name. The standalone default is `/cmd_vel`. Never allow
Nav2 and acquisition to publish to the same unarbitrated input simultaneously.

Rotation goals use radians and are tracked from the configured odometry topic.
Positive angles rotate counter-clockwise and negative angles rotate clockwise.
After rotating, the robot drives forward by `rotate_forward_distance` metres,
also measured from odometry. Set this parameter to `0.0` to disable that step.
The configured initial value is 0.235 m, based on the current 0.5 m parcel
offset and the centre of the 0.18 m gripper footprint; tune it on the robot.
The simulation default is `/odometry/wheel`; set `odometry_topic` to the
hardware or filtered odometry topic when running on the real robot.

## Change manager parameters

Set parameters before sending the next action goal:

```bash
ros2 param set /parcel_acquisition_manager target_x 0.0
ros2 param set /parcel_acquisition_manager target_y 0.35
ros2 param set /parcel_acquisition_manager target_yaw -1.5708
ros2 param set /parcel_acquisition_manager kp_x 0.8
ros2 param set /parcel_acquisition_manager kp_y 1.0
ros2 param set /parcel_acquisition_manager kp_yaw 1.0
ros2 param set /parcel_acquisition_manager max_linear_velocity 0.15
ros2 param set /parcel_acquisition_manager max_angular_velocity 0.5
ros2 param set /parcel_acquisition_manager rotate_forward_distance 0.235
```

Inspect one value or dump all manager parameters:

```bash
ros2 param get /parcel_acquisition_manager target_y
ros2 param dump /parcel_acquisition_manager
```

Changes made with `ros2 param set` are temporary. Update
`config/acquisition.yaml` to keep tuned values after restarting the node.
