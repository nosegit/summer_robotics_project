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

Or run all three stages:

```bash
ros2 run my_robot_parcel_aquisition acquisition_sequencer --marker-id 2 --angle -1.5708
```

## Nav2 integration

Pause or cancel navigation before acquisition. Remap `cmd_vel_topic` to the
acquisition input of the robot's velocity arbiter; `/cmd_vel_acquisition` is the
recommended integrated name. The standalone default is `/cmd_vel`. Never allow
Nav2 and acquisition to publish to the same unarbitrated input simultaneously.

Blind rotation is approximate and requires calibration on the real floor.
