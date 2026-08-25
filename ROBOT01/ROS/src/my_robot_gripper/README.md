# my_robot_gripper

Smooth service-based control for the two gripper servos. The node publishes
degree-valued `std_msgs/msg/Float64MultiArray` commands expected by this
repository's Arduino hardware interface.

```bash
ros2 launch my_robot_gripper gripper.launch.py

ros2 service call /gripper_controller/open std_srvs/srv/Trigger '{}'
ros2 service call /gripper_controller/close std_srvs/srv/Trigger '{}'
ros2 service call /gripper_controller/lift_down std_srvs/srv/Trigger '{}'
ros2 service call /gripper_controller/lift_down_fit std_srvs/srv/Trigger '{}'
ros2 service call /gripper_controller/lift_up std_srvs/srv/Trigger '{}'
```

Complete pick-and-drop operations are also available as single service calls:

```bash
ros2 service call /gripper_controller/idle std_srvs/srv/Trigger '{}'
ros2 service call /gripper_controller/open_gripper std_srvs/srv/Trigger '{}'
ros2 service call /gripper_controller/pickup std_srvs/srv/Trigger '{}'
```

The sequences are:

- `idle`: lower and close together
- `open_gripper`: lower to fit, then open
- `pickup`: lower to fit, close, then lift up

Each sequential stage waits for the preceding smooth servo motion to finish.
New commands are rejected while a motion or sequence is active.

`lift_down` moves to 80 degrees and `lift_down_fit` moves farther to 70
degrees. `lift_up` moves smoothly to 180 degrees and then smoothly settles at
140 degrees. Speeds and target angles are configurable in `config/gripper.yaml`.
