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
- `search_fallback(N, POSE)` searches once and, if it fails, navigates back to
  `POSE` before making one final search attempt.
- `search_parcel_two_areas(N)` starts at `parcel_right`, searches until the
  robot reaches map `y = -4`, then navigates to `parcel_left` and searches
  again if the marker was not found.
- `servo_parcel(N)` docks with marker ID `N`.
- `rotate_blind` performs the configured pickup rotation.
- `idle`, `open_gripper`, and `pickup` run the matching gripper sequence.
- `open_door(N)` sends ASCII `open` to door ID `N` over UDP.
- `close_door(N)` sends ASCII `close` to door ID `N` over UDP.
- `wait(N)` waits for `N` seconds while continuing to process ROS callbacks.
- `led_blink` alternates the red and green mission LEDs.
- `led_red` holds the red LED on.
- `led_green` holds the green LED on.
- `led_off` turns both mission LEDs off.
- `acquire_parcel(N)` is a shortcut for search, docking, and blind rotation;
  it does not move the gripper.

Gripper movement only occurs when its command is explicitly present in the
mission YAML. Gripper operations wait for the smooth motion to finish before
the mission continues.

Set `retry_count` in the mission YAML to control additional attempts after the
first (`0` disables retries and `1` gives two total attempts). Search commands
are excluded from automatic in-place retry because repeating a search at the
failed position is unsafe. `search_fallback(N, POSE)` uses the same retry count
and returns to `POSE` before each additional search attempt.

For example:

```yaml
poses:
  0: [1.0, 0.0, 0.0]
  parcel_staging: [0.0, 1.0, 1.5708]
  delivery: [2.0, 1.0, 0.0]
commands:
  - idle
  - set_initial_pose
  - led_blink
  - navigate_to_pose(0)
  - search_parcel(2)
  - servo_parcel(2)
  - open_gripper
  - rotate_blind
  - pickup
  - navigate_to_pose(parcel_staging)
  - navigate_to_pose(delivery)
  - open_door(0)
  - led_green
  - wait(10)
  - close_door(0)
  - open_door(1)
  - wait(10)
  - close_door(1)
  - led_red
  - open_gripper

doors:
  0: {host: 192.168.1.100, port: 50000}
  1: {host: 192.168.1.101, port: 50000}
```

Door commands use UDP. Each entry in `doors` has its own controller IP address;
its `port` defaults to `50000` when omitted. UDP does not acknowledge delivery,
so a successful send only confirms that the datagram was handed to the network
stack.

Commands can be reordered, omitted, or repeated. The complete command list and
all referenced poses are validated before robot motion starts.

## Drone-selected parcel row

The mission can select a fixed staging pose either from a transferred drone JSON
file or from live ROS detections. For the file workflow, configure:

```yaml
parcel_id: 6
poses:
  parcel_left: [-3.0, -3.50, 1.57]
  parcel_right: [-3.0, -6.85, 1.57]
drone_parcel:
  result_file: /tmp/targetLocation.json
  coordinate_scale: 1.0        # converted JSON is in robot-frame metres
  map_origin_x: 0.0
  map_origin_y: 0.0
  zone_boundary_y: -4.0
  boundary_margin: 0.3
  upper_staging_pose: parcel_left
  lower_staging_pose: parcel_right
commands:
  - navigate_to_drone_parcel(parcel_id)
  - search_parcel(parcel_id)
  - servo_parcel(parcel_id)
```

`Drone/convert_to_robot_frame.py` converts the drone coordinates to robot-frame
metres before transfer, so the mission uses a scale of `1.0` and zero map
offsets. It checks the marker ID when the JSON includes one and aborts before
navigation if the file is missing, malformed, for another marker, or inside the
configured boundary margin.

### Transfer after manually changing Wi-Fi

`Drone/drone_tag.py` writes `Drone/targetLocation.json` next to the script. Run
`Drone/convert_to_robot_frame.py` to create `Drone/targetLocation_robot.json`.
After the drone lands, manually change the host computer from the drone Wi-Fi
to the network containing the Raspberry Pi. Then run the cross-platform Python
helper (Windows and Linux both need OpenSSH `scp` and `ssh` on `PATH`):

```bash
python Drone/transfer_and_start_mission.py \
  --robot pi@192.168.1.20 \
  --remote-file /tmp/targetLocation.json
```

This validates the JSON before copying it. Start the already-built mission on
the Pi after the copy, or supply the exact command used by your Pi installation:

```bash
python Drone/transfer_and_start_mission.py \
  --robot pi@192.168.1.20 \
  --start-command "ros2 run my_robot_mission parcel_mission"
```

The SSH session must already source the ROS workspace for that short command to
work. If it does not, make `--start-command` source the workspace first, for
example `source /root/ros2_ws/install/setup.bash && ros2 run ...`. The hardware
bringup/Nav2 process must already be running on the robot.

A literal integer can replace `parcel_id` in any command for a one-off
override.

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
ros2 run my_robot_mission parcel_mission --mission-file /root/ros2_ws/src/my_robot_mission/config/mission.yaml
ros2 run my_robot_mission parcel_mission --mission-file /root/ros2_ws/src/my_robot_mission/config/mission_full_search.yaml


```

Add `--feedback` to print live parcel-search, servoing, and rotation feedback:

```bash
ros2 run my_robot_mission parcel_mission \
  --mission-file /root/ros2_ws/src/my_robot_mission/config/mission.yaml \
  --feedback
```

During servoing this reports `x_error`, `y_error`, and `yaw_error` as the robot
approaches the parcel.

The default configuration is disabled so its example coordinates cannot move a
real robot accidentally. Once enabled, the initial pose is published
automatically on every mission start. Only use this on a robot that is
physically placed at the configured start pose. An incorrect fixed pose can
make Nav2 drive unsafely.

## Raspberry Pi mission LEDs

The persistent LED controller alternates red and green while a mission runs,
then holds solid red when the mission completes. Hardware bringup starts it on
BCM GPIO 17 (red) and BCM GPIO 27 (green) by default. Override the pins with:

```bash
ros2 launch my_robot_bringup hardware.launch.py \
  red_led_gpio:=17 green_led_gpio:=27
```

The states can also be controlled manually:

```bash
ros2 service call /mission_led_controller/start_blinking std_srvs/srv/Trigger '{}'
ros2 service call /mission_led_controller/solid_red std_srvs/srv/Trigger '{}'
ros2 service call /mission_led_controller/solid_green std_srvs/srv/Trigger '{}'
ros2 service call /mission_led_controller/off std_srvs/srv/Trigger '{}'
```

The default alternating period is 0.5 seconds. The server Docker image includes
`gpiozero` with its `lgpio` backend, and Compose passes `/dev/gpiomem` and
`/dev/gpiochip0` into the server container. The GPIO devices exist only on a
Raspberry Pi; remove or override those mappings on other hosts.
