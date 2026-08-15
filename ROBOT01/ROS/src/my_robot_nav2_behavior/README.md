# Mecanum escape behavior

`my_robot_nav2_behavior::MecanumEscape` is a drop-in replacement for Nav2's
`nav2_behaviors::BackUp`. It keeps the standard `nav2_msgs/action/BackUp` action
server, so Nav2's default behavior trees do not need a custom BT node.

When recovery starts, the plugin:

1. Remembers the latest non-zero robot-frame `vx`/`vy` from odometry. It uses the
   opposite vector as the preferred escape direction; if no recent velocity is
   available, it prefers robot-rearward (`-X`).
2. Samples eight robot-frame directions at 45-degree intervals. In every direction,
   it projects the complete robot footprint through the local costmap in
   `projection_step` increments and measures the collision-free distance.
3. Rejects directions that cannot fit the requested travel distance plus the
   clearance margin. It scores the remaining directions using alignment with the
   preferred direction and normalized free distance, then selects the highest score.
4. Publishes a holonomic command using both `linear.x` and `linear.y`, without
   rotating the robot. During motion it checks the selected trajectory every cycle
   and stops on success, timeout, TF failure, or a newly detected collision.

The requested travel distance, speed, and timeout are supplied by the standard
Nav2 `BackUp` action, normally from a Behavior Tree node such as:

```xml
<BackUp backup_dist="0.30" backup_speed="0.15" time_allowance="10.0"/>
```

`backup_dist` is a scalar distance: the plugin chooses the actual X/Y direction.

## Configuration

The behavior is configured under its behavior-server plugin ID, normally
`backup`:

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "wait"]
    backup:
      plugin: "my_robot_nav2_behavior::MecanumEscape"
      odom_topic: /odom
      projection_step: 0.05
      search_distance: 0.60
      clearance_margin: 0.05
      simulate_ahead_time: 2.0
      min_remembered_speed: 0.03
      velocity_memory_duration: 30.0
      direction_preference_weight: 0.35
      clearance_weight: 0.65
```

| Parameter | Default | Meaning and tuning effect |
| --- | ---: | --- |
| `odom_topic` | `odom` | Odometry used to remember the motion that preceded recovery. Its twist should be expressed in the robot base frame. |
| `projection_step` | `0.05 m` | Spacing between footprint collision checks. Smaller is safer and more accurate but costs more CPU; it should normally be no larger than the local-costmap resolution. |
| `search_distance` | `0.60 m` | Maximum distance examined when comparing directions. This is not the commanded travel distance. |
| `clearance_margin` | `0.05 m` | Additional free path required beyond `backup_dist`. Increase it for more room at the end of recovery. |
| `simulate_ahead_time` | `2.0 s` | Runtime collision-check horizon, limited by the remaining travel distance. Increase for earlier reaction at higher speeds. |
| `min_remembered_speed` | `0.03 m/s` | Ignores smaller odometry velocities as noise. Increase if stationary odometry is noisy. |
| `velocity_memory_duration` | `30.0 s` | Maximum age of the remembered motion. After this, rearward becomes the preferred direction. |
| `direction_preference_weight` | `0.35` | Importance of moving opposite the previous motion. Higher values favor that direction. |
| `clearance_weight` | `0.65` | Importance of choosing the direction with more free space. Higher values favor clearance. |

The two weights are relative; keeping their sum near `1.0` makes tuning easier.
The local costmap footprint must describe the real robot, because all candidate and
runtime collision checks use that footprint. See
`my_robot_navigation/config/nav2_params*.yaml` for the active examples.



## Testing

```bash
cd /root/ros2_ws
colcon build --packages-up-to my_robot_nav2_behavior my_robot_navigation
source install/setup.bash

ros2 action send_goal /backup nav2_msgs/action/BackUp \
  "{target: {x: -0.30, y: 0.0, z: 0.0}, speed: -0.15, time_allowance: {sec: 10, nanosec: 0}}" \
  --feedback
```
