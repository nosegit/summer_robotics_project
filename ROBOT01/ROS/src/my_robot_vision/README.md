# my_robot_vision

Camera drivers and vision pipelines for the mecanum robot.

The camera topics use `mono8` grayscale images end-to-end. ArUco processing
does not require colour, and avoiding RGB/BGR conversion reduces CPU use and
ROS image bandwidth.

**Core Topics:**
- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)

---

## 1. Local Camera (ROS 2 on Pi Host)
Use this if running ROS 2 directly on the Raspberry Pi hardware.

```bash
sudo apt update && sudo apt install python3-picamera2
colcon build --packages-select my_robot_vision
source install/setup.bash

# Launch with optional parameter overrides
ros2 launch my_robot_vision pi_camera.launch.py width:=640 height:=480 fps:=10.0

```

## 2. Docker / Remote Camera (RTP Stream)

Picamera2 requires host-level access to the Pi's libcamera stack and cannot run inside standard Docker containers. Instead, run the sender on the Pi host and the receiver in Docker (or on a remote PC).

**On Pi Host (Sender):**

```bash
sudo apt install python3-picamera2
# Usage: python3 stream_picamera_rtp.py <IP> <PORT>
python3 scripts/stream_picamera_rtp.py 127.0.0.1 5600 --fps 10

```

**In Docker / Remote PC (Receiver):**

```bash
colcon build --packages-select my_robot_vision
source install/setup.bash
ros2 launch my_robot_vision rtp_camera.launch.py port:=5600

```

## 3. Camera Calibration

Metric 3D pose estimation requires physical camera calibration. Convert your OpenCV `.npz` calibration to standard ROS YAML:

```bash
ros2 run my_robot_vision convert_camera_calibration \
  /home/robot/camera_calibration.npz \
  /home/robot/camera_calibration.yaml \
  --width 640 --height 480

```

Pass the YAML file when launching the camera node:

```bash
ros2 launch my_robot_vision pi_camera.launch.py camera_info_url:=file:///home/robot/camera_calibration.yaml

```

*(Without calibration, the node publishes approximate intrinsics which are not metrically accurate).*

## 4. ArUco Pose Estimation

Detects multiple markers simultaneously, estimating their 3D poses and broadcasting transforms.
*Note: Ensure `vision_msgs` is installed (`rosdep install` or `apt install ros-<distro>-vision-msgs`).*

**Launch:**

```bash
ros2 launch my_robot_vision aruco_pose_estimator.launch.py marker_size:=0.05 dictionary_name:=DICT_5X5_100

```

**Outputs:**

* **Topic (`/aruco/detections`)**: A `vision_msgs/msg/Detection3DArray`. The specific tag ID is stored in the `results[0].hypothesis.class_id` field.
* **Topic (`/aruco/cubes`)**: A `my_robot_vision/msg/CubeDetectionArray` containing one fused cube-centre pose, horizontal face normal, ID, and visible-face count per parcel.
* **TF2 Tree**: Broadcasts unique raw face frames such as `aruco_marker_1_face_0` and a fused `cube_1` frame relative to the camera optical frame.

Debug images are disabled by default. With `debug_image:=false`, the node does
not create or publish either image topic. Enable them with:

```bash
ros2 launch my_robot_vision aruco_pose_estimator.launch.py debug_image:=true marker_size:=0.09 dictionary_name:=DICT_4X4_250
```

* `/aruco/image_detected` shows marker outlines and face axes.
* `/aruco/image_cube` shows fused cube centres, IDs, and centre-frame axes.

Cube-centre positions use outlier rejection followed by EMA smoothing by
default. The filtered position is shared by `/aruco/cubes`, `cube_<id>`, the
cube debug image, and parcel servoing. Example tuning:

```bash
ros2 launch my_robot_vision aruco_pose_estimator.launch.py \
  position_filter_enabled:=true \
  position_filter_alpha:=0.25 \
  position_outlier_threshold:=0.04 \
  position_filter_reset_timeout:=0.5
```

Lower `position_filter_alpha` for more smoothing or raise it for less control
lag. Set `position_outlier_threshold:=0.0` to disable only jump rejection.

### ArUco and cube-estimation parameters

The following table lists every parameter declared by `aruco_pose_estimator`.
Defaults shown are those used by `aruco_pose_estimator.launch.py`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `marker_size` | `0.05` | Outer black-square side length of one ArUco marker, in metres. Pose scale is directly proportional to this value, so measure the printed marker rather than relying on the design size. The existing Gazebo cubes use `0.09`. |
| `dictionary_name` | `DICT_5X5_100` | OpenCV ArUco dictionary used to decode marker IDs. It must match the generated or printed markers. The existing Gazebo cubes use `DICT_4X4_250`. |
| `image_topic` | `camera/image_raw` | Input `sensor_msgs/msg/Image` topic. |
| `camera_info_topic` | `camera/camera_info` | Input `sensor_msgs/msg/CameraInfo` topic containing the camera matrix and distortion coefficients. No pose is processed until this message is received. |
| `cube_size` | `0.10` | Parcel side length in metres. Half this value is applied along each outward face normal to project a marker pose to the cube centre. |
| `cube_detections_topic` | `aruco/cubes` | Output topic for `my_robot_vision/msg/CubeDetectionArray`. This is currently a node parameter rather than a launch-file argument. |
| `debug_image` | `false` | When `true`, render and publish `/aruco/image_detected` and `/aruco/image_cube`. When `false`, those publishers and drawing work are disabled. |
| `face_vertical_threshold` | `0.65` | Maximum absolute optical-Y component allowed when classifying a face normal as horizontal. Optical Y points downward. Lower values reject tilted/top faces more strictly; higher values accept more faces. Valid practical range is `0.0` to `1.0`. |
| `face_switch_hysteresis` | `0.15` | Camera-facing score margin that lets the estimator retain the previously selected face rather than switch to an adjacent face. Increase it if cube axes switch by approximately 90 degrees; reduce it if tracking holds a badly angled face too long. |
| `position_filter_enabled` | `true` | Enables per-ID temporal cube-centre filtering after the visible face centres have been fused with a median. It affects `/aruco/cubes`, `cube_<id>` TF, the cube debug image, and servo input. |
| `position_filter_alpha` | `0.25` | EMA weight assigned to the newest centre measurement. `1.0` gives no EMA lag; smaller values smooth more but respond more slowly. It must be greater than `0.0` and at most `1.0`. |
| `position_outlier_threshold` | `0.04` | Reject a consecutive centre measurement that jumps farther than this distance, in metres. Set it to `0.0` to disable jump rejection. A threshold that is too small can temporarily freeze a legitimately moving estimate. |
| `position_filter_reset_timeout` | `0.5` | Time without an accepted observation, in seconds, after which the stored position is discarded and the next measurement starts a fresh filter. |

All cube-estimator configuration is captured when the node starts. Relaunch the
node after changing these values; setting them dynamically does not rebuild the
existing detector or `CubeEstimator` object.

Gazebo configuration matching the repository's parcel models:

```bash
ros2 launch my_robot_vision aruco_pose_estimator.launch.py \
  marker_size:=0.09 \
  dictionary_name:=DICT_4X4_250 \
  cube_size:=0.10 \
  debug_image:=true
```

Low-lag cube position with EMA and jump rejection disabled:

```bash
ros2 launch my_robot_vision aruco_pose_estimator.launch.py \
  position_filter_enabled:=true \
  position_filter_alpha:=1.0 \
  position_outlier_threshold:=0.0
```

Disable all temporal position filtering while retaining same-frame median face
fusion:

```bash
ros2 launch my_robot_vision aruco_pose_estimator.launch.py \
  position_filter_enabled:=false
```
