# my_robot_vision

Camera drivers and vision pipelines for the mecanum robot.

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
ros2 launch my_robot_vision pi_camera.launch.py width:=640 height:=480 fps:=30.0

```

## 2. Docker / Remote Camera (RTP Stream)

Picamera2 requires host-level access to the Pi's libcamera stack and cannot run inside standard Docker containers. Instead, run the sender on the Pi host and the receiver in Docker (or on a remote PC).

**On Pi Host (Sender):**

```bash
sudo apt install python3-picamera2
# Usage: python3 stream_picamera_rtp.py <IP> <PORT>
python3 scripts/stream_picamera_rtp.py 127.0.0.1 5600

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
* **TF2 Tree**: Dynamically broadcasts individual frames for each visible tag (e.g., `aruco_marker_1`, `aruco_marker_42`) relative to the `camera_optical_frame`.
