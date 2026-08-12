# my_robot_vision

This package publishes the robot's front camera using standard ROS 2 topics:

- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)

The Gazebo sensor and bridge are started by `my_robot_sim/gazebo.launch.py`.

## Pi CSI camera on the Raspberry Pi host

When ROS runs directly on the Pi, use the normal Picamera2 driver. Install it
from Raspberry Pi OS, build the workspace, and launch the existing camera node:

```bash
sudo apt update
sudo apt install python3-picamera2
cd ~/ros2_ws
colcon build --packages-select my_robot_vision
source install/setup.bash
ros2 launch my_robot_vision pi_camera.launch.py
```

It publishes 640×480 RGB images at 30 fps by default. Change these if needed:

```bash
ros2 launch my_robot_vision pi_camera.launch.py width:=640 height:=480 fps:=30.0
```

## Pi CSI camera while ROS runs in Docker (verified on real hardware)

Do **not** try to install Picamera2 in the ROS image. Its libcamera/IPA
dependencies need to run on the Raspberry Pi host. Instead, this repository
sends host-encoded H.264 over RTP/UDP and decodes it in Docker. RTP has no RTSP
server or client buffering; with the supplied one-frame receiver queue it is
the lowest-latency practical option for this setup.

Rebuild the server image once, so it includes the GStreamer decoder:

```bash
cd ROBOT01/ROS
docker compose build server_dev
docker compose --profile dev up -d server_dev
docker compose exec server_dev bash
```

In that container, build and source the workspace:

```bash
cd /root/ros2_ws
colcon build --packages-select my_robot_vision
source install/setup.bash
```

On the **Pi host** (outside Docker), install Picamera2 once:

```bash
sudo apt update
sudo apt install python3-picamera2
```

Start the Picamera2 RTP sender on the Pi host in a second terminal:

```bash
cd ROBOT01/ROS
python3 scripts/stream_picamera_rtp.py
```

Then launch the Docker-side ROS receiver in the container terminal:

```bash
ros2 launch my_robot_vision rtp_camera.launch.py
```

This Python sender and the `rtp_camera.launch.py` receiver have been tested
together with the physical Raspberry Pi camera. The sender runs on the Pi
host; the receiver runs inside Docker and publishes `/camera/image_raw` and
`/camera/camera_info`.

The defaults are 640×480 at 30 fps to UDP port 5600. Because the compose
services use `network_mode: host`, `127.0.0.1` reaches the container directly.
For a receiver on another machine, pass its LAN address and ensure UDP 5600 is
allowed:

```bash
python3 scripts/stream_picamera_rtp.py 192.168.1.50 5600
```

Match a non-default port on the ROS side with `port:=5601`. The Python sender
uses Picamera2's H.264 encoder and packetises its output directly as RTP/UDP;
the receiver drops old frames. This favors fresh video over recovery from
packet loss; use the loopback default whenever Docker is running on the same
Pi. Optional tuning is available through `--width`, `--height`, `--fps`, and
`--bitrate` (default: 2 Mbit/s).

For ArUco pose estimation, calibrate the physical camera at the selected image
resolution. Convert the resulting OpenCV `.npz` file into the standard ROS
camera-calibration YAML format (replace `640` and `480` with the calibration
image resolution):

```bash
ros2 run my_robot_vision convert_camera_calibration \
  /home/robot/camera_calibration.npz \
  /home/robot/camera_calibration.yaml \
  --width 640 --height 480
```

The converter accepts camera-matrix keys `camera_matrix`, `cameraMatrix`,
`mtx`, or `K`, and distortion keys `distortion_coefficients`, `dist_coeffs`,
`distCoeffs`, `dist`, or `D`. Pass the generated YAML file to the publisher:

```bash
ros2 launch my_robot_vision pi_camera.launch.py \
  camera_info_url:=file:///home/robot/camera_calibration.yaml
```

Without a calibration file, the node publishes approximate intrinsics only;
the image is valid, but metric pose estimates are not.
