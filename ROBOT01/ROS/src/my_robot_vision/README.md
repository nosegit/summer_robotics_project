# my_robot_vision

This package publishes the robot's front camera using standard ROS 2 topics:

- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)

The Gazebo sensor and bridge are started by `my_robot_sim/gazebo.launch.py`.

## Raspberry Pi Camera Module 2

On the Raspberry Pi, enable the CSI camera in Raspberry Pi OS and install the
Picamera2 runtime (`python3-picamera2`). Build this package in the ROS 2
workspace, source the overlay, then run:

```bash
ros2 launch my_robot_vision pi_camera.launch.py
```

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
