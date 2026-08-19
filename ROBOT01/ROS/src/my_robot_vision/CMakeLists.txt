cmake_minimum_required(VERSION 3.8)
project(my_robot_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubeDetection.msg"
  "msg/CubeDetectionArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  FILES
    my_robot_vision/convert_npz_to_camera_info_yaml.py
    my_robot_vision/pi_camera_publisher.py
    my_robot_vision/rtp_camera_publisher.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
install(
  DIRECTORY my_robot_vision/aruco_pose
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  scripts/aruco_pose_estimator
  scripts/convert_camera_calibration
  scripts/pi_camera_publisher
  scripts/rtp_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY camera_info DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
