cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch and runtime asset directories to the share folder
install(
  DIRECTORY config launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/move_to_odom_distance.py
    scripts/rotate_wheel_to_position.py
    scripts/rotate_to_odom_angle.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
