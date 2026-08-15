cmake_minimum_required(VERSION 3.8)
project(my_robot_nav2_behavior)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

add_library(mecanum_escape_behavior SHARED
  src/mecanum_escape.cpp
)
target_compile_features(mecanum_escape_behavior PUBLIC cxx_std_17)
target_include_directories(mecanum_escape_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_escape_behavior
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  tf2
)

pluginlib_export_plugin_description_file(nav2_core mecanum_escape_plugin.xml)

install(
  TARGETS mecanum_escape_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES mecanum_escape_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mecanum_escape_behavior)
ament_export_dependencies(
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  tf2
)

ament_package()
