cmake_minimum_required(VERSION 3.8)
project(arduino_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Find system libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED IMPORTED_TARGET libserial)

# Create shared library plugin
add_library(${PROJECT_NAME} SHARED
  src/arduino_node.cpp
  src/arduino_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME}
  PkgConfig::LibSerial
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Export plugin description XML
pluginlib_export_plugin_description_file(hardware_interface arduino_hardware_interface.xml)

# Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY test_file
  DESTINATION share/${PROJECT_NAME}
)

ament_package()