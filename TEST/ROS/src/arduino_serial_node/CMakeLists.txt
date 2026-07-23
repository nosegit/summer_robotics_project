cmake_minimum_required(VERSION 3.8)
project(arduino_serial_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Find system libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED IMPORTED_TARGET libserial)

# Define the test executable
add_executable(test_arduino 
  src/test.cpp
  # If test.cpp relies on ArduinoNode.cpp, uncomment the line below:
  lib/arduino_node/arduino_node.cpp 
)

# Link the executable against system libserial
target_link_libraries(test_arduino PRIVATE PkgConfig::LibSerial)

target_include_directories(test_arduino PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
)

# Install the executable so ROS can run it
install(TARGETS test_arduino
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
