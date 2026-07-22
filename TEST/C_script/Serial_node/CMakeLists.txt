cmake_minimum_required(VERSION 3.10)
project(serial_test_project CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PkgConfig to locate libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Include header directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${LIBSERIAL_INCLUDE_DIRS}
)

# Build the test executable
add_executable(serial_test
    serial_test.cpp
    lib/arduino_node.cpp
)

# Link required libraries
target_link_libraries(serial_test
    PRIVATE
    ${LIBSERIAL_LIBRARIES}
    pthread
)