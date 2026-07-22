cmake_minimum_required(VERSION 3.14)
project(ArduinoNodeTest CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find pthread
find_package(Threads REQUIRED)

# Include the lib/ directory for headers
include_directories(lib)

# Compile library sources
add_library(arduino_node_lib
    lib/arduino_node.cpp
)

# Link LibSerial and Threads to library
target_link_libraries(arduino_node_lib
    PUBLIC
    serial
    Threads::Threads
)

# Executable target
add_executable(test_node test.cpp)

# Link library to executable
target_link_libraries(test_node
    PRIVATE
    arduino_node_lib
)