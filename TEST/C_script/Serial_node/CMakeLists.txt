cmake_minimum_required(VERSION 3.20)

project(TestArduino)

set(CMAKE_CXX_STANDARD 20)

# 1. Load the PkgConfig module
find_package(PkgConfig REQUIRED)
# 2. Ask PkgConfig to find libserial and create a CMake target for it
pkg_check_modules(LibSerial REQUIRED IMPORTED_TARGET libserial)

find_package(Threads REQUIRED)

add_library(ArduinoNode
    lib/arduino_node/arduino_node.cpp
)

target_include_directories(ArduinoNode PUBLIC
    lib/arduino_node
)

# 3. Link against the PkgConfig target
target_link_libraries(ArduinoNode PUBLIC
    PkgConfig::LibSerial
    Threads::Threads
)

add_executable(TestArduino
    src/test.cpp
)

target_link_libraries(TestArduino PRIVATE
    ArduinoNode
)