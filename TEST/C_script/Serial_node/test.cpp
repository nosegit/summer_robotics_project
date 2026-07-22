#include "lib/arduino_node.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // ADJUST THESE PORT NAMES TO MATCH YOUR SYSTEM (e.g., /dev/ttyACM0 or /dev/ttyUSB0)
    std::string port1 = "/dev/ttyACM0";
    std::string port2 = "/dev/ttyACM1";

    ArduinoNode ard1(port1, 115200, "Arduino_Front");
    ArduinoNode ard2(port2, 115200, "Arduino_Rear");

    std::cout << "Starting Arduino Serial Nodes..." << std::endl;
    ard1.start();
    ard2.start();

    // Run test loop for 10 seconds
    for (int i = 0; i < 100; ++i) {
        // Send command: w1_target, w2_target, servo_target
        ard1.send_command(1.5f, -1.5f, 90.0f);
        ard2.send_command(1.5f, -1.5f, 45.0f);

        // Read data back
        ArduinoData data1 = ard1.get_data();
        ArduinoData data2 = ard2.get_data();

        std::cout << "--- [TEST TICK " << i << "] ---" << std::endl;
        std::cout << "ARD1 (Front) -> Connected: " << data1.connected
                  << " | W1: " << data1.w1_actual << " rad/s"
                  << " | W2: " << data1.w2_actual << " rad/s"
                  << " | IR: " << data1.ir_distance
                  << " | Servo: " << data1.servo_angle << " deg" << std::endl;

        std::cout << "ARD2 (Rear)  -> Connected: " << data2.connected
                  << " | W1: " << data2.w1_actual << " rad/s"
                  << " | W2: " << data2.w2_actual << " rad/s"
                  << " | IR: " << data2.ir_distance
                  << " | Servo: " << data2.servo_angle << " deg" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Stopping Nodes..." << std::endl;
    ard1.stop();
    ard2.stop();

    return 0;
}