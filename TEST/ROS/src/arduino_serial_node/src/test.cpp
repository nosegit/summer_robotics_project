#include <iostream>
#include <thread>
#include <chrono>
#include "arduino_node/arduino_node.hpp"

int main() {
    // Replace with your actual serial port (e.g., "/dev/ttyUSB0" or "/dev/ttyACM0")
    std::string port = "/dev/ttyACM0"; 
    
    std::cout << "Starting Arduino Node Test on " << port << "..." << std::endl;
    
    ArduinoNode node(port, 115200, "Arduino 1");
    node.start();

    // Give it a brief moment to connect
    std::this_thread::sleep_for(std::chrono::seconds(1));

    for (int i = 0; i < 10; ++i) {
        // Send motor/servo target command
        node.sendCommand(1.5f, -1.5f, 90.0f);

        // Retrieve current telemetry state
        ArduinoData data = node.getData();

        std::cout << "\n--- Telemetry Read #" << (i + 1) << " ---" << std::endl;
        std::cout << "Connected: " << (data.connected ? "YES" : "NO") << std::endl;
        std::cout << "Timestamp: " << data.timestamp << " ms" << std::endl;
        std::cout << "W1 Actual: " << data.w1_actual << " rad/s | W2 Actual: " << data.w2_actual << " rad/s" << std::endl;
        std::cout << "IR 1: " << data.ir_distance_1 << " mm | IR 2: " << data.ir_distance_2 << " mm" << std::endl;
        std::cout << "Servo Angle: " << data.servo_angle << " deg" << std::endl;
        std::cout << "Quat (w,x,y,z): (" << data.qw << ", " << data.qx << ", " << data.qy << ", " << data.qz << ")" << std::endl;
        std::cout << "Gyro (x,y,z): (" << data.gx << ", " << data.gy << ", " << data.gz << ")" << std::endl;
        std::cout << "Accel (x,y,z): (" << data.ax << ", " << data.ay << ", " << data.az << ")" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "\nStopping Arduino Node..." << std::endl;
    node.stop();

    return 0;
}