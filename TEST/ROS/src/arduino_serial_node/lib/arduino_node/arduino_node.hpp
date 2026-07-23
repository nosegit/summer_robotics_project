#pragma once

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <libserial/SerialPort.h>

// --- FIXED CONFIGURATIONS ---
constexpr size_t TELEMETRY_PAYLOAD_SIZE = 33;
constexpr size_t TELEMETRY_PACKET_SIZE = 35;
constexpr uint8_t TELEMETRY_HEADER = 0xA5;
constexpr uint8_t COMMAND_HEADER = 0x5A;

// Data structure mirroring the Python dataclass
struct ArduinoData {
    uint32_t timestamp = 0;
    float w1_actual = 0.0f;
    float w2_actual = 0.0f;
    uint16_t ir_distance_1 = 0;
    uint16_t ir_distance_2 = 0;
    float servo_angle = 0.0f;
    
    // IMU Data
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    
    bool connected = false;
};

// Packed struct for incoming telemetry mapping '<IhhHHBhhhhhhhhhh'
#pragma pack(push, 1)
struct TelemetryPayload {
    uint32_t timestamp;
    int16_t w1_actual;
    int16_t w2_actual;
    uint16_t ir_distance_1;
    uint16_t ir_distance_2;
    uint8_t servo_angle;
    int16_t qx;
    int16_t qy;
    int16_t qz;
    int16_t qw;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
};

// Packed struct for outgoing command mapping '<hhB'
struct CommandPayload {
    int16_t w1_target;
    int16_t w2_target;
    uint8_t servo_target;
};
#pragma pack(pop)

class ArduinoNode {
public:
    ArduinoNode(const std::string& port, int baudrate = 115200, const std::string& name = "Arduino");
    ~ArduinoNode();

    void start();
    void stop();
    void sendCommand(float w1_target, float w2_target, float servo_target);
    ArduinoData getData();

private:
    void readLoop();

    std::string port_;
    int baud_;
    std::string name_;
    
    LibSerial::SerialPort serial_;
    ArduinoData data_;
    
    std::mutex mtx_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::chrono::steady_clock::time_point last_packet_time_;
};