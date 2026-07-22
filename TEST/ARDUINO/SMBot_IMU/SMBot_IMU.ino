#include "NanoIMU.h"

NanoIMU imu;

void setup() {
    Serial.begin(115200);
    
    // Wait for Serial Monitor to open (optional for debugging)
    while (!Serial && millis() < 4000);

    Serial.println("Initializing Nano 33 BLE IMU...");
    Serial.println("Keep robot still for gyro calibration...");

    imu.begin();

    Serial.println("IMU Ready!\n");
}

void loop() {
    // Read the latest float values from the background thread
    ROSImuData data = imu.getLatestData();

    // --- Print Quaternion (ROS 2 ENU Frame) ---
    Serial.print("Q [x,y,z,w]: ");
    Serial.print(data.qx, 4); Serial.print(", ");
    Serial.print(data.qy, 4); Serial.print(", ");
    Serial.print(data.qz, 4); Serial.print(", ");
    Serial.print(data.qw, 4);

    // --- Print Gyroscope (rad/s) ---
    Serial.print(" | Gyro [rad/s]: ");
    Serial.print(data.gx, 3); Serial.print(", ");
    Serial.print(data.gy, 3); Serial.print(", ");
    Serial.print(data.gz, 3);

    // --- Print Accelerometer (m/s^2) ---
    Serial.print(" | Accel [m/s^2]: ");
    Serial.print(data.ax, 2); Serial.print(", ");
    Serial.print(data.ay, 2); Serial.print(", ");
    Serial.println(data.az, 2);

    // Delay loop print to ~10 Hz so the Serial Monitor stays easy to read
    delay(100); 
}