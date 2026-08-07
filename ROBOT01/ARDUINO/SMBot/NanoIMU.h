#ifndef NANO_IMU_H
#define NANO_IMU_H

#include <Arduino.h>
#include <rtos.h>
#include <Arduino_LSM9DS1.h> // Back to official Arduino library
#include <ReefwingAHRS.h>    // Keeping Reefwing just for the Madgwick filter
#include <ReefwingLSM9DS1.h>

struct ROSImuData {
    float qx, qy, qz, qw;
    float gx, gy, gz;
    float ax, ay, az;
};

class NanoIMU {
public:
    NanoIMU();
    // void begin(uint32_t update_interval_ms = 20);
    bool begin();
    ROSImuData getLatestData();

private:
    ReefwingLSM9DS1 _imu;
    ReefwingAHRS _ahrs;

    // Manual calibration offsets
    float _gyro_bias_x = 0.0f;
    float _gyro_bias_y = 0.0f;
    float _gyro_bias_z = 0.0f;

    rtos::Thread _thread;
    rtos::Mutex  _data_mutex;
    bool         _is_running;
    uint32_t     _update_interval_ms;

    ROSImuData   _latest_data;

    void threadLoop();
};

#endif // NANO_IMU_H