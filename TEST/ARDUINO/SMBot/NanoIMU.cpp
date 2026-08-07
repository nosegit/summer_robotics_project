#include "NanoIMU.h"

constexpr float GRAVITY_MS2 = 9.80665f;
constexpr float DEG_TO_RAD_F = 0.01745329251f;

NanoIMU::NanoIMU() : _is_running(false), _update_interval_ms(20) {
    _latest_data = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
}

bool NanoIMU::begin() {
    // 1. Initialise the LSM9DS1 IMU & AHRS
    _imu.begin();
    _ahrs.begin();
    
    // Set the algorithm to MADGWICK
    _ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
    // _ahrs.setBeta(0.375);


    if (!_imu.connected()) {
        return false;
    }

    // 2. Start and Calibrate
    _imu.start();
    _imu.calibrateGyro();
    _imu.calibrateAccel();
    // _imu.calibrateMag();

    // 3. Flush the first reading
    delay(20);
    _imu.readGyro();
    _imu.readAccel();
    _imu.readMag();
    
    // 4. Start the FreeRTOS Thread
    _is_running = true;
    _thread.start(mbed::callback(this, &NanoIMU::threadLoop));

    return true;
}

ROSImuData NanoIMU::getLatestData() {
    ROSImuData copy;
    _data_mutex.lock();
    copy = _latest_data;
    _data_mutex.unlock();
    return copy;
}

void NanoIMU::threadLoop() {
    while (_is_running) {
        
        // 1. Read raw sensor data from LSM9DS1
        _imu.updateSensorData();

        // 2. Prepare a Right-Handed data struct FOR THE FILTER
        // (Invert Y on both Accel and Gyro BEFORE passing to Madgwick)
        SensorData rh_data;
        rh_data.ax =  _imu.data.ax;
        rh_data.ay = -_imu.data.ay; // Flip Y to Right-Handed (Left)
        rh_data.az =  _imu.data.az;

        rh_data.gx =  _imu.data.gx;
        rh_data.gy = -_imu.data.gy; // Flip Y to Right-Handed (Pitch)
        rh_data.gz =  _imu.data.gz;

        rh_data.mx = 0; 
        rh_data.my = 0; 
        rh_data.mz = 0;

        // 3. Feed RIGHT-HANDED data into the filter
        _ahrs.setData(rh_data);
        _ahrs.update();
        
        // Extract the now mathematically correct Quaternion
        Quaternion q = _ahrs.getQuaternion();

        // 4. LOCK MUTEX AND SAVE FINAL ROS DATA
        _data_mutex.lock();
        
        // Quaternion is ALREADY Right-Handed (No negative signs needed!)
        _latest_data.qw = q.q0;
        _latest_data.qx = q.q1;
        _latest_data.qy = q.q2;
        _latest_data.qz = q.q3;

        // Save raw accel in m/s^2 (using our Right-Handed Y value)
        _latest_data.ax =  rh_data.ax * GRAVITY_MS2;
        _latest_data.ay =  rh_data.ay * GRAVITY_MS2;
        _latest_data.az =  rh_data.az * GRAVITY_MS2;

        // Save raw gyro in rad/s (using our Right-Handed Y value)
        _latest_data.gx =  rh_data.gx * DEG_TO_RAD_F;
        _latest_data.gy =  rh_data.gy * DEG_TO_RAD_F;
        _latest_data.gz =  rh_data.gz * DEG_TO_RAD_F;
        
        _data_mutex.unlock();

        // Sleep to maintain loop frequency
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(_update_interval_ms));
    }
}