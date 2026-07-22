#include "NanoIMU.h"

constexpr float GRAVITY_MS2 = 9.80665f;
constexpr float DEG_TO_RAD_F = 0.01745329251f;

NanoIMU::NanoIMU() : _is_running(false), _update_interval_ms(20) {
    _latest_data = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
}

void NanoIMU::begin(uint32_t update_interval_ms) {
    _update_interval_ms = update_interval_ms;
    
    // 1. Initialize Hardware and Filter
    if (!IMU.begin()) {
        while (1); // Halt if IMU fails
    }
    
    _ahrs.begin();
    _ahrs.setDOF(DOF::DOF_6);
    _ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
    _ahrs.setBeta(0.041f); 
    
    // 2. Hardware Warm-up Delay
    delay(1000); 
    
    // 3. Bulletproof Manual Calibration
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    int samples = 0;
    
    for (int i = 0; i < 200; i++) {
        float gx, gy, gz;
        if (IMU.readGyroscope(gx, gy, gz)) {
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
            samples++;
        }
        delay(5); 
    }
    
    if (samples > 0) {
        _gyro_bias_x = sum_gx / samples;
        _gyro_bias_y = sum_gy / samples;
        _gyro_bias_z = sum_gz / samples;
    }

    // 4. Start RTOS Thread
    _is_running = true;
    _thread.start(mbed::callback(this, &NanoIMU::threadLoop));
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
        float ax, ay, az;
        float gx, gy, gz;

        // Only process if fresh data is available
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            IMU.readAcceleration(ax, ay, az); // Returns values in 'g'
            IMU.readGyroscope(gx, gy, gz);    // Returns values in 'deg/s'

            SensorData rh_data;
            
            // 1. ACCELEROMETER: Invert Y for Right-Hand Rule
            rh_data.ax = ax;
            rh_data.ay = -ay; 
            rh_data.az = az;

            // 2. GYROSCOPE: Subtract bias, then Invert Y for Right-Hand Rule
            rh_data.gx =  (gx - _gyro_bias_x);
            rh_data.gy = -(gy - _gyro_bias_y); 
            rh_data.gz =  (gz - _gyro_bias_z);

            // 3. Update Madgwick Filter
            _ahrs.setData(rh_data);
            _ahrs.update();
            Quaternion q = _ahrs.getQuaternion();

            // 4. Convert to ROS Standards
            float ax_ros = rh_data.ax * GRAVITY_MS2;
            float ay_ros = rh_data.ay * GRAVITY_MS2;
            float az_ros = rh_data.az * GRAVITY_MS2;

            float gx_ros = rh_data.gx * DEG_TO_RAD_F;
            float gy_ros = rh_data.gy * DEG_TO_RAD_F;
            float gz_ros = rh_data.gz * DEG_TO_RAD_F;

            // 5. Lock and Save
            _data_mutex.lock();
            _latest_data.qw = q.q0;
            _latest_data.qx = q.q1;
            _latest_data.qy = q.q2;
            _latest_data.qz = q.q3;

            _latest_data.ax = ax_ros;
            _latest_data.ay = ay_ros;
            _latest_data.az = az_ros;

            _latest_data.gx = gx_ros;
            _latest_data.gy = gy_ros;
            _latest_data.gz = gz_ros;
            _data_mutex.unlock();
        }

        rtos::ThisThread::sleep_for(std::chrono::milliseconds(_update_interval_ms));
    }
}