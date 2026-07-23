#ifndef ARDUINO_STATE_H
#define ARDUINO_STATE_H

#include <mbed.h>
#include <rtos.h>

struct LocalState {
    // Motor 1
    float target_w1 = 0.0;
    float actual_w1 = 0.0;
    float actual_wheel_theta1 = 0.0;

    // Motor 2
    float target_w2 = 0.0;
    float actual_w2 = 0.0;
    float actual_wheel_theta2 = 0.0;

    // Scanning Sensors
    float ir_distance_1 = 0.0;
    float ir_distance_2 = 0.0;
    float servo_angle = 0.0;

    // IMU Data (10 floats)
    float qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    float gx = 0.0, gy = 0.0, gz = 0.0;
    float ax = 0.0, ay = 0.0, az = 0.0;

    // Command Timestamp for Safety Timeout
    unsigned long last_command_time = 0; 
};

class ARDUINO_STATE {
private:
    LocalState _data;
    rtos::Mutex _lock;

public:
    // --- SETTERS ---
    
    // Updates actual wheel speeds calculated from local encoders
    void setActualSpeeds(float a1, float a2) {
        _lock.lock();
        _data.actual_w1 = a1;
        _data.actual_w2 = a2;
        _lock.unlock();
    }

    // Update wheel position
    void setActualWheelPosition(float theta1, float theta2){
        _lock.lock();
        _data.actual_wheel_theta1 = theta1;
        _data.actual_wheel_theta2 = theta2;
        _lock.unlock();
    }

    // Updates sensor readings (Now includes two IR sensors)
    void setSensors(float distance1, float distance2) {
        _lock.lock();
        _data.ir_distance_1 = distance1;
        _data.ir_distance_2 = distance2;
        _lock.unlock();
    }

    // Updates IMU readings safely from the IMU background thread
    void setIMU(float qx, float qy, float qz, float qw, 
                float gx, float gy, float gz, 
                float ax, float ay, float az) {
        _lock.lock();
        _data.qx = qx; _data.qy = qy; _data.qz = qz; _data.qw = qw;
        _data.gx = gx; _data.gy = gy; _data.gz = gz;
        _data.ax = ax; _data.ay = ay; _data.az = az;
        _lock.unlock();
    }

    // Updates target servo angle (Called by communication thread)
    void setTargetServo(float target_angle) {
        _lock.lock();
        _data.servo_angle = target_angle;
        _lock.unlock();
    }

    // Updates targets (Called by communication thread receiving data from RPi)
    void setTargetSpeeds(float t1, float t2) {
        _lock.lock();
        _data.target_w1 = t1;
        _data.target_w2 = t2;
        _data.last_command_time = millis(); // Reset safety timeout timer
        _lock.unlock();
    }

    // --- GETTERS ---

    // Safely copies the full local state to package and send up to the RPi
    LocalState getSnapshot() {
        _lock.lock();
        LocalState copy = _data; 
        _lock.unlock();
        return copy;
    }

    // Used by your local motor control loops to read targets and verify connection safety
    void getTargets(float &t1, float &t2, unsigned long &last_time) {
        _lock.lock();
        t1 = _data.target_w1;
        t2 = _data.target_w2;
        last_time = _data.last_command_time;
        _lock.unlock();
    }

    // Used by the servo control thread to safely read the angle
    float getServo() {
        _lock.lock();
        float angle = _data.servo_angle;
        _lock.unlock();
        return angle;
    }
};

#endif // ARDUINO_STATE_H