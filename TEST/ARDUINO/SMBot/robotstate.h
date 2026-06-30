#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <mbed.h>
#include <rtos.h>

struct TelemetryData {
    // Pose
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
    
    // Velocity
    float v_actual = 0.0;
    float w_actual = 0.0;

    // Sensors
    float us_left = 0.0;
    float us_right = 0.0;
    float ir_front = 0.0;
    
    // Command (Written by Communication)
    float target_v = 0.0; // m/s
    float target_w = 0.0; // rad/s
    unsigned long last_command_time = 0; // For Safety Timeout
};

class ROBOTSTATE {
private:
    TelemetryData _data;
    rtos::Mutex _lock;

public:
    // SETTERS (Used by Subsystems)
    void setPose(float x, float y, float t) {
        _lock.lock();
        _data.x = x;
        _data.y = y;
        _data.theta = t;
        _lock.unlock();
    }

    void setVelocity(float v, float w){
      _lock.lock();
      _data.v_actual = v;
      _data.w_actual = w;
      _lock.unlock();
    }

    void setTargetVelocity(float v, float w){
      _lock.lock();
      _data.target_v = v;
      _data.target_w = w;
      _data.last_command_time = millis(); // Reset timeout timer
      _lock.unlock();
    }

    void setSensors(float left, float right, float front) {
        _lock.lock();
        _data.us_left = left;
        _data.us_right = right;
        _data.ir_front = front;
        _lock.unlock();
    }

    // GETTER (Used by Communication)
    TelemetryData getSnapshot() {
        _lock.lock();
        TelemetryData copy = _data; // Copy safely
        _lock.unlock();
        return copy;
    }

    // Getter for the Drive Controller
    void getTargets(float &v, float &w, unsigned long &last_time) {
        _lock.lock();
        v = _data.target_v;
        w = _data.target_w;
        last_time = _data.last_command_time;
        _lock.unlock();
    }
};

#endif