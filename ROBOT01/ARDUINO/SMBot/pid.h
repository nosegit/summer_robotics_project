/*
 * PIDController.h
 * A simple PID controller class.
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "mbed.h"

class PIDController {
public:
    /**
     * @brief Constructor for the PID controller.
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param dt The fixed time interval (in seconds) between calculations
     */
    PIDController(float Kp, float Ki, float Kd, float dt) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->dt = dt;
        this->outMin = -100.0; // Default to motor range
        this->outMax = 100.0;
        this->integral = 0.0;
        this->prevError = 0.0;
    }

    /**
     * @brief Computes the PID control output.
     * @param setpoint The desired target value
     * @param measuredValue The current value from the sensor
     * @return The calculated control output (e.g., PWM)
     */
    float compute(float setpoint, float measuredValue) {
        // 1. Calculate Error
        float error = setpoint - measuredValue;

        // 2. Proportional Term
        float pTerm = Kp * error;

        // 3. Integral Term (with anti-windup)
        integral += error * dt;
        if (integral > outMax) integral = outMax;
        else if (integral < outMin) integral = outMin;
        float iTerm = Ki * integral;

        // 4. Derivative Term
        float derivative = (error - prevError) / dt;
        float dTerm = Kd * derivative;

        // 5. Calculate Total Output
        float output = pTerm + iTerm + dTerm;

        // 6. Clamp the final output
        if (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;

        // 7. Save current error for the next loop
        prevError = error;

        return output;
    }

    /**
     * @brief Sets the output limits (e.g., 0-255 for PWM).
     */
    void setOutputLimits(float min, float max) {
        outMin = min;
        outMax = max;
    }

    /**
     * @brief Resets the integral and derivative history.
     */
    void reset() {
        integral = 0.0;
        prevError = 0.0;
    }

private:
    float Kp, Ki, Kd;
    float dt;
    float integral, prevError;
    float outMin, outMax;
};

#endif // PID_CONTROLLER_H