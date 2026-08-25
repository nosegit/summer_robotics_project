#ifndef _SMBOT_H
#define _SMBOT_H

#include <mbed.h>
#include "motor.h"
#include "communication.h"
#include "arduinostate.h"
#include "servocontroller.h"
#include "infrared.h"
#include "NanoIMU.h"

/*
ARB_L : USB (1)
ARB_R : serial (0)
*/
#define ENABLEUSBSERIAL 0

#define MOTORDIRA P0_4
#define MOTORDIRB P0_5
#define MOTORPWMA P0_27
#define MOTORPWMB P1_2
#define MOTORENCODERA P1_11
#define MOTORENCODERB P1_12

// #define TICKPERREVOLUTION 1788  // 298*6 defined in motor.h
#define SERVOPIN P0_3
#define I2C_SDA P0_31
#define I2C_SCL P0_2

/*
ARB_L(gripper): close 110, open 0
ARB_R(lift): down 80 (grip 70), up 180->140
*/
#define DEFAULT_SERVO_ANGLE 80

class SMBOT {
public:
  SMBOT();
  void setup();

private:
  /* Main Loop Thread */
  void Main_loop();
  rtos::Thread _thread_main;

  /* Actuators */
  MOTOR _motor;
  SERVOController _servo;

  /* Communication and State */
  ARDUINO_STATE _arduino_state;
  COMMUNICATION _communication;

  /* Sensors */
  mbed::I2C _myi2c;
  INFRARED _infrared[2];
  NanoIMU _IMU;
};

#endif