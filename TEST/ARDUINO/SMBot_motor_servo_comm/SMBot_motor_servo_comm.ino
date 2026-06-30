#include <mbed.h>

#include "motor.h"
#include "robotstate.h"
#include "communication.h"
#include "arduinostate.h"
#include "servocontroller.h"

#define ENABLEUSBSERIAL 1

#define MOTORDIRA P0_4
#define MOTORDIRB P0_5
#define MOTORPWMA P0_27
#define MOTORPWMB P1_2
#define MOTORENCODERA P1_11
#define MOTORENCODERB P1_12

#define TICKPERREVOLUTION 1788  // 298*6

// MOTOR _motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);
MOTOR _motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);

ARDUINO_STATE _arduino_state;
COMMUNICATION _communication;

#define SERVOPIN P0_3

SERVOController _servo(SERVOPIN, &_arduino_state);


void setup() {
  // put your setup code here, to run once:

#if ENABLEUSBSERIAL
  Stream& _mySerial = Serial;
  Serial.begin(115200);
#else
  Stream& _mySerial = Serial1;
  Serial1.begin(115200);
#endif

  _communication.begin(&_arduino_state, &_mySerial);

  _motor.Begin(&_arduino_state);

  _servo.begin();

}

void loop() {
  // put your main code here, to run repeatedly:

  // LocalState data = _arduino_state.getSnapshot();
  // Serial.print(data.actual_w1);
  // Serial.print("||");
  // Serial.println(data.actual_w2);

  LocalState data = _arduino_state.getSnapshot();
  _motor.Set_speed_rads(data.target_w1, data.target_w2);

}
 