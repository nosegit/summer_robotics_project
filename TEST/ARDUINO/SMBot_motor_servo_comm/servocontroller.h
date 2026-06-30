#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>
#include <mbed.h>
#include <rtos.h>

#include "arduinostate.h"

class SERVOController {

public:

  SERVOController (PinName servo_pin, ARDUINO_STATE* state_ptr){
    this->_servo_pin = servo_pin;
    this->_state = state_ptr;
  }

  void begin(){
    _servo.attach(_servo_pin, 1000, 2000);
    _thread_servo.start(mbed::callback(this, &SERVOController::loop));
  }

  
private:

  PinName _servo_pin;
  Servo _servo;
  rtos::Thread _thread_servo;
  ARDUINO_STATE* _state;

  void loop(){
    _servo.write(_state->getServo());
    rtos::ThisThread::sleep_for(33); 
  }

};


#endif