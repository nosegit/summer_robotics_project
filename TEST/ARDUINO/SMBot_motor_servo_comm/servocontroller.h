#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>
#include <mbed.h>

// 1. Structure to hold our "Key-Value" pairs
struct PinMapping {
    PinName mbedPin;
    int arduinoPin;
};

// 2. The Complete Arduino Nano 33 BLE "Dictionary" Map
const PinMapping NANO_33_BLE_PIN_MAP[] = {
    // --- Digital Pins ---
    {P1_10, 0},   // D0 (RX)
    {P1_3,  1},   // D1 (TX)
    {P1_11, 2},   // D2
    {P1_12, 3},   // D3
    {P1_15, 4},   // D4
    {P1_13, 5},   // D5
    {P1_14, 6},   // D6
    {P0_23, 7},   // D7
    {P0_21, 8},   // D8
    {P0_27, 9},   // D9
    {P1_2,  10},  // D10
    {P1_1,  11},  // D11 (MOSI)
    {P1_8,  12},  // D12 (MISO)
    {P0_13, 13},  // D13 (SCK / Built-in LED)

    // --- Analog Pins ---
    {P0_4,  A0},
    {P0_5,  A1},
    {P0_30, A2},
    {P0_29, A3},
    {P0_31, A4},  // A4 (SDA)
    {P0_2,  A5},  // A5 (SCL)
    {P0_28, A6},
    {P0_3,  A7}   // A7 
};

// Automatically calculate the size of the dictionary
const int PIN_MAP_SIZE = sizeof(NANO_33_BLE_PIN_MAP) / sizeof(NANO_33_BLE_PIN_MAP[0]);

class SERVOController {

public:

  SERVOController (PinName servo_pin){
    this->_servo_pin = servo_pin;
  }

  void begin(){
    int ard_pin = pinNameToArduinoPin(_servo_pin);
    
    // Only attach and start if the Mbed pin was found in our dictionary
    if (ard_pin != -1) {
        _servo.attach(ard_pin, 500, 2500);
    }
  }

  void write(int angle){
    if(angle >= 0 && angle <= 180){
      _servo.write(angle);
    }
  }

  
private:

  PinName _servo_pin;
  Servo _servo;

  // 3. The "Dictionary Lookup" function
  int pinNameToArduinoPin(PinName pin) {
      for (int i = 0; i < PIN_MAP_SIZE; i++) {
          if (NANO_33_BLE_PIN_MAP[i].mbedPin == pin) {
              return NANO_33_BLE_PIN_MAP[i].arduinoPin; // Return the matched value
          }
      }
      return -1; // Return -1 if the Mbed pin doesn't exist on the Nano's external headers
  }

};

#endif