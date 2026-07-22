#ifndef _INFRARED_H
#define _INFRARED_H

#include <Arduino.h>
#include <mbed.h>
#include "I2C.h"

class INFRARED{

  private:
    mbed::I2C*  _I2C;
    float       _DISTANCE;

  public:

    INFRARED(mbed::I2C* I2C): _I2C(I2C) {}

    float Get_distance(){ return _DISTANCE; }
    void Begin(){

      const char mux_cmd = 0x01;
      const char mux_addr = 0xEE;
      _I2C->write(mux_addr, &mux_cmd, 1);

      char cmd[2];
      cmd[0] = 0x5E;
      cmd[1] = 0x00;
      _I2C->write(0x80, cmd, 1);
      _I2C->read(0x80, cmd, 2);

      _DISTANCE = (float)(cmd[0] * 16.0 + cmd[1])/16.0/(float)pow(2,0x02);

    }


};


#endif