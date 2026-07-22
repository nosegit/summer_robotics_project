#include "infrared.h"
#define I2C_SDA P0_31
#define I2C_SCL P0_2

mbed::I2C     _myi2c( I2C_SDA,I2C_SCL );
INFRARED      _infrared[2]= {&_myi2c, &_myi2c};


void setup() {
  // put your setup code here, to run once:

  _infrared[0].set_I2C_BUS(0);
  _infrared[1].set_I2C_BUS(1);

  Serial.begin(115200);

  // Wait for Serial Monitor to open (optional for debugging)
  while (!Serial && millis() < 4000);

  Serial.println("Infrared Ready!\n");

}

void loop() {
  
  for(int i = 0; i < 2; i++){
    _infrared[i].Begin();

    Serial.print("IR");
    Serial.print(i);
    Serial.print(":");
    Serial.print(_infrared[i].Get_distance());
    Serial.print(" CM| ");
  }

  Serial.println();

  // Delay loop print to ~10 Hz so the Serial Monitor stays easy to read
  delay(100); 
}
