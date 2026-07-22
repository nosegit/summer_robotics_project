#include <mbed.h>

#include "motor.h"
#include "communication.h"
#include "arduinostate.h"
#include "servocontroller.h"
#include "infrared.h"
#include "NanoIMU.h"

#define ENABLEUSBSERIAL 1

#define MOTORDIRA P0_4
#define MOTORDIRB P0_5
#define MOTORPWMA P0_27
#define MOTORPWMB P1_2
#define MOTORENCODERA P1_11
#define MOTORENCODERB P1_12

#define TICKPERREVOLUTION 1788  // 298*6

MOTOR _motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);

ARDUINO_STATE _arduino_state;
COMMUNICATION _communication;

#define SERVOPIN P0_3

SERVOController _servo(SERVOPIN);

#define I2C_SDA P0_31
#define I2C_SCL P0_2

mbed::I2C     _myi2c( I2C_SDA,I2C_SCL );
INFRARED      _infrared[2] = {(&_myi2c), (&_myi2c)};

NanoIMU _IMU;

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

  _infrared[0].set_I2C_BUS(0);
  _infrared[1].set_I2C_BUS(1);

  _IMU.begin();

}

void loop() {
  // put your main code here, to run repeatedly:

  // LocalState data = _arduino_state.getSnapshot();
  // Serial.print(data.actual_w1);
  // Serial.print("||");
  // Serial.println(data.actual_w2);

  LocalState data = _arduino_state.getSnapshot();
  _motor.Set_speed_rads(data.target_w1, data.target_w2);
  _servo.write((int)data.servo_angle);

  _infrared[1].Begin();
  _infrared[0].Begin();
  _arduino_state.setSensors(_infrared[0].Get_distance(), _infrared[1].Get_distance());

  ROSImuData imu_data = _IMU.getLatestData();
  _arduino_state.setIMU(imu_data.qx, imu_data.qy, imu_data.qz, imu_data.qw,
                        imu_data.gx, imu_data.gy, imu_data.gz,
                        imu_data.ax, imu_data.ay, imu_data.az);

}
 