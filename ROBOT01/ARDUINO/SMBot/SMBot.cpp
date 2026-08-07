#include "SMBot.h"

SMBOT::SMBOT() :
  _motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB),
  _servo(SERVOPIN),
  _myi2c(I2C_SDA, I2C_SCL),
  _infrared{ {&_myi2c}, {&_myi2c} }
{
}

void SMBOT::setup() {
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

  // Start the 50Hz main loop thread
  _thread_main.start(mbed::callback(this, &SMBOT::Main_loop));
}

void SMBOT::Main_loop() {
  for (;;) {
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

    rtos::ThisThread::sleep_for(20);  // 50Hz (20ms period)
  }
}