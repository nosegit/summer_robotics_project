#ifndef _MOTOR_H
#define _MOTOR_H

#include <mbed.h>
#include "pid.h"
using namespace std::chrono;

class MOTOR{


  /* PID Variables */
  const float   _PID_INTERVAL_SEC = 0.05;
  mbed::Ticker  _PID_TICKER;
  rtos::Thread  _THREAD_PID;

  private:
  /* MOTOR A */
  mbed::DigitalOut    _DIRA;
  mbed::PwmOut        _PWMA;
  mbed::InterruptIn   _ENCODERA;
  uint32_t            _PWMA_FREQ = 100e3;
  uint32_t            _PWMA_PERIOD_US;
  mbed::Timer         _ENCODERA_TIMER;
  float               _ENCODERA_SPEED;
  long long           _ENCODERA_CNT = 0;
  long long           _ENCODERA_CNT_PREV = 0;
  PIDController       _PIDA;
  float               _ENCODERA_SET_SPEED = 0;

  /* MOTOR B */
  mbed::DigitalOut    _DIRB;
  mbed::PwmOut        _PWMB;
  mbed::InterruptIn   _ENCODERB;
  uint32_t            _PWMB_FREQ = 100e3;
  uint32_t            _PWMB_PERIOD_US;
  mbed::Timer         _ENCODERB_TIMER;
  long long           _ENCODERB_SPEED;
  long long           _ENCODERB_CNT = 0;
  long long           _ENCODERB_CNT_PREV = 0;
  PIDController       _PIDB;
  float               _ENCODERB_SET_SPEED = 0;


  void _Pid_isr();
  void _Pid_loop();

  void _Cal_speed();
  void EncoderA_trigger();
  void EncoderB_trigger();


  public:

  MOTOR(PinName DIRA,PinName PWMA,PinName ENCODERA,PinName DIRB,PinName PWMB,PinName ENCODERB );

  void Begin();
  void Set_dir(int motor, int dir);
  void Set_pwm(int motor, float pwm);
  void Set_frquency(int motor, int frequency);
  void Set_speed(float speedA, float speedB);
  float Get_speed(int motor);
  long long Get_encoder_cnt(int motor){ return motor ? _ENCODERB_CNT : _ENCODERA_CNT ;}
  void Reset_encoder();
  void Print_info();
  void Print_info_CSV();

};



#endif