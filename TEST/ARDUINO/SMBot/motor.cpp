#include "motor.h"

MOTOR::MOTOR(PinName DIRA,PinName PWMA,PinName ENCODERA,PinName DIRB,PinName PWMB,PinName ENCODERB ):
    _DIRA(DIRA), _PWMA(PWMA), _ENCODERA(ENCODERA),
    _DIRB(DIRB), _PWMB(PWMB), _ENCODERB(ENCODERB),
    _PIDA(0.1,0.5, 0.001, _PID_INTERVAL_SEC),
    _PIDB(0.1,0.5, 0.001, _PID_INTERVAL_SEC),
    _THREAD_PID(osPriorityRealtime)
{
  _PWMA_PERIOD_US = 1e6/_PWMA_FREQ; 
  _PWMB_PERIOD_US = 1e6/_PWMB_FREQ; 

  _ENCODERA.rise(mbed::callback(this, &MOTOR::EncoderA_trigger));
  _ENCODERA.fall(mbed::callback(this, &MOTOR::EncoderA_trigger));

  _ENCODERB.rise(mbed::callback(this, &MOTOR::EncoderB_trigger));
  _ENCODERB.fall(mbed::callback(this, &MOTOR::EncoderB_trigger));
}

void MOTOR::Begin(){
  Set_frquency(0, _PWMA_FREQ);
  Set_frquency(1, _PWMB_FREQ);

  Set_dir(0, 0);
  Set_dir(1, 0);

  Set_pwm(0, 0);
  Set_pwm(1, 0);

  Set_speed(0.1, 0.1);

  _ENCODERA_TIMER.reset();
  _ENCODERB_TIMER.reset();

  _ENCODERA_TIMER.start();
  _ENCODERB_TIMER.start();

  _PID_TICKER.attach_us(mbed::callback(this,&MOTOR::_Pid_isr), _PID_INTERVAL_SEC * 1e6);
  _THREAD_PID.start(mbed::callback(this,&MOTOR::_Pid_loop));

} 

void MOTOR::Set_dir(int motor, int dir){
  switch (motor){
    case 0: _DIRA = dir ? 1:0; break;
    case 1: _DIRB = dir ? 0:1; break;
  }
}

void MOTOR::Set_pwm(int motor, float pwm){
  switch (motor){
    case 0: _PWMA.write(pwm < 100 ? (float)pwm/100:1); break;
    case 1: _PWMB.write(pwm < 100 ? (float)pwm/100:1); break;
  }
}

void MOTOR::Set_frquency(int motor, int frequency){
  switch (motor){
    case 0: _PWMA_FREQ = frequency; _PWMA_PERIOD_US = 1e6/_PWMA_FREQ; _PWMA.period_us(_PWMA_PERIOD_US); break;
    case 1: _PWMB_FREQ = frequency; _PWMB_PERIOD_US = 1e6/_PWMB_FREQ; _PWMB.period_us(_PWMB_PERIOD_US); break;
  }
}

void MOTOR::Set_speed(float speedA,float speedB){
  _ENCODERA_SET_SPEED = speedA;
  _ENCODERB_SET_SPEED = speedB;
}

float MOTOR::Get_speed(int motor){

  return motor ? _ENCODERB_SPEED:_ENCODERA_SPEED;

}

void MOTOR::_Pid_loop(){
  for(;;){
    rtos::ThisThread::flags_wait_all(0x01);
    _Cal_speed();

    float pwmA = _PIDA.compute(_ENCODERA_SET_SPEED, _ENCODERA_SPEED);
    float pwmB = _PIDB.compute(_ENCODERB_SET_SPEED, _ENCODERB_SPEED);

    Set_dir(0,pwmA > 0 ? 0:1);
    Set_dir(1,pwmB > 0 ? 0:1);

    Set_pwm(0, abs(pwmA));
    Set_pwm(1, abs(pwmB));

  }
}

void MOTOR::_Pid_isr(){
  _THREAD_PID.flags_set(0x01);
}

void MOTOR::_Cal_speed(){

  unsigned long diff_t;

  /* MOTOR A */
  _ENCODERA_TIMER.stop();

  diff_t = duration_cast<microseconds>(_ENCODERA_TIMER.elapsed_time()).count();
  float _ENCODERA_SPEED_RAW;
  _ENCODERA_SPEED_RAW = (float)(_ENCODERA_CNT-_ENCODERA_CNT_PREV)*1e6/(float)diff_t;
  _ENCODERA_SPEED = 0.6*_ENCODERA_SPEED + 0.4*_ENCODERA_SPEED_RAW;

  _ENCODERA_TIMER.reset();
  _ENCODERA_TIMER.start();

  _ENCODERA_CNT_PREV = _ENCODERA_CNT;

  /* MOTOR B */
  _ENCODERB_TIMER.stop();
  diff_t = duration_cast<microseconds>(_ENCODERB_TIMER.elapsed_time()).count();
  float _ENCODERB_SPEED_RAW;
  _ENCODERB_SPEED_RAW = (float)(_ENCODERB_CNT-_ENCODERB_CNT_PREV)*1e6/(float)diff_t;
  _ENCODERB_SPEED = 0.6*_ENCODERB_SPEED + 0.4*_ENCODERB_SPEED_RAW;

  _ENCODERB_TIMER.reset();
  _ENCODERB_TIMER.start();

  _ENCODERB_CNT_PREV = _ENCODERB_CNT;

}

void MOTOR::EncoderA_trigger(){

  _ENCODERA_CNT += _DIRA ? -1:1;

}

void MOTOR::EncoderB_trigger(){

  _ENCODERB_CNT += _DIRB ? 1:-1;

}

void MOTOR::Reset_encoder(){
  _ENCODERA_CNT = _ENCODERB_CNT = 0;
}


void MOTOR::Print_info(){
  if(Serial){
    for(int i = 0 ; i < 2; i++){
      Serial.print(i);
      Serial.print("encoder:");
      Serial.println(Get_encoder_cnt(i));
      Serial.print(i);
      Serial.print("speed:");
      Serial.println(i ? _ENCODERB_SPEED: _ENCODERA_SPEED);
    }
  }
}