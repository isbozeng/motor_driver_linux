#ifndef SERVO_MOTION_HPP
#define SERVO_MOTION_HPP
#include "iostream"
#include "SCServo.h"
#include "SMS_STS_THREAD.h"
#include "ctrl_step.hpp"
#include <mutex>

using namespace std;

class ServoMotion : public CtrlStepMotor {
public:
  struct Statu {
  //读取信息
  float Pos_f;
  float Speed_f;
  int Load;
  int Voltage;
  int Temper;
  int Move;
  int Current;
  u8 T_switch;
};
  ServoMotion(uint8_t _id, bool _inverse, uint8_t _reduction,
              float _angleLimitMin, float _angleLimitMax);
  virtual ~ServoMotion(); //有虚函数的继承，子类的析构函数也标虚函数
  // virtual void SetAngle(float _angle);
  virtual void SetAngleWithVelocityLimit(float _angle, float _vel);
  virtual void SetEnable(bool _enable);
  virtual void SetAcceleration(float _val);
  virtual void ApplyPositionAsHome();
  virtual void Reboot();
  virtual void UpdateAngle();

  SMS_STS_THREAD *sm_st;
  struct Statu s1; //保存电机状态参数


private:
  float sixForce;
  float current;
  bool isEnable = false;
  u8 ACC = 250;
};

#endif
