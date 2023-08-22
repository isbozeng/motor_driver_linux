#include "servocontrol.h"
#include <iostream>

ServoMotion::ServoMotion(uint8_t _id, bool _inverse, uint8_t _reduction,
                         float _angleLimitMin, float _angleLimitMax)
    : CtrlStepMotor(_id, _inverse, _reduction, _angleLimitMin, _angleLimitMax)
{
  memset(&servoDrv, 0, sizeof(servo::servoDriveInfo));
  memset(&servoInf, 0, sizeof(servo::servoStatus));
  servoDrv.id = nodeID;
  servoDrv.enableCmd = true;
  servoDrv.accCmd = 250;  
  sm_st = servo::SMS_STS_THREAD::getInstance();
  sm_st->addServo(nodeID, &servoDrv);

  SetEnable(true); // 使能
}

ServoMotion::~ServoMotion() {}

// 角度单位是±180°,角速度值最大298°/s
void ServoMotion::SetAngleWithVelocityLimit(float _angle, float _vel)
{
  // 把目标先做单位转换
  // std::cout<<"angle:"<<_angle<<" vel:"<<_vel<<std::endl;
  u16 Angle_ref = (u16)((_angle + 180) * (4096.0 / 360.0));
  u16 Vel_ref = (u16)(_vel * (50.0 / 4.392));
  sm_st->setPosVel(nodeID, Angle_ref, Vel_ref);
  state = RUNNING;
}

void ServoMotion::SetAngle(float _angle) 
{
  // std::cout<<"angle:"<<_angle<<std::endl;
  SetAngleWithVelocityLimit(_angle, 35);
}

// 使能or解能
void ServoMotion::SetEnable(bool _enable)
{
  sm_st->setEnable(nodeID, _enable);
}

// 传递进来的加速度单位：°/s2
void ServoMotion::SetAcceleration(float _val)
{
  uint8_t acc = (u8)(_val / 8.878);
  if (acc > 255)
  {
    acc = 255;
  }
  sm_st->SetAcceleration(nodeID, acc);
}

// 位置清零
void ServoMotion::ApplyPositionAsHome()
{
  sm_st->ApplyPositionAsHome(nodeID);
}

// 重新使能，//急停
void ServoMotion::Reboot()
{
  sm_st->setEnable(nodeID, false);
  sm_st->setEnable(nodeID, true);
}

// 更新状态，把另一个线程的实时角度值反馈回来
void ServoMotion::UpdateAngle()
{
  // lock_guard<mutex> g_lock(m_lock);
  sm_st->getServoInfo(nodeID, servoInf);

  angle = servoInf.Pos_f;

  // 更新舵机运动状态
  if (servoInf.isMoving == 1)
  {
    state = ServoMotion::RUNNING;
  }
  else if (servoInf.isMoving == 0 && servoInf.isEnable)
  {
    state = ServoMotion::FINISH;
  }
  else if (!servoInf.isEnable)
  {
    state = ServoMotion::STOP;
  }
  // std::cout<<"id:"<<(int)nodeID<<" state:"<<(int)state
  // <<" isEnable:"<<(int)servoInf.isEnable<<std::endl;
}

float ServoMotion::getCurrent()
{
  return servoInf.current * 6.5 * 0.001;
}

float ServoMotion::getTorque()
{
  return 0.0;
}