#include "servocontrol.h"
#include <iostream>

ServoMotion::ServoMotion(uint8_t _id, bool _inverse, uint8_t _reduction,
                         float _angleLimitMin, float _angleLimitMax)
    : CtrlStepMotor(_id, _inverse, _reduction, _angleLimitMin, _angleLimitMax) {
  sm_st = SMS_STS_THREAD::getInstance();
  sm_st->setID(nodeID);
  SetEnable(true); //使能
}

ServoMotion::~ServoMotion() {}

//角度单位是±180°,角速度值最大298°/s
void ServoMotion::SetAngleWithVelocityLimit(float _angle, float _vel) {
  //把目标先做单位转换
  u16 Angle_ref = (u16)((_angle + 180) * (4096.0 / 360.0));
  u16 Vel_ref = (u16)(_vel * (50.0 / 4.392));
  std::lock_guard<std::mutex> mylock(SMS_STS_THREAD::mtx);
  sm_st->WritePosEx(nodeID, Angle_ref, Vel_ref, ACC);
  state = RUNNING;
}

//使能or解能
void ServoMotion::SetEnable(bool _enable) {
  std::lock_guard<std::mutex> mylock(SMS_STS_THREAD::mtx);  
  if (_enable == true) {
    sm_st->writeByte(nodeID, SMS_STS_TORQUE_ENABLE, 1); //使能
  } else if (_enable == false) {
    sm_st->writeByte(nodeID, SMS_STS_TORQUE_ENABLE, 0); //解能
  }
  state = _enable ? FINISH : STOP;
}

//传递进来的加速度单位：°/s2
void ServoMotion::SetAcceleration(float _val) {
  ACC = (u8)(_val / 8.878);
  if (ACC > 255) {
    cout << "Acceleration is over Max." << endl;
    ACC = 255;
  }
}

//位置清零
void ServoMotion::ApplyPositionAsHome() {
  std::lock_guard<std::mutex> mylock(SMS_STS_THREAD::mtx);  
  sm_st->writeByte(
      nodeID, SMS_STS_TORQUE_ENABLE,
      128); //当前位置设置为2048，中间值,软件记为0，后面收到位置值做处理//写入，返回一个值，1或者0
  cout << "This angle is 0" << endl;
}

//重新使能，//急停
void ServoMotion::Reboot() {
  std::lock_guard<std::mutex> mylock(SMS_STS_THREAD::mtx);  
  sm_st->writeByte(nodeID, SMS_STS_TORQUE_ENABLE, 0);
  sm_st->writeByte(nodeID, SMS_STS_TORQUE_ENABLE, 1);
}

//更新状态，把另一个线程的实时角度值反馈回来
void ServoMotion::UpdateAngle() {
  // lock_guard<mutex> g_lock(m_lock);
  angle = sm_st->getPos(nodeID);
  // std::cout<<"Motor "<< nodeID <<" Angle is "<<angle<<std::endl;
  s1.Move = sm_st->getMove(nodeID);
  //更新舵机运动状态
  if (s1.Move == 1) {
    state = ServoMotion::RUNNING;
  } else if (s1.Move == 0) {
    state = ServoMotion::FINISH;
  } else {
  }
  //扭矩开关
  s1.T_switch = sm_st->getT_enable(nodeID);
  if (s1.T_switch != -1) {
  } else {
  }
  s1.Speed_f = sm_st->getVel(nodeID);
}
