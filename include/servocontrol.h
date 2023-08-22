#ifndef SERVO_MOTION_HPP
#define SERVO_MOTION_HPP
#include "iostream"
#include "SCServo.h"
#include "SMS_STS_THREAD.h"
#include "ctrl_step.hpp"
#include <mutex>

using namespace std;
  class ServoMotion : public CtrlStepMotor
  {
  public:

    ServoMotion(uint8_t _id, bool _inverse, uint8_t _reduction,
                float _angleLimitMin, float _angleLimitMax);
    virtual ~ServoMotion(); // 有虚函数的继承，子类的析构函数也标虚函数
    virtual void SetAngle(float _angle) override;
    virtual void SetAngleWithVelocityLimit(float _angle, float _vel) override;
    virtual void SetEnable(bool _enable) override;
    virtual void SetAcceleration(float _val) override;
    virtual void ApplyPositionAsHome() override;
    virtual void Reboot() override;
    virtual void UpdateAngle() override;
    virtual float getCurrent() override;
    virtual float getTorque() override;
    servo::SMS_STS_THREAD *sm_st = nullptr;
    servo::servoStatus servoInf; // 保存电机状态参数
    servo::servoDriveInfo servoDrv;//电机驱动数据
  };  



#endif
