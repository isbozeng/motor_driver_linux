#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP
#include <stdint.h>
class CtrlStepMotor
{
public:
    enum State
    {
        RUNNING,
        FINISH,
        STOP
    };

    const uint32_t CTRL_CIRCLE_COUNT = 200 * 256;

    CtrlStepMotor(uint8_t _id, bool _inverse = false, float _reduction = 1.0,
                  float _angleLimitMin = -180.0, float _angleLimitMax = 180.0) : nodeID(_id),
                                                                                 inverseDirection(_inverse),
                                                                                 reduction(_reduction),
                                                                                 angleLimitMin(_angleLimitMin),
                                                                                 angleLimitMax(_angleLimitMax),
                                                                                 state(STOP)
    {
    }
    virtual ~CtrlStepMotor() = default;
    uint8_t nodeID;
    float angle = 0; // getAngle
    float angleLimitMax;
    float angleLimitMin;
    bool inverseDirection;
    float reduction;
    State state = STOP; // getState
    virtual void SetAngle(float) = 0;
    virtual float getCurrent()= 0;
    virtual float getTorque() = 0;
    virtual void SetAngleWithVelocityLimit(float _angle, float _vel) = 0;
    virtual void Reboot() = 0;
    virtual void SetAcceleration(float _val) = 0;
    virtual void SetEnable(bool _enable) = 0;
    virtual void ApplyPositionAsHome() = 0;
    virtual void UpdateAngle() = 0;
    virtual float getVel() = 0;
};

#endif // DUMMY_CORE_FW_CTRL_STEP_HPP
