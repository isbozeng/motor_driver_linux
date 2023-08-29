#ifndef NI_MOTION_HPP
#define NI_MOTION_HPP
#include "UsbCanBus.hpp"
#include "ctrl_step.hpp"
#include <queue>
#include <chrono>
#include <mutex>
// 0x06 0x07 可操作状态 电机未使能
// 0x07失能
// 0x0f使能

// bit2 0失能
// bit2 1使能

// 使能，清零，单位，运动方向

// 10000 = 60,  rpm/s
// 20:1

class Nimotion : public CtrlStepMotor
{
public:
    Nimotion(uint8_t _id, bool _inverse = false, float _reduction = 1,
             float _angleLimitMin = -180.0, float _angleLimitMax = 180.0, bool ipmode = true);
    virtual ~Nimotion() = default;
    virtual void SetAngleWithVelocityLimit(float _angle, float _vel) override;
    virtual void Reboot() override;
    virtual void SetAcceleration(float _val) override;
    virtual void SetEnable(bool _enable) override;
    virtual void ApplyPositionAsHome() override;
    virtual void UpdateAngle() override;
    virtual void SetAngle(float) override;
    virtual float getCurrent() override;
    virtual float getTorque() override;  
    virtual float getVel() override;  
    // int32_t getVel() { return cur_vel; }
    CanBase *can_bus_ = nullptr;
    uint32_t timeout = 3; // s

private:
#pragma pack(1)
    typedef struct
    {
        uint16_t switchOn : 1;
        uint16_t enableVol : 1;
        uint16_t quickStop : 1;
        uint16_t enableOperation : 1;
        uint16_t operationMode : 3;
        uint16_t faultReset : 1;
        uint16_t halt : 1;
        uint16_t : 7;
    } controlword_t;
#pragma pack()
#pragma pack(1)
    typedef struct
    {
        /* uint16_t :2;
        uint16_t operationMode:2;
        uint16_t internalLimitActive:1;
        uint16_t targetReached:1;
        uint16_t remote:1;
        uint16_t :1;
        uint16_t warning:1;
        uint16_t disswitchOn:1;
        uint16_t quickStop:1;
        uint16_t enableVol:1;
        uint16_t halt:1;
        uint16_t enableOperation:1;
        uint16_t switchOn:1;
        uint16_t ready:1;*/

        uint16_t ready : 1;
        uint16_t switchOn : 1;
        uint16_t enableOperation : 1;
        uint16_t halt : 1;
        uint16_t enableVol : 1;
        uint16_t quickStop : 1;
        uint16_t disswitchOn : 1;
        uint16_t warning : 1;
        uint16_t : 1;
        uint16_t remote : 1;
        uint16_t targetReached : 1;
        uint16_t internalLimitActive : 1;
        uint16_t operationMode : 2;
        uint16_t : 2;

    } statusword_t;
#pragma pack()
    enum NI_MOTION_STATUS_t
    {
        START_NODE,
        READY,
        SWITCH_ON,
        HOMING,
        CLEAR_ERROR,
        SET_MODE,
        SET_ENABLE,
        SET_CMD,
        MOTION_ENABLE,
        SET_POS_CMD,
        SET_VEL_CMD,
        MOTION,
        DONE
    };
    enum NI_MOTION_MODE_t
    {
        NULL_MODE = 0,
        POS_MODE,
        VEL_MODE,
        IP_MODE = 0x07,
    };
    enum SDO_ACK_t
    {
        NULL_ACK,
        POS_ACK,
        VEL_ACK,
    };
    enum CLEAR_POS_ACK_t
    {
        NULL_CLEAR,
        HIGHT_CLEAR,
        LOW_CLEAR
    };
    struct posCtlCmd
    {
        int32_t pos;
        int32_t vel;
        posCtlCmd(int32_t p, int32_t v) : pos(p), vel(v) {}
    };
    int16_t sixForce;
    int16_t current;
    uint16_t error_code = 0;
    int32_t pos_cmd = 0;
    int32_t vel_cmd = 0;
    int32_t cur_vel = 0;
    int32_t cur_pos = 0;
    uint32_t det_pos = 100;
    bool isOnline = false;
    bool isHome = false;
    bool isEnable = true;
    bool isReset = false;
    bool isNewCmd = false;
    bool isIPmode = false;
    bool isIPenable = false;
    NI_MOTION_STATUS_t nimotion_state = START_NODE;
    NI_MOTION_MODE_t nimotion_mode = NULL_MODE;
    SDO_ACK_t sdo_ack = NULL_ACK;
    CLEAR_POS_ACK_t clear_ack = NULL_CLEAR;
    statusword_t statusword;
    controlword_t controlword;
    std::mutex mtx;
    std::chrono::time_point<std::chrono::steady_clock> last_time;
    std::chrono::time_point<std::chrono::steady_clock> now_time;
    // std::queue<posCtlCmd> CmdQueue;

    void switchState();
    void recMsgCallback(CanBase::CanRxMsg msg);
    void clearError();
};

#endif