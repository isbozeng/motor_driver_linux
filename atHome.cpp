#include <stdio.h>
#include <iostream>
#include "controlcan.h"
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "Nimotion.hpp"
#include <bitset>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>
static const std::string GREEN_BOLD = "\033[1;32m";
static const std::string RED_BOLD = "\033[1;31m";
static const std::string RESET_FORMAT = "\033[0m";

int main(int argc, char *argv[])
{
    std::cout << GREEN_BOLD << ">>this is motor test !" << RESET_FORMAT << std::endl;
    if (argc != 4)
    {
        std::cout << GREEN_BOLD << "输入错误！" << RESET_FORMAT << std::endl;
        std::cout << GREEN_BOLD << "示例： id[1-2] offsetangle[0-360度] vel[0-900度每秒]" << RESET_FORMAT << std::endl;
        return 0;
    }
    int32_t id = std::stoi(std::string(argv[1]));
    float OffsetAngle = std::stof(std::string(argv[2]));
    float vel = std::stof(std::string(argv[3]));
    if (id != 1 && id != 2)
    {
        std::cout << GREEN_BOLD << "电机ID不存在! " << RESET_FORMAT << std::endl;
        return 0;
    }
    Nimotion motor(id, false, 20, -360.0, 360.0);
    while (motor.state != Nimotion::FINISH)
    {
        motor.UpdateAngle();
        usleep(100000);
    };
    motor.SetAngleWithVelocityLimit(motor.angle + OffsetAngle, vel);
    while (motor.state != Nimotion::FINISH)
    {
        motor.UpdateAngle();
        usleep(100000);
    };  
    std::cout << GREEN_BOLD << "当前位置为:" << motor.angle << RESET_FORMAT << std::endl;  
    motor.SetEnable(false);
    while (motor.state != CtrlStepMotor::STOP)
    {
        std::cout << GREEN_BOLD << "电机下使能中..." << RESET_FORMAT << std::endl;
        motor.UpdateAngle();
        usleep(100000);
    }
    // std::cout << GREEN_BOLD << "移动电机到指定位置，按任意键结束！" << RESET_FORMAT << std::endl;
    // char ch;
    // std::cin.get(ch);
    motor.ApplyPositionAsHome();
    while (fabs(motor.angle) > 0.001)
    {
        motor.UpdateAngle();
        usleep(100000);
    };
    std::cout << GREEN_BOLD << "设置零为成功，当前位置为:" << motor.angle << RESET_FORMAT << std::endl;

    return 0;
}
