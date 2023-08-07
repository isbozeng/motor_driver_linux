#include <stdio.h>
#include <iostream>
//#include "controlcan.h"
#include <ctime>
#include <cstdlib>
#include "unistd.h"
//#include "Nimotion.hpp"
#include"servocontrol.h"
#include <bitset>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <random>

static const std::string GREEN_BOLD = "\033[1;32m";
static const std::string RED_BOLD = "\033[1;31m";
static const std::string RESET_FORMAT = "\033[0m";

int main(int argc, char *argv[])
{
	std::cout << GREEN_BOLD << ">>this is motor test !" << RESET_FORMAT << std::endl;
    if (argc != 5) 
    {
        std::cout << GREEN_BOLD << "输入错误！依次输入 id[0,1,2], position[-180,180.0], vel[0-48.0] repeat count[0-...]" << RESET_FORMAT << std::endl;
        std::cout << GREEN_BOLD << "例子: ./jointTest 1 220.2 288.0 10" << RESET_FORMAT << std::endl;
        std::cout << GREEN_BOLD << "电机id 1,目标位置220.2度,速度288.0度每秒,重复次数10次" << RESET_FORMAT << std::endl;
        return 0;
    }
    std::cout << GREEN_BOLD <<" "
     << std::string(argv[0]) << " "
     << std::string(argv[1]) << " "
     << std::string(argv[2]) << " "
     << std::string(argv[3]) << " "
     << std::string(argv[4]) <<RESET_FORMAT << std::endl;
    int32_t id = std::stoi(std::string(argv[1]));
    float pos = std::stof(std::string(argv[2]));
    float vel = std::stof(std::string(argv[3]));
    int32_t reptcnt = std::stoi(std::string(argv[4]));
    int32_t curCnt = 0;
    uint32_t step = 0;
    std::random_device rd;
    std::mt19937 gen(rd()); // 使用真随机种子初始化 Mersenne Twister 生成器
    std::uniform_int_distribution<int> dis(-180, 180); // 生成 -180 到 180 之间的均匀分布整数
    float next_pos = dis(gen);
    // auto last_time = std::chrono::steady_clock::now();
    // auto now_time = std::chrono::steady_clock::now();
    if (id != 1 && id != 2 && id != 3 && id != 4) 
    {
        std::cout << GREEN_BOLD << "电机ID不存在! " << RESET_FORMAT << std::endl;
        return 0;
    }
    ServoMotion motor(id, false, 1, -180.0, 180.0);


	while (motor.state != ServoMotion::FINISH)
	{
		motor.UpdateAngle();
		usleep(100000);
	};

    motor.SetAngleWithVelocityLimit(pos, vel);	
	while (true)
	{   
		motor.UpdateAngle();
		//motor.UpdateMoveState();//更新电机状态，查看是否执行完毕
        if (motor.state == 1)//ServoMotion::FINISH)
	    {
            if (curCnt == reptcnt)
            {
                return 0;
            }
            if (fabs(motor.angle - pos) < 1.0)
            {           
                curCnt++;
                motor.SetAngleWithVelocityLimit(next_pos, vel);	
                std::cout << GREEN_BOLD << "当前位置:" << motor.angle <<
                " 目标位置:"<< pos <<
                " 偏移位置:"<< next_pos <<
                " 目标位置到达次数:"<< curCnt <<RESET_FORMAT << std::endl;                    
                next_pos = dis(gen);
            }
            else
            {
                motor.SetAngleWithVelocityLimit(pos, vel);	
            }
	    };
        motor.UpdateAngle();
		usleep(100000);
	}
    return 0;
}
