#include <stdio.h>
#include <iostream>
#include "controlcan.h"
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "Nimotion.hpp"
#include "servocontrol.h"
#include <bitset>
#include <unistd.h>
#include <signal.h>
#include <time.h>

static const std::string GREEN_BOLD = "\033[1;32m";
static const std::string RED_BOLD = "\033[1;31m";
static const std::string RESET_FORMAT = "\033[0m";

int main(int agrc, char *argv[])
{
	std::cout << GREEN_BOLD << ">>this is motor test !" << RESET_FORMAT << std::endl;
	int32_t step = 0;
	bool isMax = false;
	Nimotion m4260(1, false, 20, -180.0, 180.0);
	Nimotion m4248(2, false, 20, -180.0, 180.0);
	ServoMotion m1(1, false, 1, -180.0, 180.0);
	ServoMotion m2(2, false, 1, -180.0, 180.0);
	ServoMotion m3(3, false, 1, -180.0, 180.0);
	while (m4260.state != Nimotion::FINISH 
	|| m4248.state != Nimotion::FINISH
	|| m1.state != Nimotion::FINISH
	|| m2.state != Nimotion::FINISH
	|| m3.state != Nimotion::FINISH)
	{
		m4260.UpdateAngle();
		m4248.UpdateAngle();
		m1.UpdateAngle();
		m2.UpdateAngle();
		m3.UpdateAngle();		
		usleep(100000);
	};
	auto last_time = std::chrono::steady_clock::now();
	auto now_time = std::chrono::steady_clock::now();
	// motor.SetEnable(false);
	// motor.ApplyPositionAsHome();
	while (true)
	{
		auto elapsedTime = now_time - last_time;
		int32_t delt = std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count();
		if (delt > 500)
		{
			m4260.SetAngle(step);
			m4248.SetAngle(step);
			m1.SetAngle(step);
			m2.SetAngle(step);
			m3.SetAngle(step);
			step += 1;
			if(step == 90)
			{
				isMax = true;
			}
			if (isMax)
			{
				step -= 1;
			}
			if(step == 0)
			{
				isMax = false;
			}
			last_time = std::chrono::steady_clock::now();
		}
		m4260.UpdateAngle();
		m4248.UpdateAngle();
		m1.UpdateAngle();
		m2.UpdateAngle();
		m3.UpdateAngle();
		now_time = std::chrono::steady_clock::now();
		usleep(100000);
	}
}
