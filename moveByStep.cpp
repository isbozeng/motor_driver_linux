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

static const std::string GREEN_BOLD = "\033[1;32m";
static const std::string RED_BOLD = "\033[1;31m";
static const std::string RESET_FORMAT = "\033[0m";

int main(int agrc, char *argv[])
{
	std::cout << GREEN_BOLD << ">>this is motor test !" << RESET_FORMAT << std::endl;
	int32_t step = 0;
	Nimotion motor_4260(1, false, 20, -180.0, 180.0);
	Nimotion motor_4248(2, false, 20, -180.0, 180.0);

	while (motor_4260.state != Nimotion::FINISH || motor_4248.state != Nimotion::FINISH)
	{
		motor_4260.UpdateAngle();
		motor_4248.UpdateAngle();
		usleep(100000);
	};
	auto last_time = std::chrono::steady_clock::now();
	auto now_time = std::chrono::steady_clock::now();
	// motor.SetEnable(false);
	// motor.ApplyPositionAsHome();
	while (true)
	{
		if (motor_4260.state == Nimotion::FINISH)
		{
		}
		if (motor_4248.state == Nimotion::FINISH)
		{
		}
		auto elapsedTime = now_time - last_time;
		int32_t delt = std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count();
		if (delt > 500)
		{
			motor_4260.SetAngleWithVelocityLimit(10.0 + step, 100);
			motor_4248.SetAngleWithVelocityLimit(10.0 + step, 100);
			step += 30;
			step %= 360;
			last_time = std::chrono::steady_clock::now();
		}
		motor_4260.UpdateAngle();
		motor_4248.UpdateAngle();
		now_time = std::chrono::steady_clock::now();
		usleep(100000);
	}
}
