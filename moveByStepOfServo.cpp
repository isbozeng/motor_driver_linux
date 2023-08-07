#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
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
	int16_t step = 0;
	//ServoMotion motor_1(1, false, 20, -180.0, 180.0);
	//ServoMotion motor_2(2, false, 20, -180.0, 180.0);
	ServoMotion motor1(1,false,1,-180,180);
	ServoMotion motor2(2,false,1,-180,180);
	ServoMotion motor3(3,false,1,-180,180);
	ServoMotion motor4(4,false,1,-180,180);

	while (motor1.state != ServoMotion::FINISH || motor2.state != ServoMotion::FINISH || motor3.state != ServoMotion::FINISH) // || motor4.state != ServoMotion::FINISH)
	{
		motor1.UpdateAngle();
		motor2.UpdateAngle();
		motor3.UpdateAngle();
		motor4.UpdateAngle();
		usleep(100000);
	};
	auto last_time = std::chrono::steady_clock::now();
	auto now_time = std::chrono::steady_clock::now();
	// motor.SetEnable(false);
	// motor.ApplyPositionAsHome();
	while (true)
	{
		if (motor1.state == ServoMotion::FINISH)
		{
		}
		// if (motor2.state == ServoMotion::FINISH)
		// {
		// }
		auto elapsedTime = now_time - last_time;
		int32_t delt = std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count();
		if (delt > 500)
		{
			motor1.SetAngleWithVelocityLimit(0.0 + step, 200);
			motor2.SetAngleWithVelocityLimit(0.0 + step, 200);
			motor3.SetAngleWithVelocityLimit(0.0 + step, 200);
			motor4.SetAngleWithVelocityLimit(0.0 + step, 200);
			step += 35;
			step %= 180;
			last_time = std::chrono::steady_clock::now();
		}
		motor1.UpdateAngle();
		motor2.UpdateAngle();
		motor3.UpdateAngle();
		motor4.UpdateAngle();
		now_time = std::chrono::steady_clock::now();
		usleep(100000);
	}
}
