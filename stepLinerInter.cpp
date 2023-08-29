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


double linearInterpolation(double start, double end, double t0, double t1) 
{
    if (t0 <= 0) {
        return start;
    } else if (t0 >= t1) {
        return end;
    }

    // 计算线性插值结果
    double interpolatedValue = start + (end - start) * (t0 / t1);

    return interpolatedValue;
}

int main(int agrc, char *argv[])
{
    float tar[5][2] = {
        {30.0, 2000.0}, //m1
        {40.0, 2000.0}, //m2
        {30.0, 2000.0}, //m3
        {270.0, 5000.0}, //s1 guanjie2
        {-270.0, 5000.0}  //s2 guanjie3
    };
    Nimotion m4260(1, false, 1, -360.0, 360.0);
	Nimotion m4248(2, true, 1, -360.0, 360.0);
	// ServoMotion m1(1, false, 1, -180.0, 180.0);
	// ServoMotion m2(2, false, 1, -180.0, 180.0);
	// ServoMotion m3(3, false, 1, -180.0, 180.0);    
	while (m4260.state != Nimotion::FINISH 
	|| m4248.state != Nimotion::FINISH)
	// || m1.state != Nimotion::FINISH
	// || m2.state != Nimotion::FINISH
	// || m3.state != Nimotion::FINISH)
	{
		m4260.UpdateAngle();
		m4248.UpdateAngle();
		// m1.UpdateAngle();
		// m2.UpdateAngle();
		// m3.UpdateAngle();
		// std::cout<<"----"<<std::endl;
		// std::cout<<(int)m1.state<<std::endl;	
		// std::cout<<(int)m2.state<<std::endl;
		// std::cout<<(int)m3.state<<std::endl;
		// std::cout<<(int)m4260.state<<std::endl;
		// std::cout<<(int)m4248.state<<std::endl;		
		usleep(100000);
	}

    // m1.SetAngle(0);
    // m2.SetAngle(0);
    // m3.SetAngle(0);
    // m4260.SetAngleWithVelocityLimit(0, 50);
    // m4248.SetAngleWithVelocityLimit(0, 50);
    sleep(1);
	while (m4260.state != Nimotion::FINISH 
	|| m4248.state != Nimotion::FINISH)
	// || m1.state != Nimotion::FINISH
	// || m2.state != Nimotion::FINISH
	// || m3.state != Nimotion::FINISH)
	{
		m4260.UpdateAngle();
		m4248.UpdateAngle();
		// m1.UpdateAngle();
		// m2.UpdateAngle();
		// m3.UpdateAngle();
		// std::cout<<"----"<<std::endl;
		// std::cout<<(int)m1.state<<std::endl;	
		// std::cout<<(int)m2.state<<std::endl;
		// std::cout<<(int)m3.state<<std::endl;
		// std::cout<<(int)m4260.state<<std::endl;
		// std::cout<<(int)m4248.state<<std::endl;		
		usleep(100000);
	}
    // return 0;

    auto curTime = std::chrono::system_clock::now();
    auto curTimeMs = std::chrono::time_point_cast<std::chrono::milliseconds>(curTime);  
    uint64_t curMs = curTimeMs.time_since_epoch().count(); 

    uint32_t duration = 0;
    float s1Cmd = 0;
    float s2Cmd = 0;
    float m1Cmd = 0;
    float m2Cmd = 0;
    float m3Cmd = 0;

	// motor.SetEnable(false);
	// motor.ApplyPositionAsHome();
	auto update_time = std::chrono::steady_clock::now();
	auto last_time = update_time;
	auto now_time = update_time;
    auto elapsedTime = now_time - last_time;

    std::cout<<"**********************"<<std::endl;
    std::cout<<"*angle ms:"<<curMs<</*",m1:"<<m1.angle<<",m2:"<<m2.angle<<",m3:"<<m3.angle
        <<*/",s1:"<<m4260.angle<<",s2:"<<m4248.angle<<std::endl;
    std::cout<<"*vel ms:"<<curMs/*<<",velM1:"<<m1.getVel()<<",velM2:"<<m2.getVel()<<",velM3:"<<m3.getVel()*/
        <<",velS1:"<<m4260.getVel()<<",velS2:"<<m4248.getVel()<<std::endl;  
    std::cout<<"**********************"<<std::endl;
    // float m1Start = m1.angle;
    // float m2Start = m2.angle;
    // float m3Start = m3.angle; 
    float s1Start = m4260.angle;
    float s2Start = m4248.angle;          
	while (true)
	{


        // m1Cmd = linearInterpolation(m1Start, m1Start + tar[0][0], duration, tar[0][1]);
        // m2Cmd = linearInterpolation(m2Start, m2Start + tar[1][0], duration, tar[1][1]);
        // m3Cmd = linearInterpolation(m3Start, m3Start + tar[2][0], duration, tar[2][1]);
        s1Cmd = linearInterpolation(s1Start, s1Start + tar[3][0], duration, tar[3][1]);
        s2Cmd = linearInterpolation(s2Start, s2Start + tar[4][0], duration, tar[4][1]);

        // m1.SetAngle(m1Cmd);
        // m2.SetAngle(m2Cmd);
        // m3.SetAngle(m3Cmd);
        m4260.SetAngle(s1Cmd);
        m4248.SetAngle(s2Cmd);            
		curTime = std::chrono::system_clock::now();
    	curTimeMs = std::chrono::time_point_cast<std::chrono::milliseconds>(curTime);
    	curMs = curTimeMs.time_since_epoch().count();
		std::cout<<"Cmdms:"<<curMs/*<<",m1Cmd:"<<m1Cmd<<",m2Cmd:"<<m2Cmd<<",m3Cmd:"<<m3Cmd*/
                <<",s1Cmd:"<<s1Cmd<<",s2Cmd:"<<s2Cmd<<",duration:"<<duration<<std::endl;

		auto updateElapsed = now_time - update_time;
		int32_t delt = std::chrono::duration_cast<std::chrono::milliseconds>(updateElapsed).count();
		if (delt > 40)//位置更新周期
        {
            m4260.UpdateAngle();
            m4248.UpdateAngle();
            // m1.UpdateAngle();
            // m2.UpdateAngle();
            // m3.UpdateAngle();   
            curTime = std::chrono::system_clock::now();
            curTimeMs = std::chrono::time_point_cast<std::chrono::milliseconds>(curTime);
            curMs = curTimeMs.time_since_epoch().count();
            std::cout<<"Anglems:"<<curMs/*<<",m1:"<<m1.angle<<",m2:"<<m2.angle<<",m3:"<<m3.angle*/
                    <<",s1:"<<m4260.angle<<",s2:"<<m4248.angle<<" ,duration:"<<duration<<std::endl;
            std::cout<<"Velms:"<<curMs/*<<",velM1:"<<m1.getVel()<<",velM2:"<<m2.getVel()<<",velM3:"<<m3.getVel()*/
                    <<",velS1:"<<m4260.getVel()<<",velS2:"<<m4248.getVel()<<" ,duration:"<<duration<<std::endl;  
            update_time = std::chrono::steady_clock::now();              
        }

		usleep(5000);
        now_time = std::chrono::steady_clock::now();
		elapsedTime = now_time - last_time;
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count();        
	}
}


// # 添加第一个折线图轨迹
// fig.add_trace(go.Scatter(x=curDuration2, y=velM1, mode='lines+markers', name='velM1'), row=1, col=3)

// # 添加第二个折线图轨迹
// fig.add_trace(go.Scatter(x=curDuration1, y=velM2, mode='lines+markers', name='velM2'), row=1, col=3)

// fig.add_trace(go.Scatter(x=curDuration1, y=velM3, mode='lines+markers', name='velM3'), row=1, col=3)

// fig.add_trace(go.Scatter(x=curDuration1, y=velS1, mode='lines+markers', name='velS1'), row=1, col=3)

// fig.add_trace(go.Scatter(x=curDuration1, y=velS2, mode='lines+markers', name='velS2'), row=1, col=3)