#ifndef _SMS_STS_THERAD_H
#define _SMS_STS_THREAD_H

#include"SMS_STS.h"
#include"stdint.h"
#include<thread>
#include <chrono>
#include<mutex>
#include<iostream>

#define MAX_NUM 4  //电机最大个数

class SMS_STS_THREAD: public SMS_STS
{
public:
	
	struct Status {
		//u16 Pos;
		int32_t id;
		float Pos_f;
        float Speed_f;
        int Load;
        int Voltage;
        int Temper;
        int Move;
        int Current;
        u8 T_switch;
	};
	static std::mutex mtx;
	SMS_STS_THREAD();//构造函数中启动线程
	SMS_STS_THREAD(u8 End);
	SMS_STS_THREAD(u8 End, u8 Level);

	float getPos(int id);
	int getMove(int id);
	u8 getT_enable(int id);
	float getVel(int id);
	int ReadEnable(int ID);//读使能

	void setID(int8_t n)
	{
		int i = 0;
		std::lock_guard<std::mutex> mylock(mtx);
		for(i;i<MAX_NUM;i++)
		{
			if(st[i].id == -1)
			break;
		}
		st[i].id = n;
		std::cout<< i <<" id " <<st[i].id<<std::endl;
	}

	static SMS_STS_THREAD *getInstance()
	{
		static SMS_STS_THREAD * instance = new SMS_STS_THREAD();
		return instance;
	}

private:
	std::thread th;
	Status st[MAX_NUM];
	void thread_f();
	
};



#endif