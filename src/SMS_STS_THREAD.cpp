#include "SMS_STS_THREAD.h"
#include "unistd.h"
#include <iostream>
std::mutex SMS_STS_THREAD::mtx;
SMS_STS_THREAD::SMS_STS_THREAD() : th(&SMS_STS_THREAD::thread_f, this) {
  for (int i = 0; i < MAX_NUM; i++) {
    st[i].id = -1; //构造时，-1表示无电机实例
  }
  begin(115200, "/dev/ttyUSB0");
  th.detach();
}

SMS_STS_THREAD::SMS_STS_THREAD(u8 End)
    : SMS_STS(End), th(&SMS_STS_THREAD::thread_f, this) {
  begin(115200, "/dev/ttyUSB0");
  th.detach();
}

SMS_STS_THREAD::SMS_STS_THREAD(u8 End, u8 Level)
    : SMS_STS(End, Level), th(&SMS_STS_THREAD::thread_f, this) {
  const char *str = "/dev/ttyUSB0";
  begin(115200, str);
  th.detach();
}

void SMS_STS_THREAD::thread_f() {
  int Pos = 0;
  int Speed = 0;
  // std::cout<<"Thread state start."<<std::endl;
  while (true) {
    for (int i = 0; i < MAX_NUM; i++) {
      if (st[i].id != -1) {
        std::lock_guard<std::mutex> mylock(mtx);
        Pos = ReadPos(st[i].id);
        st[i].Pos_f = (float)(Pos * 0.088 - 180.0);
        Speed = ReadSpeed(st[i].id);
        st[i].Speed_f = (float)Speed * (4.392 / 50);
        st[i].Load = ReadLoad(st[i].id);
        st[i].Voltage = ReadVoltage(st[i].id);
        st[i].Temper = ReadTemper(st[i].id);
        st[i].Move = ReadMove(st[i].id);
        st[i].Current = ReadCurrent(st[i].id);
        st[i].T_switch = ReadEnable(st[i].id);
      }
    }
    usleep(50 * 1000);
  }
}

    //     //if (FeedBack(st[i].id) != -1) {
    //     //   Pos = ReadPos(-1); //-1表示缓冲区数据，以下相同
    //     //   st[i].Pos_f = (float)(Pos * 0.088 - 180.0);
    //     //   Speed = ReadSpeed(-1);
    //     //   st[i].Speed_f = (float)Speed * (4.392 / 50);
    //     //   st[i].Load = ReadLoad(-1);
    //     //   st[i].Voltage = ReadVoltage(-1);
    //     //   st[i].Temper = ReadTemper(-1);
    //     //   st[i].Move = ReadMove(-1);
    //     //   st[i].Current = ReadCurrent(-1);
    //     //   st[i].T_switch = ReadEnable(st[i].id);
    //     //}

float SMS_STS_THREAD::getPos(int id) {
  int i = 0;
  std::lock_guard<std::mutex> mylock(mtx);
  for (i; i < MAX_NUM; i++) {
    if (st[i].id == id)
      break;
  }
  return st[i].Pos_f;
}

int SMS_STS_THREAD::getMove(int id) {
  int i = 0;
  std::lock_guard<std::mutex> mylock(mtx);
  for (i; i < MAX_NUM; i++) {
    if (st[i].id == id)
      break;
  }
  return st[i].Move;
}

u8 SMS_STS_THREAD::getT_enable(int id) {
  int i = 0;
  std::lock_guard<std::mutex> mylock(mtx);
  for (i; i < MAX_NUM; i++) {
    if (st[i].id == id)
      break;
  }
  return st[i].T_switch;
}

float SMS_STS_THREAD::getVel(int id) {
  int i = 0;
  std::lock_guard<std::mutex> mylock(mtx);
  for (i; i < MAX_NUM; i++) {
    if (st[i].id == id)
      break;
  }
  return st[i].Speed_f;
}

int SMS_STS_THREAD::ReadEnable(int ID)
{
	int _Enable = -1;
	Err = 0;
	_Enable = readByte(ID, SMS_STS_TORQUE_ENABLE);
	if(_Enable==-1){
		Err = 1;
	}
	
	return _Enable; //0,1
}
