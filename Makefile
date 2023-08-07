all:
	g++ moveByStep.cpp src/Nimotion.cpp src/UsbCanBus.cpp src/CanBase.cpp lib/libcontrolcan.so -Iinclude -lpthread -std=c++11 -o test_elf/moveBystep.elf
moveBystep:
	g++ moveByStep.cpp src/Nimotion.cpp src/UsbCanBus.cpp src/CanBase.cpp lib/libcontrolcan.so -Iinclude -lpthread -std=c++11 -o test_elf/moveBystep.elf
atHome:
	g++ atHome.cpp src/Nimotion.cpp src/UsbCanBus.cpp src/CanBase.cpp lib/libcontrolcan.so -Iinclude -lpthread -std=c++11 -o test_elf/atHome.elf
jointTest:
	g++ jointTest.cpp src/Nimotion.cpp src/UsbCanBus.cpp src/CanBase.cpp lib/libcontrolcan.so -Iinclude -lpthread -std=c++11 -o test_elf/jointTest.elf
servoJointTest:
	g++ jointTestOfServo.cpp  src/servocontrol.cpp src/SMS_STS_THREAD.cpp lib/libSCServo.so -Iinclude  -lpthread -std=c++11 -o test_elf/ServoOfJointTest.elf
servoMoveByStep:
	g++ moveByStepOfServo.cpp src/servocontrol.cpp src/SMS_STS_THREAD.cpp lib/libSCServo.so -Iinclude/ -lpthread -std=c++11 -g -o  test_elf/ServoMoveByStep.elf
clean:
	rm -f *.o hello
