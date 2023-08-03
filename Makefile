all:
	g++ moveByStep.cpp Nimotion.cpp UsbCanBus.cpp CanBase.cpp libcontrolcan.so  -lpthread -std=c++11 -o test_elf/moveBystep.elf
moveBystep:
	g++ moveByStep.cpp Nimotion.cpp UsbCanBus.cpp CanBase.cpp libcontrolcan.so  -lpthread -std=c++11 -o test_elf/moveBystep.elf
atHome:
	g++ atHome.cpp Nimotion.cpp UsbCanBus.cpp CanBase.cpp libcontrolcan.so  -lpthread -std=c++11 -o test_elf/atHome.elf
jointTest:
	g++ jointTest.cpp Nimotion.cpp UsbCanBus.cpp CanBase.cpp libcontrolcan.so  -lpthread -std=c++11 -o test_elf/jointTest.elf
clean:
	rm -f *.o hello
