
#ifndef USB_CAN_BUS__H_
#define USB_CAN_BUS__H_

#include <stdbool.h>
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <netinet/in.h>
#include <vector>
#include "controlcan.h"
#include "CanBase.hpp"

using namespace std;
// ===================================================================================
class UsbCanBus : public CanBase
{
public:
    enum Bit_Rate
    {
        BR1000k = 0x0014,
        BR800k = 0x0016,
        BR666k = 0x80b6,
        BR500k = 0x001c,
        BR400k = 0x80fa,
        BR250k = 0x011c,
        BR200k = 0x81fa,
        BR125k = 0x031c,
        BR100k = 0x041c,
        BR80k = 0x83ff,
        BR50k = 0x091c,
        BR40k = 0x87ff,
        BR20k = 0x181c,
        BR10k = 0x311c,
    };

public:
    UsbCanBus(uint8_t canind_, Bit_Rate);
    UsbCanBus();
    ~UsbCanBus(void);
    int Open();
    void Close();

    ssize_t Transmit(CanTxMsg *TxMessage, unsigned int len);
    ssize_t Receive(CanRxMsg *RxMessage, uint32_t);

protected:
private:
    VCI_BOARD_INFO pInfo;   // 用来获取设备信息。
    VCI_INIT_CONFIG config; // can口配置
    VCI_ERR_INFO error_code[2];
    VCI_CAN_STATUS status[2];
    vector<CanTxMsg> msg_list;
    uint8_t canind_;
};

class UsbCan2Bus : public CanBase
{
public:
    enum Bit_Rate
    {
        BR1000k = 0x0014,
        BR800k = 0x0016,
        BR666k = 0x80b6,
        BR500k = 0x001c,
        BR400k = 0x80fa,
        BR250k = 0x011c,
        BR200k = 0x81fa,
        BR125k = 0x031c,
        BR100k = 0x041c,
        BR80k = 0x83ff,
        BR50k = 0x091c,
        BR40k = 0x87ff,
        BR20k = 0x181c,
        BR10k = 0x311c,
    };

public:
    UsbCan2Bus(uint8_t canind_, Bit_Rate);
    UsbCan2Bus();
    ~UsbCan2Bus(void);
    int Open();
    void Close();

    ssize_t Transmit(CanTxMsg *TxMessage, unsigned int len);
    ssize_t Receive(CanRxMsg *RxMessage, uint32_t);

protected:
private:
    VCI_BOARD_INFO pInfo;   // 用来获取设备信息。
    VCI_INIT_CONFIG config; // can口配置
    VCI_ERR_INFO error_code[2];
    VCI_CAN_STATUS status[2];
    vector<CanTxMsg> msg_list;
    uint8_t canind_;
};

#endif // UsbCanBus_H