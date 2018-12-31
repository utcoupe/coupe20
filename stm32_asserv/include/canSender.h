/**
 * \file	sender.h
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief   Functions to send data accross the serial communication line
 * \date	06/12/2016
 * \copyright Copyright (c) 2016 UTCoupe All rights reserved.
 */

#ifndef CAN_SENDER_H
#define CAN_SENDER_H

#include "QueueList.h"
#include "can.h"
//#include "serial.h"



//#include "stm32f3xx_hal.h"

#ifndef SENDER_ENUM
#define SENDER_ENUM
typedef enum
{
    SERIAL_ERROR = 0,
    SERIAL_INFO,
    SERIAL_DEBUG
} canSendEnum;
#endif

//#ifdef __cplusplus
//extern "C" void canSendC(canSendEnum level, String data);
//#endif
// typedef enum
// {
//     HANDSHAKE   =   0,
//     WHOAMI      =   1,
//     SET_MODE    =   3,
//     SPEED       =   4,
//     GET_CODER   =   5,
//     MANAGEMENT  =   7,
//     GOTOA       =   8,
//     GOTO        =   9,
//     ROT         =   10,
//     ROTNOMODULO =   11,
//     PIDLEFT     =   12,
//     PIDRIGHT    =   13,
//     PIDALL      =   14,
//     PWM         =   15,
//     SET_POS     =   16,
//     SET_PARAM   =   17,
//     CURRENT_POS =   18,
//     CURRENT_PWM =   19,
//     CURRENT_SPD =   20
// }canProtocolEnum;

extern Can g_can;

class CanSender
{
public:
    CanSender();
    ~CanSender() {}
    static void canSend(uint8_t mode, ...);
    static void canSendTask();
private:
    // static String CharArrayToString(const char * str, uint8_t size);
    static QueueList<uint8_t> dataToSend;
};

#endif //CAN_SENDER_H
