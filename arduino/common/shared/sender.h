/**
 * \file	sender.h
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief   Functions to send data accross the serial communication line
 * \date	06/12/2016
 * \copyright Copyright (c) 2016 UTCoupe All rights reserved.
 */

#ifndef ARDUINO_SENDER_H
#define ARDUINO_SENDER_H

#include <stdarg.h>
#include <stdint.h>

#include <QueueList.h>

#ifndef SENDER_ENUM
#define SENDER_ENUM
typedef enum
{
    SERIAL_ERROR = 0,
    SERIAL_INFO,
    SERIAL_DEBUG
} SerialSendEnum;
#endif

//#ifdef __cplusplus
//extern "C" void SerialSendC(SerialSendEnum level, String data);
//#endif

class SerialSender
{
public:
    SerialSender();
    ~SerialSender() {}
    //to be used everywhere
    //todo check why its only working with minimal 1 variadic argument
    static void SerialSend(SerialSendEnum level, const char* data, ...);
    static void SerialSend(SerialSendEnum level, String data);
    //to be used in the task
    static void SerialSendTask();
private:
    static String CharArrayToString(const char * str, uint8_t size);
    static QueueList<String> dataToSend;
};

#endif //ARDUINO_SENDER_H
