/**
 * \file	serial_sender.h
 * \author	Thomas Fuhrmann, Gaëtan Blond <tomesman@gmail.com>
 * \brief   Functions to send data accross the serial communication line
 * \date	06/12/2016, 14/02/2019
 * \copyright Copyright (c) 2019 UTCoupe All rights reserved.
 */

#ifndef STM32_SENDER_H
#define STM32_SENDER_H

#include "serial.h"

#include <cstdarg>
#include <queue>
#include <string>

class SerialSender
{
public:
    enum SerialSendEnum { SERIAL_ERROR = 0, SERIAL_INFO, SERIAL_DEBUG };
    
    SerialSender(Serial* serialPtr);
    ~SerialSender() = default;
    
    void serialSend(SerialSendEnum level, std::string data);
    void serialSend(SerialSendEnum level, const char* str, ...);
    
    void serialSendTask();
        
private:
    Serial* _serialInterfacePtr;
    std::queue<std::string> _dataToSend;
    
    std::string charArrayToString(const char * str, uint8_t size);
};




#endif // STM32_SERIAL_H
