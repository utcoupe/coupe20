//
// Created by tfuhrman on 29/01/17.
//

#ifndef ARDUINO_SENDER_WRAPPER_H
#define ARDUINO_SENDER_WRAPPER_H

#ifndef SENDER_ENUM
#define SENDER_ENUM
typedef enum
{
    SERIAL_ERROR = 0,
    SERIAL_INFO,
    SERIAL_DEBUG
} SerialSendEnum;
#endif

#ifdef __cplusplus
#include "string.hpp"
extern "C" void SerialSendWrap(SerialSendEnum level, const String& data);
extern "C" void SerialSendWrapVar(SerialSendEnum level, const char* data, ...);
extern "C" void SerialSendGoalReached(int goal_id);
#else
//void SerialSendWrap(SerialSendEnum level, String data);
void SerialSendWrapVar(SerialSendEnum level, const char* data, ...);
void SerialSendGoalReached(int goal_id);
#endif

#endif //ARDUINO_SENDER_WRAPPER_H
