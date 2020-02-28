//
// Created by tfuhrman on 29/01/17.
//

#include "sender_wrapper.h"
#include "serial_sender.h"

#include <stdarg.h>

//static void SerialSendWrapVariadic(SerialSendEnum level, const char* data, va_list args);

static void SerialSendWrapVariadic(SerialSendEnum level, const char* data, va_list args) {
    g_serialSender.serialSend(level, data, args);
}

void SerialSendWrap(SerialSendEnum level, const String& data) {
    g_serialSender.serialSend(level, data);
}

// For some reason this does not work
void SerialSendWrapVar(SerialSendEnum level, const char* data, ...) {
    va_list args;
    va_start(args, data);
    SerialSendWrapVariadic(level, data, args);
    va_end(args);
}

void SerialSendGoalReached(int goal_id) {
    g_serialSender.serialSend(SERIAL_INFO, "%d;", goal_id);
}
