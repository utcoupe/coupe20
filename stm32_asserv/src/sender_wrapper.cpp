//
// Created by tfuhrman on 29/01/17.
//

#include "sender_wrapper.h"
#include "serial_sender.h"

#include <cstdarg>
#include <string>

using namespace std;

//static void SerialSendWrapVariadic(SerialSendEnum level, const char* data, va_list args);

static void SerialSendWrapVariadic(SerialSendEnum level, const char* data, va_list args) {
    g_serialSender.serialSend(level, data, args);
}

void SerialSendWrap(SerialSendEnum level, const string& data) {
    g_serialSender.serialSend(level, data);
}

void SerialSendWrapVar(SerialSendEnum level, const char* data, ...) {
    va_list args;
    va_start(args, data);
    SerialSendWrapVariadic(level, data, args);
    va_end(args);
}
