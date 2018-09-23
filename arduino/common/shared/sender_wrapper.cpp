//
// Created by tfuhrman on 29/01/17.
//

#include "sender_wrapper.h"
#include "sender.h"
#include <stdarg.h>

//static void SerialSendWrapVariadic(SerialSendEnum level, const char* data, va_list args);

static void SerialSendWrapVariadic(SerialSendEnum level, const char* data, va_list args) {
    SerialSender::SerialSend(level, data, args);
}

void SerialSendWrap(SerialSendEnum level, String data) {
    SerialSender::SerialSend(level, data);
}

void SerialSendWrapVar(SerialSendEnum level, const char* data, ...) {
    va_list args;
    va_start(args, data);
    SerialSendWrapVariadic(level, data, args);
    va_end(args);
}
