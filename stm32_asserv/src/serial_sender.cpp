/**
 * \file	serial_sender.h
 * \author	Thomas Fuhrmann, Gaëtan Blond <tomesman@gmail.com>
 * \brief   Functions to send data accross the serial communication line
 * \date	06/12/2016, 14/02/2019
 * \copyright Copyright (c) 2019 UTCoupe All rights reserved.
 */

#include "serial_sender.h"

#include "_shared_parameters.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern Serial g_serial;

const unsigned SS_BUFFER_SIZE = 70;

SerialSender::SerialSender(Serial* serial):
    _serialInterfacePtr(serial) {
}

void SerialSender::serialSend(SerialSendEnum level, String data) {
    if (level <= DEBUG_LEVEL && data != "") {
        _dataToSend.push(data);
    }
}

void SerialSender::serialSend(SerialSendEnum level, const char* str, ...) {
    static char tmpBuff[SS_BUFFER_SIZE]; // TODO Maybe dynamic allocation ?
        
    if (level <= DEBUG_LEVEL) {
        va_list argv;
        va_start(argv, str);
        
        auto result = vsnprintf(tmpBuff, SS_BUFFER_SIZE - 1, str, argv);
        va_end(argv);
        if (result >= 0) {
            _dataToSend.push(tmpBuff);
        }
        // TODO error handling
    }
}

void SerialSender::serialSendTask() {
    while (!_dataToSend.isEmpty()) {
        _serialInterfacePtr->println(_dataToSend.pop());
    }
}

String SerialSender::charArrayToString(const char * str, uint8_t size) {
    String returnedString = "";
    if ((str != nullptr) && (size > 0) && (size < SS_BUFFER_SIZE + 1)) {
        static char tmpBuffer[SS_BUFFER_SIZE];
        memcpy(tmpBuffer, str, size);
        tmpBuffer[size] = '\0';
        returnedString = tmpBuffer;
    }
    return returnedString;
}
