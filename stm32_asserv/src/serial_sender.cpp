/**
 * \file	serial_sender.h
 * \author	Thomas Fuhrmann, Gaëtan Blond <tomesman@gmail.com>
 * \brief   Functions to send data accross the serial communication line
 * \date	06/12/2016, 14/02/2019
 * \copyright Copyright (c) 2019 UTCoupe All rights reserved.
 */

#include "serial_sender.h"

#include "parameters.h"

#include <cstdint>
#include <cstring>

using namespace std;

const unsigned BUFFER_SIZE = 70;

SerialSender::SerialSender(Serial* serial):
    _serialInterfacePtr(serial) {
}

void SerialSender::serialSend(SerialSendEnum level, string data) {
    if (level <= DEBUG_LEVEL && data != "") {
        _dataToSend.push(data);
    }
}

void SerialSender::serialSend(SerialSendEnum level, const char* str, ...) {
    uint8_t i, j, count = 0;
    string serialData, tmpString = "";
    if (level <= DEBUG_LEVEL) {
        va_list argv;
        va_start(argv, str);
        for (i = 0, j = 0; str[i] != '\0'; i++) {
            if (str[i] == '%') {
                count++;

                tmpString = charArrayToString((str + j), i - j);
                serialData += tmpString;

                switch (str[++i]) {
                    case 'i':
                    case 'd':
                        tmpString = std::to_string(va_arg(argv, int));
                        break;
                    case 'l':
                        tmpString = std::to_string(va_arg(argv, long));
                        break;
                    case 'f': //tmpString = String(va_arg(argv, float), 4);
                        break;
                    case 'c':
                        tmpString = static_cast<char>(va_arg(argv, int));
                        break;
                    case 's':
                        tmpString = std::string(va_arg(argv, char *));
                        break;
//                    case '%':
//                        Serial.print("%");
//                        break;
                    default:;
                }
                serialData += tmpString;
                j = i + 1;
            }
        }
        va_end(argv);

        if (i > j) {
            tmpString = charArrayToString((str + j), i - j);
            serialData += tmpString;
        }

        _dataToSend.push(serialData);
    }
}

void SerialSender::serialSendTask() {
    std::string dataToPrint;
    while (!_dataToSend.empty()) {
        dataToPrint = _dataToSend.front();
        _dataToSend.pop();
        _serialInterfacePtr->println(dataToPrint);
    }
}

string SerialSender::charArrayToString(const char * str, uint8_t size) {
    string returnedString = "";
    //todo size as define
    if ((str != nullptr) && (size > 0) && (size < BUFFER_SIZE + 1)) {
        static char tmpBuffer[BUFFER_SIZE];
        memcpy(tmpBuffer, str, size);
        tmpBuffer[size] = '\0';
        returnedString = string(tmpBuffer);
    }
    return returnedString;
}
