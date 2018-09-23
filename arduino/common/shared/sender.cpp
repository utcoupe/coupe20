/**
 * \file	sender.c
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief   Functions to send data accross the serial communication line
 * \date	06/12/2016
 * \copyright Copyright (c) 2016 UTCoupe All rights reserved.
 */

#include "sender.h"
#include "parameters.h"

#include <QueueList.h>
#include <Arduino.h>

/* Initialize static members */
QueueList<String> SerialSender::dataToSend;

SerialSender::SerialSender() {
}

void SerialSender::SerialSend(SerialSendEnum level, String data) {
    if (level <= DEBUG_LEVEL && data != "") {
        dataToSend.push(data);
    }
}

void SerialSender::SerialSend(SerialSendEnum level, const char* str, ...) {
    uint8_t i, j, count = 0;
    String serialData, tmpString = "";
    if (level <= DEBUG_LEVEL) {
        va_list argv;
        va_start(argv, str);
        for (i = 0, j = 0; str[i] != '\0'; i++) {
            if (str[i] == '%') {
                count++;

                tmpString = CharArrayToString((str + j), i - j);
                serialData.concat(tmpString);

                switch (str[++i]) {
                    case 'i':
                    case 'd':
                        tmpString = String(va_arg(argv, int));
                        break;
                    case 'l':
                        tmpString = String(va_arg(argv, long));
                        break;
                    case 'f': //tmpString = String(va_arg(argv, float), 4);
                        break;
                    case 'c':
                        tmpString = String((char) va_arg(argv, int));
                        break;
                    case 's':
                        tmpString = String(va_arg(argv, char *));
                        break;
//                    case '%':
//                        Serial.print("%");
//                        break;
                    default:;
                }
                serialData.concat(tmpString);
                j = i + 1;
            }
        }
        va_end(argv);

        if (i > j) {
            tmpString = CharArrayToString((str + j), i - j);
            serialData.concat(tmpString);
        }

        dataToSend.push(serialData);
    }
}

void SerialSender::SerialSendTask() {
    String dataToPrint;
    while (!dataToSend.isEmpty()) {
        dataToPrint = dataToSend.pop();
        Serial.println(dataToPrint);
        Serial.flush();
    }
}

String SerialSender::CharArrayToString(const char * str, uint8_t size) {
    String returnedString = "";
    //todo size as define
    if ((str != nullptr) && (size > 0) && (size < 71)) {
        static char tmpBuffer[70];
        memcpy(tmpBuffer, str, size);
        tmpBuffer[size] = '\0';
        returnedString = String(tmpBuffer);
    }
    return returnedString;
}
