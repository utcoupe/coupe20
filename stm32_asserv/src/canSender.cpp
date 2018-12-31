/**
 * \file	sender.c
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \brief   Functions to send data accross the serial communication line
 * \date	06/12/2016
 * \copyright Copyright (c) 2016 UTCoupe All rights reserved.
 */
#include <stdarg.h>
#include <stdint.h>
#include "protocol.h"
#include "canSender.h"
#include "parameters.h"

//#include "QueueList.h"

/* Initialize static members */
QueueList<uint8_t> CanSender::dataToSend;

CanSender::CanSender() {
}

// void CanSender::canSend(canProtocolEnum mode, String data) {
//     if (mode <= DEBUG_LEVEL & data != "") {
//         dataToSend.push(data);
//     }
// }

void CanSender::canSend(uint8_t mode, ...)
{
   
    uint8_t message[] = {0, 0, 0, 0, 0, 0, 0, 0};
    // String serialData, tmpString = "";
        va_list argv;
        va_start(argv, mode);
        switch(mode)        
        {
            case WHOAMI:
                int id;
                id = va_arg(argv, int);
                message[0] = mode;
                message[1] = id;
                break;

            case GET_CODER:
                int coder_l_dist;
                int coder_r_dist;

                coder_l_dist = va_arg(argv, int);
                coder_r_dist = va_arg(argv, int);
                message[0] = mode;
                // TODO: find how to cast in the right type
                //MSB first
                message[1] = coder_l_dist >> 8;
                message[2] = coder_l_dist & 0x00FF;

                message[3] = coder_r_dist >> 8;
                message[4] = coder_r_dist & 0x00FF;

                break;

            case CURRENT_POS:
                int pos_x; 
                int pos_y;
                int angle;
                pos_x = va_arg(argv, int);
                pos_y = va_arg(argv, int);
                angle  = va_arg(argv, int);
                message[0] = mode;
                // TODO: find how to cast in the right type
                //MSB first
                message[1] = pos_x >> 8;
                message[2] = pos_x & 0x00FF;

                message[3] = pos_y >> 8;
                message[4] = pos_y & 0x00FF;

                message[5] = angle >> 8;
                message[6] = angle & 0x00FF;

                break;

            case CURRENT_PWM:
                int l_pwm;
                int r_pwm;

                l_pwm = va_arg(argv, int);
                r_pwm = va_arg(argv, int);
                message[0] = mode;
                // TODO: find how to cast in the right type
                //MSB first
                message[1] = l_pwm >> 8;
                message[2] = l_pwm & 0x00FF;

                message[3] = r_pwm >> 8;
                message[4] = r_pwm & 0x00FF;
                break;

            case CURRENT_SPD:
                int linear_spd;
                int l_wheel_spd;
                int r_wheel_spd;
                linear_spd = va_arg(argv, int);
                l_wheel_spd = va_arg(argv, int);
                r_wheel_spd = va_arg(argv, int);
                message[0] = mode;
                // TODO: find how to cast in the right type
                //MSB first
                message[1] = linear_spd >> 8;
                message[2] = linear_spd & 0x00FF;

                message[3] = l_wheel_spd >> 8;
                message[4] = l_wheel_spd & 0x00FF;

                message[5] = r_wheel_spd >> 8;
                message[6] = r_wheel_spd & 0x00FF;
                break;

            case ORDER_COMPLETED:
            {
                message[0] = ORDER_COMPLETED;
                message[1] = 0;
                break;
            }

            case ROBOT_BLOCKED:
            {
                message[0] = ROBOT_BLOCKED;
            }

            default:
                break;

        }
//         for (i = 0, j = 0; str[i] != '\0'; i++) {
//             if (str[i] == '%') {
//                 count++;

//                 tmpString = CharArrayToString((str + j), i - j);
//                 serialData.concat(tmpString);

//                 switch (str[++i]) {
//                     case 'i':
//                     case 'd':
//                         tmpString = String(va_arg(argv, int));
//                         break;
//                     case 'l':
//                         tmpString = String(va_arg(argv, long));
//                         break;
//                     case 'f': //tmpString = String(va_arg(argv, float), 4);
//                         break;
//                     case 'c':
//                         tmpString = String((char) va_arg(argv, int));
//                         break;
//                     case 's':
//                         tmpString = String(va_arg(argv, char *));
//                         break;
// //                    case '%':
// //                        Can.print("%");
// //                        break;
//                     default:;
//                 }
//                 serialData.concat(tmpString);
//                 j = i + 1;
//             }
    va_end(argv);

    // if (i > j) {
    //     tmpString = CharArrayToString((str + j), i - j);
    //     serialData.concat(tmpString);
    // }
    for (uint8_t i = 0 ; i < 8 ; i++)
    {
        dataToSend.push(message[i]);    
    }
    
    
}

void CanSender::canSendTask() {
    uint8_t canMsg[8];
    while (!dataToSend.isEmpty()) {
    
        for (int8_t i = 0; i < 8 ; i++)
        {
            canMsg[i] = dataToSend.pop();
        }

        g_can.write(canMsg);
        
    }
}