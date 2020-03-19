//
// Created by tfuhrman on 09/05/17.
//
#include "protocol.hpp"
#include "serial_sender.h"

//get from old protocol file
// extern "C" {
#include "compat.h"
#include "robotstate.h"
#include "control.h"
#include "shared_asserv/control.h"
#include "shared_asserv/goals.h"
#include "shared_asserv/parameters.h"
// }

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

void autoSendStatus() {
    g_serialSender.serialSend(
        SERIAL_INFO,
        "%c;%i;%i;%i;%i;%i;%i;%i;%i;%i;",
        AUTO_SEND,
        control.last_finished_id,
        static_cast<int>(current_pos.x),
        static_cast<int>(current_pos.y),
        static_cast<int>(current_pos.angle*static_cast<float>(FLOAT_PRECISION)),
        control.speeds.pwm_left,
        control.speeds.pwm_right,
        static_cast<int>(control.speeds.linear_speed),
        static_cast<int>(wheels_spd.left),
        static_cast<int>(wheels_spd.right)
    );

// #ifdef SERIAL_DEBUG
//     g_serial.print("x ");
//     g_serial.print((int16_t)current_pos.x);
//     g_serial.print("    y ");
//     g_serial.print((int16_t)current_pos.y);
//     g_serial.print("    ang ");
//     g_serial.print((int16_t)(current_pos.angle*FLOAT_PRECISION));
//     g_serial.print("    pwm l ");
//     g_serial.print((int16_t)control.speeds.pwm_left);
//     g_serial.print("    pwm r ");
//     g_serial.print((int16_t)control.speeds.pwm_right);
//     g_serial.print("    l spd ");
//     g_serial.print((int16_t)control.speeds.linear_speed);
//     g_serial.print("    l wheel ");
//     g_serial.print((int16_t)wheels_spd.left);
//     g_serial.print("    r wheel ");
//     g_serial.print((int16_t)wheels_spd.right);
//     g_serial.print("\n");
// #endif

}


void ProtocolAutoSendStatus() 
{
    autoSendStatus();
}

uint8_t getLog10(const uint16_t number) {
    if(number>=10000) return 5;
    if(number>=1000) return 4;
    if(number>=100) return 3;
    if(number>=10) return 2;
    return 1;
}

void parseAndExecuteOrder(const String& order) {
    static char receivedOrder[25];
    char* receivedOrderPtr = receivedOrder;
    strcpy(receivedOrder, order.c_str());
    char orderChar = receivedOrder[ORDER_INDEX];
    uint16_t order_id = atoi(&receivedOrder[ID_INDEX]); // TODO maybe strtol ?
    uint8_t numberDigits = getLog10(order_id);
    // Move to the first parameter of the order
    receivedOrderPtr +=  ID_INDEX + numberDigits + (uint8_t)1;
    switch (orderChar) {
        case START:
        {
            start();
            // Ack that STM32 has started
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            g_serialSender.serialSend(
                SERIAL_DEBUG,
                "STM32 %s has started (%d)",
                STM32_ID,
                order_id
            );
            break;
        }
        case HALT:
        {
            halt();
            // Ack that STM32 has stopped
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            g_serialSender.serialSend(
                SERIAL_DEBUG,
                "STM32 %s has stopped (%d)",
                STM32_ID,
                order_id
            );
            break;
        }
        case PINGPING:
            //todo add LED on arduino
            /*digitalWrite(LED_DEBUG, HIGH);
            delay(1);
            digitalWrite(LED_DEBUG, LOW);*/
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        case GET_CODER:
            g_serialSender.serialSend(
                SERIAL_INFO,
                "%d;%d;%d;",
                order_id,
                static_cast<long>(get_left_encoder()),
                static_cast<long>(get_right_encoder())
            );
            break;

        case GOTO:
            parseGOTO(receivedOrderPtr, order_id);
            break;

        case GOTOA:
            parseGOTOA(receivedOrderPtr, order_id);
            break;

        case ROT:
            parseROT(receivedOrderPtr, order_id);
            break;

        case ROTNOMODULO:
            parseROTNMODULO(receivedOrderPtr, order_id);
            break;
            
        case PWM:
            parsePWM(receivedOrderPtr, order_id);
            break;

        case SPD:
            parseSPD(receivedOrderPtr, order_id);
            break;
        
        case PIDALL:
            parsePIDALL(receivedOrderPtr);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        
        case PIDRIGHT:
            parsePIDRIGHT(receivedOrderPtr);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;

        case PIDLEFT:
            parsePIDLEFT(receivedOrderPtr);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        
        case KILLG:
            FifoNextGoal();
            ControlPrepareNewGoal();
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;

        case CLEANG:
            FifoClearGoals();
            ControlPrepareNewGoal();
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;

        case RESET_ID:
            resetID();
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;

        case SET_POS:
        {
            parseSETPOS(receivedOrderPtr);
            int x, y, a_int, mode;
            sscanf(receivedOrderPtr, "%i;%i;%i;%i", &x, &y, &a_int, &mode);
            g_serialSender.serialSend(SERIAL_INFO, "%d;%i;", order_id, a_int);
            break;
        }
        case GET_POS:
        {
            int x, y, a_int;
            x = static_cast<int>(round(current_pos.x));
            y = static_cast<int>(round(current_pos.y));
            a_int = static_cast<int>(current_pos.angle * static_cast<float>(FLOAT_PRECISION));
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;%d;", order_id, x, y, a_int);
            break;
        }
        case GET_SPD:
        {
            int l, r;
            l = static_cast<int>(wheels_spd.left);
            r = static_cast<int>(wheels_spd.right);
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;", order_id, l, r);
            break;
        }
        case GET_TARGET_SPD:
        {
            int left_spd, right_spd;
            left_spd = static_cast<int>(control.speeds.linear_speed - control.speeds.angular_speed);
            right_spd = static_cast<int>(control.speeds.linear_speed + control.speeds.angular_speed);
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;", order_id, left_spd, right_spd);
            break;
        }
        case GET_POS_ID:
        {
            int x, y, a_int;
            x = static_cast<int>(round(current_pos.x));
            y = static_cast<int>(round(current_pos.y));
            a_int = static_cast<int>(current_pos.angle * static_cast<float>(FLOAT_PRECISION));
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;%d;", order_id, x, y, a_int, control.last_finished_id);
            break;
        }
        case SPDMAX:
        {
            parseSPDMAX(receivedOrderPtr);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        }
        case ACCMAX:
        {
            parseACCMAX(receivedOrderPtr);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        }
        case GET_LAST_ID:
            g_serialSender.serialSend(SERIAL_INFO, "%d", control.last_finished_id);
            break;
        case PAUSE:
            ControlSetStop(PAUSE_BIT);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        case RESUME:
            ControlUnsetStop(PAUSE_BIT);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        case WHOAMI:
            g_serialSender.serialSend(SERIAL_INFO, "%d;%s;", order_id, STM32_ID);
            break;
        case SETEMERGENCYSTOP:
        {
            parseSETEMERGENCYSTOP(receivedOrderPtr);
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        }
        default:
            g_serialSender.serialSend(SERIAL_INFO, "Order %c is wrong !", orderChar);
    }
}

