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
#include "goals.h"
#include "parameters.h"
//}

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


uint8_t flagSTM32Connected = 0;

void autoSendStatus() {
    g_serialSender.serialSend(
        SERIAL_INFO,
        "%c;%i;%i;%i;%i;%i;%i;%i;%i;%i;",
        AUTO_SEND,
        control.last_finished_id,
        static_cast<int>(current_pos.x),
        static_cast<int>(current_pos.y),
        static_cast<int>(current_pos.angle*FLOAT_PRECISION),
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

void emergencyStop(const uint8_t enable) {
    // Reset the PID to remove the error sum
    PIDReset(&PID_left);
    PIDReset(&PID_right);
    if (enable == 0) {
        //digitalWrite(LED_DEBUG, LOW);
        ControlUnsetStop(EMERGENCY_BIT);
    } else {
        //digitalWrite(LED_DEBUG, HIGH);
        ControlSetStop(EMERGENCY_BIT);
    }
}

void parseAndExecuteOrder(const String& order) {
    static char receivedOrder[25];
    char* receivedOrderPtr = receivedOrder;
    strcpy(receivedOrder, order.c_str());
    char orderChar = receivedOrder[ORDER_INDEX];
    uint16_t order_id = (uint16_t) atoi(&receivedOrder[ID_INDEX]);
    uint8_t numberDigits = getLog10(order_id);
    // Move to the first parameter of the order
    receivedOrderPtr +=  ID_INDEX + numberDigits + (uint8_t)1;
    switch (orderChar) {
        case START:
        {
            emergencyStop(0);
            // Ack that arduino has started
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            g_serialSender.serialSend(
                SERIAL_DEBUG,
                "STM32 %s has started (%d)",
                STM32_ID,
                order_id
            );
            flagSTM32Connected = true;
            break;
        }
        case HALT:
        {
            emergencyStop(1);
            // Ack that arduino has stopped
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            g_serialSender.serialSend(
                SERIAL_DEBUG,
                "STM32 %s has stopped (%d)",
                STM32_ID,
                order_id
            );
            flagSTM32Connected = 0;
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
        {
            int x, y, direction, slow_go;
            direction = 0;
            sscanf(receivedOrderPtr, "%i;%i;%i;%i", &x, &y, &direction, &slow_go);
            if(slow_go) 
                ControlSetStop(SLOWGO_BIT);    
            else
                ControlUnsetStop(SLOWGO_BIT);    

            goal_data_t goal;
            goal.pos_data = {x, y, direction};
            FifoPushGoal(order_id, TYPE_POS, goal);
            break;
        }
        case GOTOA:
        {
            int x, y, a_int, direction, slow_go;
            float a;
            direction = 0;
            sscanf(receivedOrderPtr, "%i;%i;%i;%i;%i", &x, &y, &a_int, &direction, &slow_go);
            if(slow_go) 
                ControlSetStop(SLOWGO_BIT);    
            else
                ControlUnsetStop(SLOWGO_BIT);   

            a = a_int / (float)FLOAT_PRECISION;
            goal_data_t goal;
            goal.pos_data = {x, y, direction};
            FifoPushGoal(order_id, TYPE_POS, goal);
            goal.ang_data = {a, 1};
            FifoPushGoal(order_id, TYPE_ANG, goal);
            break;
        }
        case ROT:
        {
            int a_int;
            float a;
            sscanf(receivedOrderPtr, "%i", &a_int);
            a = a_int / (float)FLOAT_PRECISION;
            goal_data_t goal;
            goal.ang_data = {a, 1};
            FifoPushGoal(order_id, TYPE_ANG, goal);
            break;
        }
        case ROTNOMODULO:
        {
            long a_int;
            float a;
            sscanf(receivedOrderPtr, "%li", &a_int);
            a = a_int / (float)FLOAT_PRECISION;
            goal_data_t goal;
            goal.ang_data = {a, 0};
            FifoPushGoal(order_id, TYPE_ANG, goal);
            break;
        }
        case PWM:
        {
            int l, r, t, s; //s = autoStop
            sscanf(receivedOrderPtr, "%i;%i;%i;%i", &l, &r, &t, &s);
            goal_data_t goal;
            goal.pwm_data = {(float)t, l, r, s};
            FifoPushGoal(order_id, TYPE_PWM, goal);
            break;
        }
        case SPD:
        {
            int l, a, t;
            sscanf(receivedOrderPtr, "%i;%i;%i", &l, &a, &t);
            goal_data_t goal;
            goal.spd_data = {(float)t, l, a};
            FifoPushGoal(order_id, TYPE_SPD, goal);
            break;
        }
        case PIDALL:
        case PIDRIGHT:
        case PIDLEFT:
        {
            long p_int, i_int, d_int;
            float p, i, d;
            sscanf(receivedOrderPtr, "%li;%li;%li", &p_int, &i_int, &d_int);
            p = p_int / (float)FLOAT_PRECISION;
            i = i_int / (float)FLOAT_PRECISION;
            d = d_int / (float)FLOAT_PRECISION;
            if (orderChar == PIDLEFT)
                PIDSet(&PID_left, p, i, d, LEFT_BIAS);
            else if (orderChar == PIDRIGHT)
                PIDSet(&PID_right, p, i, d, RIGHT_BIAS);
            else {
                PIDSet(&PID_left, p, i, d, LEFT_BIAS);
                PIDSet(&PID_right, p, i, d, RIGHT_BIAS);
            }
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        }
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
            control.last_finished_id = 0;
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        case SET_POS:
        {
            int x, y, a_int, mode;
            float a;
            sscanf(receivedOrderPtr, "%i;%i;%i;%i;", &x, &y, &a_int, &mode);
            a = a_int / (float)FLOAT_PRECISION;
            if(mode){
                if(!(mode & BIT_MODE_A))
                    a = current_pos.angle;
                if(!(mode & BIT_MODE_X))
                    x = round(current_pos.x); 
                if(!(mode & BIT_MODE_Y))
                    y = round(current_pos.y);
            }
            RobotStateSetPos(x, y, a);
            g_serialSender.serialSend(SERIAL_INFO, "%d;%i;", order_id, a_int);
            break;
        }
        case GET_POS:
        {
            int x, y, a_int;
            float a;
            a = current_pos.angle;
            x = round(current_pos.x);
            y = round(current_pos.y);
            a_int = a * (float)FLOAT_PRECISION;
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;%d;", order_id, x, y, a_int);
            break;
        }
        case GET_SPD:
        {
            int l, r;
            l = wheels_spd.left;
            r = wheels_spd.right;
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;", order_id, l, r);
            break;
        }
        case GET_TARGET_SPD:
        {
            int left_spd, right_spd;
            left_spd = control.speeds.linear_speed - control.speeds.angular_speed;
            right_spd = control.speeds.linear_speed + control.speeds.angular_speed;
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;", order_id, left_spd, right_spd);
            break;
        }
        case GET_POS_ID:
        {
            int x, y, a_int;
            float a;
            a = current_pos.angle;
            x = round(current_pos.x);
            y = round(current_pos.y);
            a_int = a * (float)FLOAT_PRECISION;
            g_serialSender.serialSend(SERIAL_INFO, "%d;%d;%d;%d;", order_id, x, y, a_int, control.last_finished_id);
            break;
        }
        case SPDMAX:
        {
            int r_int, s;
            float r;
            sscanf(receivedOrderPtr, "%i;%i", &s, &r_int);
            r = r_int / (float)FLOAT_PRECISION;
            control.max_spd = s;
            control.rot_spd_ratio = r;
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        }
        case ACCMAX:
        {
            int a;
            sscanf(receivedOrderPtr, "%i", &a);
            control.max_acc = a;
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
            int enable;
            sscanf(receivedOrderPtr, "%i", &enable);
            if (enable == 0) {
                emergencyStop(0);
            } else {
                emergencyStop(1);
            }
            g_serialSender.serialSend(SERIAL_INFO, "%d;", order_id);
            break;
        }
        default:
            g_serialSender.serialSend(SERIAL_INFO, "Order %c is wrong !", orderChar);
    }
}

