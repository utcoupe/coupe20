//
// Created by tfuhrman on 09/05/17.
//

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "protocol.h"
#include <stdlib.h>
#include "canSender.h"
//get from old protocol file
// extern "C" {
#include "compat.h"
#include "robotstate.h"
#include "control.h"
#include "goals.h"
#include "emergency.h"
#include "parameters.h"
//}

void autoSendStatus() {
    //todo auto_send ?
    CanSender::canSend(CURRENT_POS, 
        (int)current_pos.x, 
        (int)current_pos.y,
        (int)(current_pos.angle*FLOAT_PRECISION));
    
    CanSender::canSend(CURRENT_PWM, 
        (int)control.speeds.pwm_left, 
        (int)control.speeds.pwm_right); 

    CanSender::canSend(CURRENT_SPD,
        (int)(control.speeds.linear_speed), 
        (int)wheels_spd.left, 
        (int)wheels_spd.right);

#ifdef SERIAL_DEBUG
    g_serial.print("x ");
    g_serial.print((int16_t)current_pos.x);
    g_serial.print("    y ");
    g_serial.print((int16_t)current_pos.y);
    g_serial.print("    ang ");
    g_serial.print((int16_t)(current_pos.angle*FLOAT_PRECISION));
    g_serial.print("    pwm l ");
    g_serial.print((int16_t)control.speeds.pwm_left);
    g_serial.print("    pwm r ");
    g_serial.print((int16_t)control.speeds.pwm_right);
    g_serial.print("    l spd ");
    g_serial.print((int16_t)control.speeds.linear_speed);
    g_serial.print("    l wheel ");
    g_serial.print((int16_t)wheels_spd.left);
    g_serial.print("    r wheel ");
    g_serial.print((int16_t)wheels_spd.right);
    g_serial.print("\n");
#endif

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

bool flagConnected = false;

//order is order;id_servo;params
void parseAndExecuteOrder(uint8_t* message) {
    uint8_t mode = message[0];
    int order_id = 0;

    int8_t value;

    switch (mode) 
    {
        case HANDSHAKE:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("HANDSHAKE");
            g_serial.print("\n");
#endif
            CanSender::canSend(WHOAMI,STM_CAN_ADDR);
            break;
        }

        // case WHOAMI:
        // {
        //     break;
        // }

        case SET_MODE:
        {
            break;
        }

        case SPEED:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("SPEED");
            g_serial.print("\n");
#endif
            int16_t l, a, t;
            //sscanf(receivedOrderPtr, "%i;%i;%i", &l, &a, &t);

            l =     message[1] << 8 |  message[2];

            a =     message[3] << 8 | message[4];

            t =     message[5] << 8 | message[6];

            goal_data_t goal;
            goal.spd_data = {(float)t, l, a};
            FifoPushGoal(order_id, TYPE_SPD, goal);
            break;
        }

        case GET_CODER:
        {
#ifdef SERIAL_DEBUG

            g_serial.print("GET CODER");
            g_serial.print("\n");

#endif
            // CanSender::canSend(GET_CODER,get_left_encoder(), get_right_encoder());
            break;
        }

        case MANAGEMENT:
        {
            uint8_t order = message[1];
            switch(order)
            {
                case STOP:
                {
#ifdef SERIAL_DEBUG                    
                    g_serial.print("STOP");
                    g_serial.print("\n");
#endif
                    //CanSender::canSend(WHOAMI,CAN_ADDR);
                    flagConnected = false;
                    break;
                }
                case START:
                {   
#ifdef SERIAL_DEBUG
                    g_serial.print("START");
                    g_serial.print("\n");
#endif
                    //CanSender::canSend(1,32);
                    flagConnected = true;
                    break;
                }
                case PAUSE:
                {
#ifdef SERIAL_DEBUG
                    g_serial.print("PAUSE");
                    g_serial.print("\n");
#endif
                    ControlSetStop(PAUSE_BIT);
                    break;
                }
                case RESUME:
                {
#ifdef SERIAL_DEBUG
                    g_serial.print("RESUME");
                    g_serial.print("\n");
#endif
                    ControlUnsetStop(PAUSE_BIT);
                    break;
                }
                case RESET_ID:
                {
#ifdef SERIAL_DEBUG
                    g_serial.print("RESET_ID");
                    g_serial.print("\n");
#endif
                    control.last_finished_id = 0;
                    break;
                }
                case SETEMERGENCYSTOP:
                {
#ifdef SERIAL_DEBUG
                    g_serial.print("EMGSTOP");
                    g_serial.print("\n");
#endif
                    EmergencySetStatus(1);
                    break;
                }
                case NEXT_ORDER:
                {
#ifdef SERIAL_DEBUG
                    g_serial.print("NEXT GOAL");
                    g_serial.print("\n");
#endif
                    FifoNextGoal();
                    ControlPrepareNewGoal();
                    break;
                }
                case RESET_ORDERS:
                {
#ifdef SERIAL_DEBUG
                    g_serial.print("RESET GOALS");
                    g_serial.print("\n");
#endif
                    FifoClearGoals();
                    ControlPrepareNewGoal();
                    break;
                }
                case UNSETEMERGENCYSTOP:
                {
#ifdef SERIAL_DEBUG
                    g_serial.print("RESET EMGSTOP");
                    g_serial.print("\n");
#endif
                    EmergencySetStatus(0);

                }
            }

            break;
        }

        case GOTOA:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("GOTOA");
            g_serial.print("\n");
#endif
            int16_t x, y, a_int, direction;
            float a;
            direction = 0;

            x =     message[1] << 8 | message[2];

            y =     message[3] << 8 | message[4];

            a_int =     message[5] << 8 | message[6];

            direction = message[7];
#ifdef SERIAL_DEBUG
            g_serial.print(x);
            g_serial.print("\n");
            g_serial.print(y);
            g_serial.print("\n");
            g_serial.print(a_int);
            g_serial.print("\n");
            g_serial.print(direction);
            g_serial.print("\n");
#endif
            a = a_int / (float)FLOAT_PRECISION;
            goal_data_t goal;
            goal.pos_data = {x, y, direction};
            FifoPushGoal(order_id, TYPE_POS, goal);
            goal.ang_data = {a, 1};
            FifoPushGoal(order_id, TYPE_ANG, goal);
            break;
        }

        case GOTO:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("GOTO");
            g_serial.print("\n");
#endif
            int16_t x, y;
            int8_t direction;

            x =     message[1] << 8 | message[2];

            y =     message[3] << 8 | message[4];

            // g_serial.print(x);
            // g_serial.print("\n");
            // g_serial.print(y);
            // g_serial.print("\n");

            direction = message[5];
            
            goal_data_t goal;
            goal.pos_data = {x, y, direction};
            FifoPushGoal(order_id, TYPE_POS, goal);
            break;
        }

        case ROT:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("ROT");
            g_serial.print("\n");
#endif
            int16_t a_int;
            float a;

            a_int =     message[1] << 8 | message[2];

            // g_serial.print(a_int);
            // g_serial.print("\n");

            a = a_int / (float)FLOAT_PRECISION;
            goal_data_t goal;
            goal.ang_data = {a, 1};
            FifoPushGoal(order_id, TYPE_ANG, goal);
            break;
        }

        case ROTNOMODULO:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("ROT NO MODULO");
            g_serial.print("\n");
#endif
            long a_int;
            float a;
            
            a_int =     message[1] << 8 | message[2];
            // g_serial.print(a_int);
            // g_serial.print("\n");
            

            a = a_int / (float)FLOAT_PRECISION;
            goal_data_t goal;
            goal.ang_data = {a, 0};
            FifoPushGoal(order_id, TYPE_ANG, goal);
            break;
        }

        case PIDLEFT:
        case PIDRIGHT:
        case PIDALL:
        {
            long p_int, i_int, d_int;
            float p, i, d;
            
            p_int =     message[1] << 8 | message[2];

            i_int =     message[3] << 8 | message[4];

            d_int =     message[5] << 8 | message[6];

            // g_serial.print(p_int);
            // g_serial.print("\n");
            // g_serial.print(i_int);
            // g_serial.print("\n");
            // g_serial.print(d_int);
            // g_serial.print("\n");
            
            p = (float)p_int / (float)FLOAT_PRECISION;
            i = (float)i_int / (float)FLOAT_PRECISION;
            d = (float)d_int / (float)FLOAT_PRECISION;

            
            if (mode == PIDLEFT)
            {
#ifdef SERIAL_DEBUG
                g_serial.print("PID LEFT");
                g_serial.print("\n");
#endif
                PIDSet(&PID_left, p, i, d, LEFT_BIAS);
            }
            else if (mode == PIDRIGHT)
            {
#ifdef SERIAL_DEBUG
                g_serial.print("PID RIGHT");
                g_serial.print("\n");
#endif
                PIDSet(&PID_right, p, i, d, RIGHT_BIAS);
            }
            else 
            {
#ifdef SERIAL_DEBUG
                g_serial.print("PID ALL");
                g_serial.print("\n");
#endif
                PIDSet(&PID_left, p, i, d, LEFT_BIAS);
                PIDSet(&PID_right, p, i, d, RIGHT_BIAS);
            }
            // CanSender::canSend(SERIAL_INFO, "%d;", order_id);
            break;
        }

        case PWM:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("PWM");
            g_serial.print("\n");
#endif
            int16_t l, r;
            uint16_t t;

            l =     message[1] << 8 | message[2];

            r =     message[3] << 8 | message[4];

            t =     message[5] << 8 | message[6];

            // g_serial.print(l);
            // g_serial.print("\n");
            // g_serial.print(r);
            // g_serial.print("\n");
            // g_serial.print(t);
            // g_serial.print("\n");

            goal_data_t goal;
            goal.pwm_data = {(float)t, l, r};
            FifoPushGoal(order_id, TYPE_PWM, goal);
            break;
        }

        case SET_POS:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("SET POS");
            g_serial.print("\n");
#endif
            int16_t x, y, a_int;
            float angle;

            
            x =     message[1] << 8 | message[2];

            y =     message[3] << 8 | message[4];

            a_int = message[5] << 8 | message[6];

            angle = a_int / (float)FLOAT_PRECISION;

            char buffer[200];
            int ret = snprintf(buffer, sizeof buffer, "%lf", angle);

            // g_serial.print(x);
            // g_serial.print("\n");
            // g_serial.print(y);
            // g_serial.print("\n");
            // g_serial.print(a_int);
            // g_serial.print("\n");

            



            RobotStateSetPos(x, y, angle);
            break;
        }

        case SET_PARAM:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("SET PARAM");
            g_serial.print("\n");
#endif
            int16_t r_int, s,a;
            float r;
            
            s =     message[1] << 8 | message[2];

            r_int = message[3] << 8 | message[4];

            a =     message[5] << 8 | message[6];

            // g_serial.print(s);
            // g_serial.print("\n");
            // g_serial.print(r_int);
            // g_serial.print("\n");
            // g_serial.print(a);
            // g_serial.print("\n");

            r = r_int / (float)FLOAT_PRECISION;
            control.max_spd = s;
            control.rot_spd_ratio = r;
            control.max_acc = a;
            break;
        }

        default:
        {
#ifdef SERIAL_DEBUG
            g_serial.print("default handling\n");
#endif SERIAL_DEBUG
        }

        
    }
}