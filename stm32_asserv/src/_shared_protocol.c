#include "_shared_protocol.h"
#include "_shared_goals.h"
#include "_shared_control.h"
#include "_shared_robotstate.h"
#include <stdio.h>
#include <math.h>

control_t control;
uint8_t flagSTM32Connected = 0;

void start() {
    emergencyStop(0);
    flagSTM32Connected = 1;
}

void halt() {
    emergencyStop(1);
    flagSTM32Connected = 0;
}

void parseGOTO(char *receivedOrderPtr, int order_id) {
    int x, y, direction, slow_go;
    direction = 0;
    sscanf(receivedOrderPtr, "%i;%i;%i;%i", &x, &y, &direction, &slow_go);
    if(slow_go) 
        ControlSetStop(SLOWGO_BIT);    
    else
        ControlUnsetStop(SLOWGO_BIT);    

    goal_data_t goal;
    goal.pos_data.x = x;
    goal.pos_data.y = y;
    goal.pos_data.d = direction;
    FifoPushGoal(order_id, TYPE_POS, goal);
}

void parseGOTOA(char *receivedOrderPtr, int order_id) {
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
    goal.pos_data.x = x;
    goal.pos_data.y = y;
    goal.pos_data.d = direction;
    FifoPushGoal(order_id, TYPE_POS, goal);
    goal.ang_data.angle = a;
    goal.ang_data.modulo = 1;
    FifoPushGoal(order_id, TYPE_ANG, goal);
}

void parseROT(char *receivedOrderPtr, int order_id) {
    int a_int;
    float a;
    sscanf(receivedOrderPtr, "%i", &a_int);
    a = a_int / (float)FLOAT_PRECISION;
    goal_data_t goal;
    goal.ang_data.angle = a;
    goal.ang_data.modulo = 1;
    FifoPushGoal(order_id, TYPE_ANG, goal);
}

void parseROTNMODULO(char *receivedOrderPtr, int order_id) {
    long a_int;
    float a;
    sscanf(receivedOrderPtr, "%li", &a_int);
    a = a_int / (float)FLOAT_PRECISION;
    goal_data_t goal;
    goal.ang_data.angle = a;
    goal.ang_data.modulo = 1;
    FifoPushGoal(order_id, TYPE_ANG, goal);
}

void parsePWM(char *receivedOrderPtr, int order_id) {
    int l, r, t, s;
    sscanf(receivedOrderPtr, "%i;%i;%i;%i", &l, &r, &t, &s);
    goal_data_t goal;
    goal.pwm_data.time = (float)t;
    goal.pwm_data.pwm_l = l;
    goal.pwm_data.pwm_r = r;
    goal.pwm_data.auto_stop = s;

    FifoPushGoal(order_id, TYPE_PWM, goal);
}

void parseSPD(char *receivedOrderPtr, int order_id) {
    int l, a, t, s;
    sscanf(receivedOrderPtr, "%i;%i;%i;%i;", &l, &a, &t, &s);
    goal_data_t goal;

    goal.spd_data.time = (float)t;
    goal.spd_data.lin = l;
    goal.spd_data.ang = a;
    goal.spd_data.auto_stop = s;
    
    FifoPushGoal(order_id, TYPE_SPD, goal);
}

void parsePIDALL(char *receivedOrderPtr) {
    long p_int, i_int, d_int;
    float p, i, d;
    sscanf(receivedOrderPtr, "%li;%li;%li", &p_int, &i_int, &d_int);
    p = p_int / (float)FLOAT_PRECISION;
    i = i_int / (float)FLOAT_PRECISION;
    d = d_int / (float)FLOAT_PRECISION;
    PIDSet(&PID_left, p, i, d, LEFT_BIAS);
    PIDSet(&PID_right, p, i, d, RIGHT_BIAS);
}

void parsePIDRIGHT(char *receivedOrderPtr) {
    long p_int, i_int, d_int;
    float p, i, d;
    sscanf(receivedOrderPtr, "%li;%li;%li", &p_int, &i_int, &d_int);
    p = p_int / (float)FLOAT_PRECISION;
    i = i_int / (float)FLOAT_PRECISION;
    d = d_int / (float)FLOAT_PRECISION;
    PIDSet(&PID_right, p, i, d, RIGHT_BIAS);
}

void parsePIDLEFT(char *receivedOrderPtr) {
    long p_int, i_int, d_int;
    float p, i, d;
    sscanf(receivedOrderPtr, "%li;%li;%li", &p_int, &i_int, &d_int);
    p = p_int / (float)FLOAT_PRECISION;
    i = i_int / (float)FLOAT_PRECISION;
    d = d_int / (float)FLOAT_PRECISION;
    PIDSet(&PID_left, p, i, d, LEFT_BIAS);
}

void parseSETPOS(char *receivedOrderPtr) {
    int x, y, a_int, mode;
    float a;
    sscanf(receivedOrderPtr, "%i;%i;%i;%i", &x, &y, &a_int, &mode);
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
}

void parseSPDMAX(char *receivedOrderPtr) {
    int r_int, s;
    float r;
    sscanf(receivedOrderPtr, "%i;%i", &s, &r_int);
    r = r_int / (float)FLOAT_PRECISION;
    control.max_spd = s;
    control.rot_spd_ratio = r;
}

void parseACCMAX(char *receivedOrderPtr) {
    int a;
    sscanf(receivedOrderPtr, "%i", &a);
    control.max_acc = a;
}

void parseSETEMERGENCYSTOP(char *receivedOrderPtr) {
    // Reset the PID to remove the error sum
    int enable;
    sscanf(receivedOrderPtr, "%i", &enable);
    emergencyStop(enable);
}

void emergencyStop(int enable) {
    PIDReset(&PID_left);
    PIDReset(&PID_right);
    if (enable == 0) {
        ControlUnsetStop(EMERGENCY_BIT);
    } else {
        ControlSetStop(EMERGENCY_BIT);
    }
}

void resetID() {
    control.last_finished_id = 0;
}       
