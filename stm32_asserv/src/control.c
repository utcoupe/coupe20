/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/

#include "encoder.h"
#include "robotstate.h"
#include "goals.h"
#include "control.h" 
#include "control_shared.h"
#include "compat.h"
#include "motor.h"
#include "local_math.h"
#include "compat.h"
#include "sender_wrapper.h"

#include <math.h>

//inline float fminf(float x, float y) {
//    return (x < y ? x: y);
//}
//
//inline float fmaxf(float x, float y) {
//    return (x > y ? x : y);
//}

//char sign(float x) {
//    return (x >= 0 ? 1 : -1);
//}

uint16_t lastReachedID = 0;

control_t control;

void goalPwm(goal_t *goal);
void goalSpd(goal_t *goal);

void applyPwm(void);

void ControlSetStop(int mask) {
	control.status_bits |= mask;
}

void ControlUnsetStop(int mask) {
	control.status_bits &= ~mask;
}

void ControlInit(void) {
	control.reset = 1;
	control.status_bits = 0;
	control.speeds.angular_speed = 0,
	control.speeds.linear_speed = 0;
	control.last_finished_id = 0;

	control.max_acc = ACC_MAX;
	control.max_spd = SPD_MAX; 
	control.rot_spd_ratio = RATIO_ROT_SPD_MAX;

	motorsInit();
	RobotStateInit();
	FifoInit();
	PIDInit(&PID_left);
	PIDInit(&PID_right);
	PIDSet(&PID_left, LEFT_P, LEFT_I, LEFT_D, LEFT_BIAS);
	PIDSet(&PID_right, RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_BIAS);
}

void ControlReset(void) {
	control.speeds.linear_speed = 0;
	control.last_finished_id = 0;
	FifoClearGoals();
	RobotStateReset();
	ControlPrepareNewGoal();
}

void ControlPrepareNewGoal(void) {
	control.order_started = 0;
	PIDReset(&PID_left);
	PIDReset(&PID_right);
}

void ControlCompute(void) {
#if TIME_BETWEEN_ORDERS
	static long time_reached = -1;
	long now;
	now = timeMicros();
#endif
	goal_t* current_goal = FifoCurrentGoal();
	RobotStateUpdate();

	if (
        control.status_bits & EMERGENCY_BIT
        || control.status_bits & PAUSE_BIT
        || control.status_bits & TIME_ORDER_BIT
    ) {
		stopRobot();
	} else {
		switch (current_goal->type) {
			case TYPE_ANG:
				goalAngle(current_goal);
				break;
			case TYPE_POS:
				goalPos(current_goal);
				break;
			case TYPE_PWM:
				goalPwm(current_goal);
				break;
			case TYPE_SPD:
				goalSpd(current_goal);
				break;
			default:
				stopRobot();
				break;
		}
	}

	applyPwm();

	if (current_goal->is_reached) {
		control.last_finished_id = current_goal->ID;
        // Instead of calling SerialSend directly (does not work), we use a global variable to send the id from main
        SerialSendGoalReached((int)control.last_finished_id);
        lastReachedID = control.last_finished_id;
		FifoNextGoal();
		ControlPrepareNewGoal();

#if TIME_BETWEEN_ORDERS
		time_reached = now;
	}
	if (time_reached > 0 && (now - time_reached) < TIME_BETWEEN_ORDERS*1000000) {
		ControlSetStop(TIME_ORDER_BIT);
	} else {
		ControlUnsetStop(TIME_ORDER_BIT);
		time_reached = -1;
#endif
	}
}

/* INTERNAL FUNCTIONS */

void goalPwm(goal_t *goal) {
	static long start_time;
	long now = timeMicros();
	if (!control.order_started){
		start_time = now;
		control.order_started = 1;
	}
	if ((float)((now - start_time)/1000.0) <= goal->data.pwm_data.time){
		control.speeds.pwm_left = goal->data.pwm_data.pwm_l;
		control.speeds.pwm_right = goal->data.pwm_data.pwm_r;
	}
	else {
		control.speeds.pwm_left = 0;
		control.speeds.pwm_right = 0;
		goal->is_reached = 1;
	}
}

void goalSpd(goal_t *goal) {
	static long start_time;
	long now = timeMicros();
	if (!control.order_started){
		start_time = now;
		control.order_started = 1;
	}
	if ((float)((now - start_time)/1000.0) <= goal->data.spd_data.time){
		float time_left, v_dec;
		time_left = (goal->data.spd_data.time - (float)(((now - start_time)/1000.0))) / (float)1000.0;
		v_dec = time_left * control.max_acc;

		control.speeds.linear_speed = fminf(fminf(
			control.speeds.linear_speed + (float)DT * control.max_acc,
			goal->data.spd_data.lin),
			v_dec);
		control.speeds.angular_speed = fminf(fminf(
			control.speeds.angular_speed + (float)(DT) * control.max_acc,
			goal->data.spd_data.ang),
			v_dec);
	}
	else {
		control.speeds.linear_speed = 0;
		control.speeds.angular_speed = 0;
		goal->is_reached = 1;
	}
	applyPID();
}

void applyPwm(void) {
	setPWMLeft(control.speeds.pwm_left);
	setPWMRight(control.speeds.pwm_right);
}

float speedToPwm(float speed) {
    float pwm = (float)SPD_TO_PWM_A * speed;
    if     (speed > (float)0.001)
        pwm += SPD_TO_PWM_B;
    else if(speed < (float)(-0.001))
        pwm -= SPD_TO_PWM_B;
    return pwm;
}
