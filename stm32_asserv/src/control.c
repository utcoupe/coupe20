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

control_t control;


void applyPwm(void);

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

void ControlCompute(void) {
#if TIME_BETWEEN_ORDERS
	static long time_reached = -1;
#endif

	long now;
	now = timeMicros();

	RobotStateUpdate();
	// Pass current time in argument to call function
	// from simulation without stm32 timer
	processCurrentGoal(now);
	applyPwm();

	if (FifoCurrentGoal()->is_reached) {
		// Instead of calling SerialSend directly (does not work),
		// we use a global variable to send the id from main
		SerialSendGoalReached((int)control.last_finished_id);

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
