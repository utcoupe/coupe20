/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/

#include "encoder.h"
#include "robotstate.h"
#include "goals.h"
#include "control.h" 
#include "compat.h"
#include "motor.h"
#include "local_math.h"
#include "compat.h"
#include "sender_wrapper.h"

#include <math.h>

#define ANG_REACHED (0x1)
#define POS_REACHED (0x2)
#define REACHED (ANG_REACHED | POS_REACHED)

#define sign(x) ((x)>=0?1:-1)

inline float fabsf(float x) { return (x >= 0 ? x : -x); }

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

void goalPos(goal_t *goal);
void goalPwm(goal_t *goal);
void goalSpd(goal_t *goal);
void goalAngle(goal_t *goal);
int controlPos(float dd, float da);

float calcSpeed(float init_spd, float dd, float max_spd, float final_speed);
void applyPID(void);
void applyPwm(void);
void allStop(void);
void stopRobot(void);

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

void goalAngle(goal_t *goal) {
	float angle, da;
	angle = goal->data.ang_data.angle;
	da = angle - current_pos.angle;

	if (goal->data.ang_data.modulo) {
		da = moduloTwoPI(da);
	}
	
	if (controlPos(0, da) & ANG_REACHED) {
		goal->is_reached = 1;
	}
	applyPID();
}

void goalPos(goal_t *goal) {
	int x, y;
	float dx, dy, da, dd, goal_a;

	x = goal->data.pos_data.x;
	y = goal->data.pos_data.y;
	dx = x - current_pos.x;
	dy = y - current_pos.y;
	goal_a = atan2f(dy, dx);
	da = (goal_a - current_pos.angle);
	da = moduloTwoPI(da);
	dd = sqrtf(powf(dx, 2.0)+powf(dy, 2.0));

	if (goal->data.pos_data.d == ANY) {
		if (fabsf(da) > (float)(CONE_ALIGNEMENT)) {
			da = moduloPI(da);
			dd = - dd;
		}
	} else if (goal->data.pos_data.d == BACKWARD) {
		dd = - dd;
		da = moduloTwoPI(da + (float)(M_PI));
	}

	if (controlPos(dd, da) & POS_REACHED) {
		goal->is_reached = 1;
	}
	applyPID();
}

int controlPos(float dd, float da) {
	int ret;
	float dda, ddd, max_speed;
	float ang_spd, lin_spd;

	int pos_error;
	float ddd_final, dda_next;

	if (FifoGetGoal(FifoCurrentIndex()+1)->type == TYPE_POS) {
		float da_next, dd_final;
		int dx, dy, x, y, goal_count;

		// Calculate angle between current and next goal
		goal_t *current_goal = FifoCurrentGoal();
		goal_t *next_goal = FifoGetGoal(FifoCurrentIndex()+1);

		x = current_goal->data.pos_data.x;
		y = current_goal->data.pos_data.y;
		dx = next_goal->data.pos_data.x - x;
		dy = next_goal->data.pos_data.y - y;
		da_next = atan2f(dy, dx) - current_pos.angle;
		
		// Calculate distance to final goal
		dd_final = 0;
		goal_count = 1;
		while (FifoGetGoal(FifoCurrentIndex()+goal_count)->type == TYPE_POS) {
			next_goal = FifoGetGoal(FifoCurrentIndex()+goal_count++);

			dx = next_goal->data.pos_data.x - x;
			dy = next_goal->data.pos_data.y - y;

			dd_final += sqrtf(powf(dx, 2.0) + powf(dy, 2.0));

			x = next_goal->data.pos_data.x;
			y = next_goal->data.pos_data.y;
		}

		// Adjust direction
		if (sign(dd) == -1) {
			da_next = da_next + (float)(M_PI);
			dd_final = -dd_final;
		}
		da_next = moduloTwoPI(da_next);

		pos_error = ERROR_INTERMEDIATE_POS;
		dda_next = da_next * (float)(ENTRAXE_ENC / 2.0);

		if (da_next > (float)MAX_ANGLE_DIFF)
			ddd_final = 0;
		else
			ddd_final = dd_final * expf(-fabsf(K_DISTANCE_REDUCTION * da_next));
	}
	else {
		pos_error = ERROR_POS;
		dda_next = 0;
		ddd_final = 0;
	}

	dda = da * (float)(ENTRAXE_ENC / 2.0);
	if (da > (float)MAX_ANGLE_DIFF) 
		ddd = 0;
	else
		ddd = dd * expf(-fabsf(K_DISTANCE_REDUCTION * da));

	max_speed = control.max_spd;
	if (control.status_bits & SLOWGO_BIT) {
		max_speed *= (float)EMERGENCY_SLOW_GO_RATIO;
	}

	ang_spd = control.speeds.angular_speed;
	lin_spd = control.speeds.linear_speed;

	control.speeds.angular_speed = calcSpeed(ang_spd, dda, 
			max_speed * control.rot_spd_ratio, dda_next);
	control.speeds.linear_speed = calcSpeed(lin_spd, ddd,
			max_speed, ddd_final);

	ret = 0;
	if (fabsf(dd) < pos_error) {
		ret |= POS_REACHED;
	}
	if (fabsf(da) < (float)ERROR_ANGLE) {
		ret |= ANG_REACHED;
	}

	return ret;
}

float calcSpeed(float init_spd, float dd, float max_spd, float final_speed) {
	float dd_abs, acc_spd, dec_spd, target_spd;
	int d_sign;
	dd_abs = fabsf(dd);
	d_sign = sign(dd);

	init_spd *= d_sign;
	acc_spd = init_spd + control.max_acc * (float)DT;
	dec_spd = sqrtf(powf(final_speed, 2) + 2*control.max_acc*dd_abs);
	target_spd = fminf(max_spd, fminf(acc_spd, dec_spd))*d_sign;
	return target_spd;
}

void stopRobot(void) {
	int sign;
	float speed;

    speed = fabsf(control.speeds.angular_speed);
    if (BRK_COEFF != 0.0) {
        speed -= control.max_acc * (float)(DT * BRK_COEFF);
    } else {
        speed = 0.0;
    }
    speed = fmaxf(0.0f, speed);
    control.speeds.angular_speed = speed;

    sign = sign(control.speeds.linear_speed);
    speed = fabsf(control.speeds.linear_speed);
    if (BRK_COEFF != 0.0) {
        speed -= control.max_acc * (float)(DT * BRK_COEFF);
    } else {
        speed = 0.0;
    }
    speed = fmaxf(0.0f, speed);
    control.speeds.linear_speed = sign*speed;

	if (fabsf(wheels_spd.left) + fabsf(wheels_spd.right) < (float)SPD_TO_STOP) {
		allStop();
	} else {
		applyPID();
	}
}

void allStop(void) {
	control.speeds.pwm_left = 0;
	control.speeds.pwm_right = 0;
	control.speeds.linear_speed = 0;
	control.speeds.angular_speed = 0;
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

void applyPID(void) {
	float left_spd, right_spd;
	float left_ds, right_ds;
	left_spd = control.speeds.linear_speed - control.speeds.angular_speed;
	right_spd = control.speeds.linear_speed + control.speeds.angular_speed;
	left_ds = left_spd - wheels_spd.left;
	right_ds = right_spd - wheels_spd.right;
	// Control feed forward
    //control.speeds.pwm_left = speedToPwm(left_spd) + PIDCompute(&PID_left, left_ds);
    //control.speeds.pwm_right = speedToPwm(right_spd) + PIDCompute(&PID_right, right_ds);
	control.speeds.pwm_left = (int)ceilf(PIDCompute(&PID_left, left_ds));
    control.speeds.pwm_right = (int)ceilf(PIDCompute(&PID_right, right_ds));
}
