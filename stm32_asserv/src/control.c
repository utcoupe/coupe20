/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/
//#include <math.h>
#include "encoder.h"
#include "robotstate.h"
#include "goals.h"
#include "control.h" 
#include "compat.h"
#include "motor.h"
// #include "local_math.h"
#include "emergency.h"
#include "protocol.h"
#include "canSender.h"

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


#define ANG_REACHED (0x1)
#define POS_REACHED (0x2)
#define REACHED (ANG_REACHED | POS_REACHED)


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

	MotorsInit();
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

	// clear emergency everytime, it will be reset if necessary
	ControlUnsetStop(EMERGENCY_BIT);
	ControlUnsetStop(SLOWGO_BIT);
	
	//if (ABS(control.speeds.linear_speed) > 1) {
		// int direction;
		// if (control.speeds.linear_speed >= 0) {
		// 	direction = EM_FORWARD;
		// } else {
		// 	direction = EM_BACKWARD;
		// }

	if (emergency_status[EM_FORWARD].phase == FIRST_STOP ||
		emergency_status[EM_BACKWARD].phase == FIRST_STOP) {
		ControlSetStop(EMERGENCY_BIT);
	} else if (emergency_status[EM_FORWARD].phase == SLOW_GO ||
			   emergency_status[EM_BACKWARD].phase == SLOW_GO) {
		ControlSetStop(SLOWGO_BIT);
	}
	//}


	if (control.status_bits & EMERGENCY_BIT  || 
		control.status_bits & PAUSE_BIT      ||
		control.status_bits & TIME_ORDER_BIT ) {
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
//        SerialSendWrapVar(SERIAL_INFO, "%d;", (int)control.last_finished_id);
        lastReachedID = control.last_finished_id;
#if KEEP_LAST_GOAL
        if ( FifoRemainingGoals() == 2)
#else
        if ( FifoRemainingGoals() == 1)
#endif
        {
			CanSender::canSend(ORDER_COMPLETED);
        }

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
	if ((now - start_time)/1000.0 <= goal->data.pwm_data.time){
		float pwmR, pwmL;
		pwmL = goal->data.pwm_data.pwm_l;
		pwmR = goal->data.pwm_data.pwm_r;

		control.speeds.pwm_left = pwmL;
		control.speeds.pwm_right = pwmR;
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
	if ((now - start_time)/1000.0 <= goal->data.spd_data.time){
		float time_left, v_dec;
		time_left = (goal->data.spd_data.time - ((now - start_time)/1000.0)) / 1000.0;
		v_dec = time_left * control.max_acc;

		control.speeds.linear_speed = min(min(
			control.speeds.linear_speed+DT*control.max_acc,
			goal->data.spd_data.lin),
			v_dec);
		control.speeds.angular_speed = min(min(
			control.speeds.angular_speed+DT*control.max_acc,
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
	goal_a = atan2(dy, dx);
	da = (goal_a - current_pos.angle);
	da = moduloTwoPI(da);
	dd = sqrt(pow(dx, 2.0)+pow(dy, 2.0));

	if (goal->data.pos_data.d == ANY) {
		if (ABS(da) > CONE_ALIGNEMENT) {
			da = moduloPI(da);
			dd = - dd;
		}
	} else if (goal->data.pos_data.d == BACKWARD) {
		dd = - dd;
		da = moduloTwoPI(da+M_PI);
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

	dda = da * (ENTRAXE_ENC/2);
	ddd = dd * exp(-ABS(K_DISTANCE_REDUCTION*da));

	max_speed = control.max_spd;
	if (control.status_bits & SLOWGO_BIT) {
		max_speed *= EMERGENCY_SLOW_GO_RATIO;
	}

	ang_spd = control.speeds.angular_speed;
	lin_spd = control.speeds.linear_speed;

	control.speeds.angular_speed = calcSpeed(ang_spd, dda, 
			max_speed * control.rot_spd_ratio, 0);
	control.speeds.linear_speed = calcSpeed(lin_spd, ddd,
			max_speed, 0);

	ret = 0;
	if (ABS(dd) < ERROR_POS) {
		ret |= POS_REACHED;
	}
	if (ABS(da) < ERROR_ANGLE) {
		ret |= ANG_REACHED;
	}

	return ret;
}

float calcSpeed(float init_spd, float dd, float max_spd, float final_speed) {
	float dd_abs, acc_spd, dec_spd, target_spd;
	int d_sign;
	dd_abs = ABS(dd);
	d_sign = sign(dd);

	init_spd *= d_sign;
	acc_spd = init_spd + (control.max_acc*DT);
	dec_spd = sqrt(pow(final_speed, 2) + 2*control.max_acc*dd_abs);
	target_spd = min(max_spd, min(acc_spd, dec_spd))*d_sign;
	return target_spd;
}

void stopRobot(void) {
	// int sign;
	// float speed;

//	sign = sign(control.speeds.angular_speed);
//	speed = abs(control.speeds.angular_speed);
//	speed -= control.max_acc * DT;
//	speed = max(0, speed);
//	control.speeds.angular_speed = speed;
//
//	sign = sign(control.speeds.linear_speed);
//	speed = abs(control.speeds.linear_speed);
//	speed -= control.max_acc * DT;
//	speed = max(0, speed);
//	control.speeds.linear_speed = sign*speed;
//
//	if (abs(wheels_spd.left) + abs(wheels_spd.right) < SPD_TO_STOP) {
//		allStop();
//	} else {
//		applyPID();
//	}
    allStop();
}

void allStop(void) {
	control.speeds.pwm_left = 0;
	control.speeds.pwm_right = 0;
	control.speeds.linear_speed = 0;
	control.speeds.angular_speed = 0;
}

void applyPwm(void) {
	set_pwm_left(control.speeds.pwm_left);
	set_pwm_right(control.speeds.pwm_right);
}

float speedToPwm(float speed) {
    float pwm = SPD_TO_PWM_A*speed;
    if     (speed > 0.001) pwm += SPD_TO_PWM_B;
    else if(speed < -0.001) pwm -= SPD_TO_PWM_B;
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
	control.speeds.pwm_left = PIDCompute(&PID_left, left_ds);
    control.speeds.pwm_right = PIDCompute(&PID_right, right_ds);
}
