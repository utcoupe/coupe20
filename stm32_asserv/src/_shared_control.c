/**
 * Author : Paul Constant & Sébastien Darche
 * Date : 22/12/29
 * 
 * This file is used in both stm32_asserv and asserv_simu.py.
 * It regroups all functions linked to the command law of the robot.
 * 
 * A shared library is generated from this file and imported in the python 
 * code of the simulation. This way, the stm32 command law can be tested without 
 * the robots.
 * The file is imported like any other file in stm32_asserv.
 *
 * Be careful when editing : 
 * 
 * - If you change a function which is imported in the simulation asserv (name, parameters, return value),
 *   update the simulation asserv so that it does not break.
 * 
 * Functions imported in the simulation asserv (the ones which could make it break): 
 * - ControlLogicInit()
 * - ControlSetStop()
 * - ControlPrepareNewGoal()
 * - processCurrentGoal()
 * - setCurrentGoalReached()
 * 
 * 
 * - If you change the name of this file, make the change in the compile.sh script as well.
**/

#include "_shared_control.h"
#include "_shared_parameters.h"
#include "_shared_robotstate.h"
#include "_shared_local_math.h"
#include <math.h>

control_t control;
PID_t PID_left, PID_right;
pos_t current_pos;
wheels_spd_t wheels_spd;
uint8_t flagSTM32Connected;

void ControlLogicInit() {
	control.reset = 1;
	control.status_bits = 0;
	control.speeds.angular_speed = 0,
	control.speeds.linear_speed = 0;
	control.last_finished_id = 0;

	control.max_acc = ACC_MAX;
	control.max_spd = SPD_MAX; 
	control.rot_spd_ratio = RATIO_ROT_SPD_MAX;

	FifoInit();
	PIDInit(&PID_left);
	PIDInit(&PID_right);
	PIDSet(&PID_left, LEFT_P, LEFT_I, LEFT_D, LEFT_BIAS);
	PIDSet(&PID_right, RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_BIAS);
}

void ControlSetStop(int mask) {
	control.status_bits |= mask;
}

void ControlUnsetStop(int mask) {
	control.status_bits &= ~mask;
}

float splineInterpolation(pos_t pos, goal_t currGoal, goal_t nextGoal) {
	//Returns the curvature of the spline interpolation in order to generate the 
	//angular speed
	splinePoly_t x, y;
	
	// Computing the 2d spline interpolation
	// It's supposed to be a linear system solution but was considerably simplified
	// in order to fit it inside the stm32! (only works for this particular problem)

	
	float distToGoal = powf(powf(pos.x - currGoal.data.pos_data.x, 2) + powf(pos.y - currGoal.data.pos_data.y, 2), 0.5);
	float goalToNext = powf(powf(nextGoal.data.pos_data.x - currGoal.data.pos_data.x, 2) + powf(nextGoal.data.pos_data.y - currGoal.data.pos_data.y, 2), 0.5);

	// Current position (~t=0)
	x.d = pos.x;
	y.d = pos.y;
	
	// Current tan (derivative at t = 0)
	x.c = cosf(pos.angle);
	y.c = sinf(pos.angle);

	// Goal and next goal : we have to solve a simple linear system, the direct solution is
	// explicited here using the reduced echelon form

	float pivot = distToGoal / goalToNext;
	x.b = (currGoal.data.pos_data.x - pivot * nextGoal.data.pos_data.x - x.d - x.c*distToGoal) / powf(distToGoal, 2);
	y.b = (currGoal.data.pos_data.y - pivot * nextGoal.data.pos_data.y - y.d - y.c*distToGoal) / powf(distToGoal, 2);

	x.a = currGoal.data.pos_data.x - x.b * powf(goalToNext, 2) - x.c * goalToNext - x.d;
	y.a = currGoal.data.pos_data.y - y.b * powf(goalToNext, 2) - y.c * goalToNext - y.d;

	if(x.c*y.b - x.b*y.c < 0.001){ //If we try dividing by something too small : keep moving
		//The curvature is too small
		return 0;
	} else {
		// Now calculating angular speed using a wonderful formula
		return 10*powf(x.c * x.c + y.c * y.c, 1.5) / (2.0 * x.c * y.b - 2.0 * x.b * y.c);
	}
}

int controlPos(float dd, float da) {
	int ret;
	char interP = 0;
	float da_next = da, dda, ddd, max_speed;
	float ang_spd, lin_spd;

	int pos_error;
	float ddd_final, dda_next;
	
	if (FifoGetGoal(FifoCurrentIndex()+1)->type == TYPE_POS) {
		float dd_final;
		int dx, dy, x, y, goal_count;
		
		goal_t *current_goal = FifoCurrentGoal();
		goal_t *next_goal = FifoGetGoal(FifoCurrentIndex()+1);
		
		// Calculate angle between current and next goal
		x = current_goal->data.pos_data.x;
		y = current_goal->data.pos_data.y;
		dx = next_goal->data.pos_data.x - x;
		dy = next_goal->data.pos_data.y - y;
		da_next = atan2f(dy, dx) - current_pos.angle;

		
		
		if(wrapToPI(da_next) < MAX_ANGLE_DIFF_INTERPOL) {
			//if the angle to the next objective is not to high, we can use the interpolation
			interP = 1;
			dda = splineInterpolation(current_pos, *current_goal, *next_goal);
		}


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

	if(!interP)
		dda = da * (float)(ENTRAXE_ENC / 2.0);

	//If we really have to turn, don't go forward!
	if (fabs(da) > (float)MAX_ANGLE_DIFF) {
		ddd = 0;
	} else {
		ddd = dd * expf(-fabsf(K_DISTANCE_REDUCTION * da));
	}

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

/* INTERNAL FUNCTIONS */

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

void goalPwm(goal_t *goal, long now) {
	static long start_time;
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

void goalSpd(goal_t *goal, long now) {
	static long start_time;
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
	float speed;

    speed = fabsf(control.speeds.angular_speed);
    if (BRK_COEFF != 0.0) {
        speed -= control.max_acc * (float)(DT * BRK_COEFF);
    } else {
        speed = 0.0;
    }
    speed = fmaxf(0.0f, speed);
    control.speeds.angular_speed = speed;

    speed = fabsf(control.speeds.linear_speed);
    if (BRK_COEFF != 0.0) {
        speed -= control.max_acc * (float)(DT * BRK_COEFF);
    } else {
        speed = 0.0;
    }
    speed = fmaxf(0.0f, speed);
    control.speeds.linear_speed = sign(control.speeds.linear_speed)*speed;

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

void ControlPrepareNewGoal(void) {
	control.order_started = 0;
	PIDReset(&PID_left);
	PIDReset(&PID_right);
}

void processCurrentGoal(long now_micros) {
	goal_t* current_goal = FifoCurrentGoal();

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
				goalPwm(current_goal, now_micros);
				break;
			case TYPE_SPD:
				goalSpd(current_goal, now_micros);
				break;
			default:
				stopRobot();
				break;
		}
	}
}

void setCurrentGoalReached(void) {
	goal_t *current_goal = FifoCurrentGoal();
	control.last_finished_id = current_goal->ID;
	current_goal = FifoNextGoal();
	ControlPrepareNewGoal();
}