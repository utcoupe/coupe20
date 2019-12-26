/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 13/10/13			*
 ****************************************/
#include "robotstate.h"
#include "compat.h"
#include "local_math.h"
#include "encoder.h"
#include <math.h>

#define FC (10)
#define RC (1.0 / (2.0 * M_PI * FC))
#define ALPHA (DT / (RC + DT))

pos_t current_pos;
wheels_spd_t wheels_spd;

void RobotStateInit() {
	RobotStateLogicInit();
	encoders_reset();
}


void lowPass(wheels_spd_t *old_spd, wheels_spd_t *new_spd, float a) {
	new_spd->left = old_spd->left + a * (new_spd->left - old_spd->left);
	new_spd->right = old_spd->right + a * (new_spd->right - old_spd->right);
}

void RobotStateUpdate() {
	static int32_t left_last_ticks = 0, right_last_ticks = 0;
	static int32_t left_total_ticks = 0, right_total_ticks = 0;
	static float last_angle = 0;
	float dd, dl, dr, d_angle;
// 	int16_t lt, rt;
	wheels_spd_t old_wheels_spd = wheels_spd;

	left_total_ticks  += get_left_encoder();
	right_total_ticks += get_right_encoder();

	// lt = get_left_encoder();
	// rt = get_right_encoder();
	encoders_reset();


	dl = (left_total_ticks  - left_last_ticks)*TICKS_TO_MM_LEFT;
	dr = (right_total_ticks - right_last_ticks)*TICKS_TO_MM_RIGHT;
	wheels_spd.left = dl * HZ;
	wheels_spd.right = dr * HZ;

	

	// low pass filter on speed
	lowPass(&old_wheels_spd, &wheels_spd, ALPHA);

	d_angle = atan2((dr - dl), ENTRAXE_ENC); //sans approximation tan
	//d_angle = (dr - dl) / (float)ENTRAXE_ENC; // approximation tan
	current_pos.angle += d_angle;
#if MODULO_TWOPI
	PosUpdateAngle();
#endif

	dd = (dr + dl) / (float)2.0;
    float new_angle = (current_pos.angle + last_angle) / (float)2.0;
	current_pos.x += dd*cosf(new_angle);
	current_pos.y += dd*sinf(new_angle);
	// g_serial.print("|");
	// g_serial.print(lt);
	// g_serial.print("|");
	// g_serial.print(rt);
	// g_serial.print("|");
	// g_serial.print(left_total_ticks);
	// g_serial.print("|");
	// g_serial.print(right_total_ticks);
	// g_serial.print("|");
	// g_serial.print(left_last_ticks);
	// g_serial.print("|");
	// g_serial.print(right_last_ticks);
	// g_serial.print("|\n");
	// prepare la prochaine update
	right_last_ticks = right_total_ticks;
	left_last_ticks =  left_total_ticks;
	last_angle = current_pos.angle;
}
