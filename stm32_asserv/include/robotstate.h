/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 13/10/13			*
 ****************************************/
#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "parameters.h"
#include "encoder.h"
#include <math.h>
 #include "serial.h"

#if ENCODER_EVAL == 4
	#define TICKS_PER_TURN (ENC_RESOLUTION * 4)
#elif ENCODER_EVAL == 2
	#define TICKS_PER_TURN (ENC_RESOLUTION * 2)
#elif ENCODER_EVAL == 1
	#define TICKS_PER_TURN ENC_RESOLUTION
#endif
#define TICKS_TO_MM_LEFT ((float)((2.0*M_PI*ENC_LEFT_RADIUS)/(TICKS_PER_TURN)))// = mm/ticks
#define MM_TO_TICKS_LEFT ((float)(1/ENC_TICKS_TO_MM_LEFT))
#define TICKS_TO_MM_RIGHT ((float)((2.0*M_PI*ENC_RIGHT_RADIUS)/(TICKS_PER_TURN)))// = mm/ticks
#define MM_TO_TICKS_RIGHT ((float)(1/ENC_TICKS_TO_MM_RIGHT))

typedef struct pos {
	float x;
	float y;
	float angle;
	int modulo_angle;
} pos_t;

typedef struct wheels_spd {
	float left, right;
} wheels_spd_t;

extern pos_t current_pos;
extern wheels_spd_t wheels_spd;
extern Serial g_serial;

void RobotStateInit();
void RobotStateUpdate();
void RobotStateSetPos(float x, float y, float angle);
extern inline void RobotStateReset(void) { RobotStateInit(); };

#endif