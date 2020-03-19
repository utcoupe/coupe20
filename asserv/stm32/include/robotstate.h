/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 13/10/13			*
 ****************************************/
#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "shared_asserv/parameters.h"
#include "encoder.h"
#include "shared_asserv/robotstate.h"
#include <math.h>

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

#ifdef __cplusplus
extern "C" {
#endif

void RobotStateInit();
void RobotStateUpdate();
inline void RobotStateReset(void) { RobotStateInit(); }

#ifdef __cplusplus
}
#endif

#endif
