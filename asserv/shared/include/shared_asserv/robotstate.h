/**
 * Author : Paul Constant
 * Date : 22/12/29
 * 
 * This file is used in both stm32_asserv and asserv_simu.py.
 * It regroups SOME functions linked to the robot state.
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
 * - RobotStateLogicInit()
 * - RobotStateSetPos()
 * 
 * - If you change the name of this file, make the change in the compile.sh script as well.
**/

#ifndef SHARED_ASSERV_ROBOTSTATE_H
#define SHARED_ASSERV_ROBOTSTATE_H

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

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

	void RobotStateSetPos(float x, float y, float angle);
	void RobotStateLogicInit();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // SHARED_ASSERV_ROBOTSTATE_H