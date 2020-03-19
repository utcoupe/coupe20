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

#include "shared_asserv/robotstate.h"
#include "shared_asserv/local_math.h"

void PosUpdateAngle() {
	if (current_pos.angle > (float)M_PI) {
		current_pos.angle -= (float)(2.0 * M_PI);
		current_pos.modulo_angle++;
	}
	else if (current_pos.angle <= (float) -M_PI) {
		current_pos.angle += (float)(2.0 * M_PI);
		current_pos.modulo_angle--;
	}
}

void RobotStateSetPos(float x, float y, float angle) {

	current_pos.x = x;
	current_pos.y = y;
	current_pos.angle = angle;

	PosUpdateAngle();
}

void RobotStateLogicInit() {
	current_pos.x = 0;
	current_pos.y = 0;
	current_pos.angle = 0;
	current_pos.modulo_angle = 0;
	wheels_spd.left = 0;
	wheels_spd.right = 0;
}