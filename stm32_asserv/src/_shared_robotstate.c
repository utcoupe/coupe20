#include "_shared_robotstate.h"
#include "_shared_local_math.h"

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