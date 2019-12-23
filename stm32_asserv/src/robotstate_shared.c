#include "robotstate_shared.h"
#include "local_math.h"

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