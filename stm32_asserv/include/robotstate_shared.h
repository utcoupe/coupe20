#ifndef ROBOTSTATE_SHARED_H
#define ROBOTSTATE_SHARED_H

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
#endif

void RobotStateSetPos(float x, float y, float angle);

#ifdef __cplusplus
}
#endif

#endif