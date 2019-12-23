#ifndef CONTROL_SHARED_H
#define CONTROL_SHARED_H

#include <stdint.h>
#include "PID.h"
#include "goals.h"

#define ANY 0
// TODO never Used ????
#define FORWARD 1
#define BACKWARD -1

#define ANG_REACHED (0x1)
#define POS_REACHED (0x2)
#define REACHED (ANG_REACHED | POS_REACHED)

#define PAUSE_BIT (1<<0)
#define EMERGENCY_BIT (1<<1)
#define SLOWGO_BIT (1<<2)
#define TIME_ORDER_BIT (1<<3)

#define sign(x) ( x >= 0 ? 1 : -1)

typedef struct control {
	struct speeds {
		int pwm_left, pwm_right;
		float angular_speed, linear_speed;
	} speeds;
	float max_acc, max_spd, rot_spd_ratio;
	uint8_t reset;
    uint16_t last_finished_id;
    uint16_t order_started;
	int status_bits;
} control_t;

extern control_t control;
extern PID_t PID_left, PID_right;

void goalPos(goal_t *goal);
void goalAngle(goal_t *goal);
int controlPos(float dd, float da);

float calcSpeed(float init_spd, float dd, float max_spd, float final_speed);
void applyPID(void);

void allStop(void);
void stopRobot(void);
#endif