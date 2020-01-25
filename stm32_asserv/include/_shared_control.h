#ifndef _SHARED_CONTROL_H
#define _SHARED_CONTROL_H

#include <stdint.h>
#include "_shared_PID.h"
#include "_shared_goals.h"

#define BACKWARD 0
#define FORWARD 1
#define ANY 2

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

void ControlLogicInit();
void ControlSetStop(int mask);
void ControlUnsetStop(int mask);

void goalPos(goal_t *goal);
void goalAngle(goal_t *goal);
void goalPwm(goal_t *goal, long now);
void goalSpd(goal_t *goal, long now);

int controlPos(float dd, float da);
float calcSpeed(float init_spd, float dd, float max_spd, float final_speed);
void applyPID(void);

void allStop(void);
void stopRobot(void);

void ControlPrepareNewGoal(void);
void processCurrentGoal(long);
void setCurrentGoalReached(void);
#endif