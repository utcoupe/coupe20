/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 13/10/13			*
 ****************************************/
#ifndef _SHARED_GOALS_H
#define _SHARED_GOALS_H

#include <stdint.h>
#include "_shared_parameters.h"
#define MAX_GOALS 20 //nombre max de goals dans la file, évite surcharge mémoire

#define TYPE_POS 1
#define TYPE_ANG 2
#define TYPE_PWM 3
#define TYPE_SPD 4
#define NO_GOAL -1

typedef struct pos_data {
	int x, y, d;
} pos_data_t;

typedef struct ang_data {
	float angle;
	int modulo;
} ang_data_t;

typedef struct pwm_data {
	float time;
	int pwm_l, pwm_r;
	int auto_stop;
} pwm_data_t;

typedef struct spd_data {
	float time;
	int lin, ang;
	int auto_stop;
} spd_data_t;

typedef union goal_data {
	pos_data_t pos_data;
	ang_data_t ang_data;
	pwm_data_t pwm_data;
	spd_data_t spd_data;
} goal_data_t;

typedef struct goal {
	goal_data_t data;
	int type;
	uint16_t ID;
	int is_reached;
} goal_t;

typedef struct fifo {
	goal_t fifo[MAX_GOALS];
	int nb_goals;
	int current_goal;
	int last_goal;

} fifo_t;

extern fifo_t fifo;

#ifdef __cplusplus
extern "C" {
#endif

void FifoInit();
int FifoPushGoal(int ID, int type, goal_data_t data);
goal_t* FifoCurrentGoal();
goal_t* FifoNextGoal();
goal_t* FifoGetGoal(int index);

uint8_t FifoCurrentIndex();
uint8_t FifoRemainingGoals();
inline void FifoClearGoals() { FifoInit(); }

#ifdef __cplusplus
}
#endif

#endif
