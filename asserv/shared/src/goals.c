/**
 * Author : Paul Constant
 * Date : 22/12/29
 * 
 * This file is used in both stm32_asserv and asserv_simu.py.
 * It regroups all functions linked to the goal list of the robot.
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
 * - FifoInit()
 * - FifoCurrentGoal()
 * - FifoNextGoal()
 * - FifoGetGoal()
 * 
 * - If you change the name of this file, make the change in the compile.sh script as well.
**/

#include "shared_asserv/goals.h"


fifo_t fifo;

void FifoInit() {
	int i;
	fifo.nb_goals = 0;
	fifo.current_goal = 0;
	fifo.last_goal = -1;
	for (i=0; i<MAX_GOALS; i++) {
		fifo.fifo[i].type = NO_GOAL;
	}
}

int FifoPushGoal(int ID, int type, goal_data_t data) {
	goal_t *new_goal;

	fifo.last_goal = (fifo.last_goal + 1) % MAX_GOALS;
	new_goal = &fifo.fifo[fifo.last_goal % MAX_GOALS];

	new_goal->type = type;
	new_goal->data = data;
	new_goal->ID = ID;
	new_goal->is_reached = 0;

	fifo.nb_goals++;
	return 0;

}

goal_t* FifoCurrentGoal() {
	return &fifo.fifo[fifo.current_goal % MAX_GOALS];
}

goal_t* FifoNextGoal() {
	goal_t *current_goal = FifoCurrentGoal();
#if KEEP_LAST_GOAL
	if (current_goal->type != NO_GOAL &&
		fifo.nb_goals > 1) {
#else
	if (current_goal->type != NO_GOAL) {
#endif
		current_goal->type = NO_GOAL;
		current_goal->is_reached = 0;
		fifo.current_goal = (fifo.current_goal + 1);
		fifo.nb_goals--;
		
	}
	return FifoCurrentGoal();
}

goal_t* FifoGetGoal(int i) {
	return &fifo.fifo[i % MAX_GOALS];
}

uint8_t FifoRemainingGoals()
{
	return fifo.nb_goals;
}

uint8_t FifoCurrentIndex() {
	return fifo.current_goal;
}