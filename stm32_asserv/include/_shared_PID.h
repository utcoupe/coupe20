/**
 * Author : Paul Constant
 * Date : 22/12/29
 * 
 * This file is used in both stm32_asserv and asserv_simu.py.
 * It regroups PID utility functions.
 * 
 * A shared library is generated from this file and imported in the python 
 * code of the simulation. This way, the stm32 command law can be tested without the robots.
 * The file is imported like any other file in stm32_asserv.
 * 
 * - If you change the name of this file, make the change in the compile.sh script as well.
**/

#ifndef _SHARED_PID_H
#define _SHARED_PID_H



typedef struct PID {
	float P, I, D, bias;
	float error_sum, last_error;
	float output;
	int init_done;
} PID_t;

#ifdef __cplusplus
extern "C" {
#endif
extern PID_t PID_left, PID_right;


void PIDInit(PID_t *pid);
void PIDReset(PID_t *pid);
void PIDSet(PID_t *pid, float P, float I, float D, float bias);
float PIDCompute(PID_t *pid, float error);
//long map(long x, long in_min, long in_max, long out_min, long out_max);
#ifdef __cplusplus
}
#endif
#endif
