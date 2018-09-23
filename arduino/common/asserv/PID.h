/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 15/04/15			*
 ****************************************/
#ifndef PID_H
#define PID_H

typedef struct PID {
	float P, I, D, bias;
	float error_sum, last_error;
	float output;
	int init_done;
} PID_t;


extern PID_t PID_left, PID_right;
void PIDInit(PID_t *pid);
void PIDReset(PID_t *pid);
void PIDSet(PID_t *pid, float P, float I, float D, float bias);
float PIDCompute(PID_t *pid, float error);

#endif
