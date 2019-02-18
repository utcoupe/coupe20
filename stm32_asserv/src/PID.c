/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 15/04/15			*
 ****************************************/

#include "parameters.h"
#include "PID.h"
#include "compat.h"

PID_t PID_left, PID_right;

void PIDInit(PID_t *pid) {
	pid->P = 0;
	pid->I = 0;
	pid->D = 0;
	pid->bias = 0;
	pid->error_sum = 0;
	pid->last_error = 0;
	pid->init_done = 0;
}

void PIDReset(PID_t *pid) {
	pid->error_sum = 0;
	pid->last_error = 0;
	pid->init_done = 0;
}

void PIDSet(PID_t *pid, float P, float I, float D, float bias) {
		I *= PID_I_RATIO;
		D *= PID_D_RATIO;
		I /= HZ;
		D *= HZ;
		pid->P = P;
		pid->I = I;
		pid->D = D;
		pid->bias = bias;
		PIDReset(pid);
}

float PIDCompute(PID_t *pid, float error) {
	float error_D, bias, P_part, I_part, D_part;

	if(!pid->init_done){ 
		//Lors du premier compute, on ne tient pas compte de D
		error_D = 0;
		pid->init_done = 1;	
	} else {
		//derivée = deltaErreur/dt - dt est la période de compute
		error_D = (error - pid->last_error); 
	}

	pid->error_sum = (pid->error_sum) + error;
	pid->last_error = error;
	
        //calcul de la sortie avec le PID
	bias = pid->bias;
	P_part = pid->P * error;
	I_part = pid->I * pid->error_sum;
	D_part = pid->D * error_D;
	pid->output = P_part + I_part + D_part ;

	//nothing happens if pid->output == 0
	if (pid->output > 0)
	{
		pid->output += bias;
		//pid->output = map(pid->output,0,255,bias,255);

	}
	else if (pid->output < 0)
	{
		pid->output -=bias;
		//pid->output = map(pid->output,-255,0,-255,-bias);
	}

	if (pid->output > 255)
	{
		pid->output = 255;
	}
	else if (pid->output < -255)
	{
		pid->output = -255;
	}

	return pid->output;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}