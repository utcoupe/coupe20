/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 25/10/13			*
 ****************************************/

#include "local_math.h"
#include <math.h>

float moduloTwoPI(float angle){
	if(angle >= 0)
		while(angle > M_PI)
			angle -= 2.0*M_PI;
	else
		while(angle <= -M_PI)
			angle += 2.0*M_PI;
	return angle;
}

float moduloPI(float angle){
	if(angle >= 0)
		while(angle > M_PI/2)
			angle -= M_PI;
	else
		while(angle <= -M_PI/2)
			angle += M_PI;
	return angle;
}
