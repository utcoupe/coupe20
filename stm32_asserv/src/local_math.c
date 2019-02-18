/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 25/10/13			*
 ****************************************/

#include "local_math.h"
#include <math.h>

float moduloTwoPI(float angle){
		while(angle > M_PI)
			angle -= 2.0*M_PI;
		while(angle <= -M_PI)
			angle += 2.0*M_PI;
	return angle;
}

float moduloPI(float angle){
		while(angle > M_PI/2)
			angle -= M_PI;
		while(angle <= -M_PI/2)
			angle += M_PI;
	return angle;
}
