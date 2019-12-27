/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 25/10/13			*
 ****************************************/

#include "_shared_local_math.h"
#include <math.h>

float moduloTwoPI(float angle){
    while(angle > (float)M_PI) {
        angle -= (float)(2.0 * M_PI);
    }
    while(angle <= (float)-M_PI) {
        angle += (float)(2.0 * M_PI);
    }
    return angle;
}

float moduloPI(float angle){
    while(angle > (float)(M_PI / 2.0)) {
        angle -= (float)M_PI;
    }
    while(angle <= (float)(-M_PI/2.0)) {
        angle += (float)M_PI;
    }
    return angle;
}
