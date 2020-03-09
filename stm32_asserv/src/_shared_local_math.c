/**
 * Author : Paul Constant
 * Date : 22/12/29
 * 
 * This file is used in both stm32_asserv and asserv_simu.py.
 * It regroups math utility functions.
 * 
 * A shared library is generated from this file and imported in the python 
 * code of the simulation. This way, the stm32 command law can be tested without the robots.
 * The file is imported like any other file in stm32_asserv.
 * 
 * - If you change the name of this file, make the change in the compile.sh script as well.
**/

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

float wrapToPI(float angle) {
    //Returns an angle between 0 and M_PI
    while(angle > (float) M_PI) {
        angle -= (float) M_PI;
    }
    while(angle < 0) {
        angle += (float) M_PI;
    }
    return angle;
}
