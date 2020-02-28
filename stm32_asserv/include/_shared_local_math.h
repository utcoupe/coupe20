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

#ifndef _SHARED_LOCAL_MATH
#define _SHARED_LOCAL_MATH

#ifndef M_PI
#   define M_PI (3.14159265358979323846)
#endif

// #ifndef ABS
// #define ABS(x)   ((x) > 0 ? (x) : -(x))
// #endif

float moduloTwoPI(float angle);
float moduloPI(float angle);

#endif
