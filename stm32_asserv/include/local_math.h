/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 25/10/13			*
 ****************************************/

#ifndef LOCAL_MATH_H
#define LOCAL_MATH_H

#define MAX(a,b) a > b ? a : b
#define sign(x) ((x)>=0?1:-1)
#define ABS(x)   ((x) > 0 ? (x) : -(x))
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


float moduloTwoPI(float angle);
float moduloPI(float angle);

#endif
