/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 25/10/13			*
 ****************************************/

#ifndef LOCAL_MATH_H
#define LOCAL_MATH_H

#ifndef M_PI
#   define M_PI (3.14159265358979323846)
#endif

#ifndef ABS
#define ABS(x)   ((x) > 0 ? (x) : -(x))
#endif

float moduloTwoPI(float angle);
float moduloPI(float angle);

#endif
