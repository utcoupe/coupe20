/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 31/03/13			*
 ****************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "brushlessMotor.h"

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

#ifdef __cplusplus
extern "C" {
#endif
void set_pwm(int side, int pwm);

void MotorsInit(void);
void set_pwm_left(int pwm);
void set_pwm_right(int pwm);

#ifdef __cplusplus
}
#endif

#endif
