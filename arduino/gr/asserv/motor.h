/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 31/03/13			*
 ****************************************/
#ifndef MOTOR_H
#define MOTOR_H

void set_pwm(int side, int pwm);

inline void MotorsInit(void) {};

void set_pwm_left(int pwm);

void set_pwm_right(int pwm);

#endif
