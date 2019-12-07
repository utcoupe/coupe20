/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 31/03/13			*
 ****************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "parameters.h"
#include "pins.h"
#include "stm32f3xx_hal.h"
#include "local_math.h"

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

#define LEFT_READY_SHIFT 1
#define RIGHT_READY_SHIFT 2

#define LEFT_READY (1<<LEFT_READY_SHIFT)
#define RIGHT_READY (1<<RIGHT_READY_SHIFT)

#ifdef __cplusplus
extern "C" {
#endif

void setPWM(int side, int pwm);

void motorsInit(void);
void setPWMLeft(int pwm);
void setPWMRight(int pwm);

#ifdef __cplusplus
}
#endif

#endif
