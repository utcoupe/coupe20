/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 18/04/15			*
 ****************************************/
#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_tim.h"
#include "parameters.h"


// extern volatile long left_ticks;
// extern volatile long right_ticks;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void init_encoders(void);

void left_encoder_reset(void);
void right_encoder_reset(void);

int16_t get_left_encoder(void);
int16_t get_right_encoder(void);

extern inline void encoders_reset(void) {
	left_encoder_reset();
	right_encoder_reset();
}




#endif
