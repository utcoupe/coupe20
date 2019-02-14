/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 18/04/15			*
 ****************************************/
#include "encoder.h"
#include "compat.h"
#include "pins.h"

// volatile long left_ticks = 0;
// volatile long right_ticks = 0;

// int left_last_value_A = 0;
// int left_last_value_B = 0;
// int right_last_value_A = 0;
// int right_last_value_B = 0;

void init_encoders(void)
{
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
}

void left_encoder_reset(void) {
	L_ENC_TIM->CNT = 0;
}

void right_encoder_reset(void) {
	R_ENC_TIM->CNT = 0;
}

int16_t get_left_encoder(void)
{
	return (int16_t)L_ENC_TIM->CNT;
}

int16_t get_right_encoder(void)
{
	return (int16_t)R_ENC_TIM->CNT;
}