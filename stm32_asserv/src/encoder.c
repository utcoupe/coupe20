/****************************************
 * Author : Mindstan                    *
 * Mail : mindstan@hotmail.fr           *
 * Date : 24/05/19			            *
 ****************************************/
#include "encoder.h"
#include "compat.h"
#include "pins.h"

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

int16_t uint32_to_int16(uint32_t val) {
    uint16_t newVal = val;
    if (val >= 0)
        return newVal & 0x7FFFu;
    return newVal | 0x8000u;
    // equivalent to :
    //  lsl r0, r0, #17
    //  lsr r0, r0, #17
    //  bx lr
}

int16_t get_left_encoder(void)
{
	return uint32_to_int16(L_ENC_TIM->CNT);
}

int16_t get_right_encoder(void)
{
	return uint32_to_int16(R_ENC_TIM->CNT);
}

void encoders_reset(void) {
	left_encoder_reset();
	right_encoder_reset();
}
