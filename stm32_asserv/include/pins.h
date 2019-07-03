#ifndef PINS_H
#define PINS_H

#include "stm32f3xx_hal_def.h"
#include "stm32f3xx_hal.h"

// ***************************************************
// * WARNING: THESE PINS ARE DEFINED FOR STM32F303K8 *
// ***************************************************

// **************** ENCODERS *****************
// Right encoder, signal A: PA_0 (A0)
#define ENC_R_A_Pin GPIO_PIN_0
#define ENC_R_A_GPIO_Port GPIOA

// Right encoder, signal B: PA_1 (A1)
#define ENC_R_B_Pin GPIO_PIN_1
#define ENC_R_B_GPIO_Port GPIOA

// Left encoder, signal A: PA_6 (A5)
#define ENC_L_A_Pin GPIO_PIN_6
#define ENC_L_A_GPIO_Port GPIOA

// Left encoder, signal B: PA_4 (A3)
#define ENC_L_B_Pin GPIO_PIN_4
#define ENC_L_B_GPIO_Port GPIOA

// ******************* MOTORS *****************

// Left motor, PWM control: PA_7 (A6)
#define MOT_L_EN_Pin GPIO_PIN_7
#define MOT_L_EN_GPIO_Port GPIOA

// Left motor, direction of rotation: PB_7 (D4)
#define MOT_L_DIR_Pin GPIO_PIN_7
#define MOT_L_DIR_GPIO_Port GPIOB

// Right motor, PWM control: PA_12 (D2)
#define MOT_R_EN_Pin GPIO_PIN_12
#define MOT_R_EN_GPIO_Port GPIOA

// Right motor, direction of rotation: PB_1 (D6)
#define MOT_R_DIR_Pin GPIO_PIN_1
#define MOT_R_DIR_GPIO_Port GPIOB

// ******************* OTHERS *******************

// Built-in LED3: PB_3 (D13), SHOULD NOT BE CONNECTED TO ANYTHING ELSE
#define TEST_LED_Pin GPIO_PIN_3
#define TEST_LED_GPIO_Port GPIOB

#define HIGH 	GPIO_PIN_SET
#define LOW 	GPIO_PIN_RESET

#define L_ENC_TIM TIM3
#define R_ENC_TIM TIM2

#endif
