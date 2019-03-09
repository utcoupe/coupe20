#ifndef PINS_H
#define PINS_H

#include "stm32f3xx_hal.h"

// #define ENC_R_A_Pin GPIO_PIN_0
// #define ENC_R_A_GPIO_Port GPIOA
// #define ENC_R_B_Pin GPIO_PIN_1
// #define ENC_R_B_GPIO_Port GPIOA
// #define ENC_L_B_Pin GPIO_PIN_4
// #define ENC_L_B_GPIO_Port GPIOA
// #define ENC_L_A_Pin GPIO_PIN_6
// #define ENC_L_A_GPIO_Port GPIOA
// #define MOT_L_EN_Pin GPIO_PIN_7
// #define MOT_L_EN_GPIO_Port GPIOA
// #define TEST_LED_Pin GPIO_PIN_3
// #define TEST_LED_GPIO_Port GPIOB
// #define MOT_R_EN_Pin GPIO_PIN_4
// #define MOT_R_EN_GPIO_Port GPIOB
// #define MOT_R_DIR_Pin GPIO_PIN_6
// #define MOT_R_DIR_GPIO_Port GPIOB
// #define MOT_L_DIR_Pin GPIO_PIN_7
// #define MOT_L_DIR_GPIO_Port GPIOB

#define HIGH 	GPIO_PIN_SET
#define LOW 	GPIO_PIN_RESET

#define L_ENC_TIM TIM3
#define R_ENC_TIM TIM2

// mega
// #ifdef __AVR_ATmega2560__
// #define MOTOR1_EN 30
// #define MOTOR2_EN 34

// #define MOTOR1_SPD 3
// #define MOTOR2_SPD 2

// #define MOTOR1_RDY 32
// #define MOTOR2_RDY 36

// #define PIN_ENC_LEFT_A 19
// #define PIN_ENC_LEFT_B 18
// #define PIN_ENC_RIGHT_A 20
// #define PIN_ENC_RIGHT_B 21

// #define PIN_SHARP_FORWARD 14
// #define PIN_SHARP_BACKWARD 15

// #define INTERRUPT_ENC_LEFT_A 5
// #define INTERRUPT_ENC_LEFT_B 4
// #define INTERRUPT_ENC_RIGHT_A 3
// #define INTERRUPT_ENC_RIGHT_B 2

// //#define PIN_JACK 52
// //#define LED_JACK 51
// //
// #define LED_DEBUG 16
// #define LED_BLOCKED 16
// #define LED_MAINLOOP 16
// #endif

#endif
