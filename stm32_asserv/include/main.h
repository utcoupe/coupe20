/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
// #include <stdio.h>
// #include <iostream>
// #include <cstring>
// #include "constantes.h"

// #include "pins.h"
// #include "parameters.h"
// #include "Timer.h"
// #include "brushlessMotor.h"
// #include "motor.h"
// #include "block.h"
// #include "compat.h"
// #include "protocol.h"
// #include "control.h"
// #include "emergency.h"



/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ENC_R_A_Pin GPIO_PIN_0
#define ENC_R_A_GPIO_Port GPIOA
#define ENC_R_B_Pin GPIO_PIN_1
#define ENC_R_B_GPIO_Port GPIOA
#define ENC_L_B_Pin GPIO_PIN_4
#define ENC_L_B_GPIO_Port GPIOA
#define ENC_L_A_Pin GPIO_PIN_6
#define ENC_L_A_GPIO_Port GPIOA
#define MOT_L_EN_Pin GPIO_PIN_7
#define MOT_L_EN_GPIO_Port GPIOA
#define TEST_LED_Pin GPIO_PIN_3
#define TEST_LED_GPIO_Port GPIOB
#define MOT_R_EN_Pin GPIO_PIN_4
#define MOT_R_EN_GPIO_Port GPIOB
#define MOT_R_DIR_Pin GPIO_PIN_6
#define MOT_R_DIR_GPIO_Port GPIOB
#define MOT_L_DIR_Pin GPIO_PIN_7
#define MOT_L_DIR_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define SERIAL_DELAY 50


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
