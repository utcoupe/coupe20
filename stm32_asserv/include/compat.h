/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 13/10/13			*
 ****************************************/
#ifndef COMPASTM_H
#define COMPASTM_H

#include "stm32f3xx_hal.h"
#include "encoder.h"
#include "parameters.h"

#define timeMillis() (HAL_GetTick())

// TODO: check eventual buffer overflow issue
// #define timeMicros() (DWT->CYCCNT/(HAL_RCC_GetHCLKFreq()/1000000L))
#ifdef __cplusplus
extern "C"
#endif // __cplusplus
long timeMicros();


#endif
