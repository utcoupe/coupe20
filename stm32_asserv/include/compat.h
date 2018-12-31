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
#include "pins.h"



extern inline unsigned long timeMillis(){
	return HAL_GetTick();
}
extern inline unsigned long timeMicros(){
	// TODO: check eventual buffer overflow issue
	uint32_t freq = HAL_RCC_GetHCLKFreq();
	return DWT->CYCCNT/(freq/1000000L);
}

#ifdef __cplusplus
extern "C" void serial_print_int(int i);
extern "C" char generic_serial_read();
extern "C" void serial_send(char data);
extern "C" void serial_print(const char *str);
extern "C" void serial_print_float(float f);
#else
void serial_print_int(int i);
char generic_serial_read();
void serial_send(char data);
void serial_print(const char *str);
void serial_print_float(float f);
#endif

// extern inline void initPins(){
// 	//set mode des pins pour arduino
// 	pinMode(PIN_ENC_LEFT_A,INPUT_PULLUP);
// 	pinMode(PIN_ENC_LEFT_B,INPUT_PULLUP);
// 	pinMode(PIN_ENC_RIGHT_A,INPUT_PULLUP);
// 	pinMode(PIN_ENC_RIGHT_B,INPUT_PULLUP);

// 	pinMode(LED_DEBUG, OUTPUT);
// 	pinMode(LED_MAINLOOP, OUTPUT);
// 	pinMode(LED_BLOCKED, OUTPUT) ;

// 	digitalWrite(LED_DEBUG, LOW); //LOW = eteinte
// 	digitalWrite(LED_MAINLOOP, LOW); //LOW = eteinte
// 	digitalWrite(LED_BLOCKED, LOW); //LOW = eteinte
// 	//Definition des interruptions arduino en fonction du type d'évaluation
// #ifdef LED_JACK
// 	pinMode(LED_JACK, OUTPUT);
// #endif
// #if ENCODER_EVAL == 4
// 	attachInterrupt(INTERRUPT_ENC_LEFT_A,leftInterruptA,CHANGE);
// 	attachInterrupt(INTERRUPT_ENC_RIGHT_A,rightInterruptA,CHANGE);
// 	attachInterrupt(INTERRUPT_ENC_LEFT_B,leftInterruptB,CHANGE);
// 	attachInterrupt(INTERRUPT_ENC_RIGHT_B,rightInterruptB,CHANGE);
// #elif ENCODER_EVAL == 2
// 	attachInterrupt(INTERRUPT_ENC_LEFT_A,leftInterruptA,CHANGE);
// 	attachInterrupt(INTERRUPT_ENC_RIGHT_A,rightInterruptA,CHANGE);
// #elif ENCODER_EVAL == 1
// 	attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A),leftInterruptA,RISING);
// 	attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A),rightInterruptA,RISING);
// #endif
// #ifdef PIN_JACK
// 	pinMode(PIN_JACK,INPUT_PULLUP);
// #endif
// }

#endif
