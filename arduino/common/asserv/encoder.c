/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 18/04/15			*
 ****************************************/
#include "encoder.h"
#include "compat.h"
#include "pins.h"

volatile long left_ticks = 0;
volatile long right_ticks = 0;

int left_last_value_A = 0;
int left_last_value_B = 0;
int right_last_value_A = 0;
int right_last_value_B = 0;

void left_encoder_reset(void) {
	left_ticks = 0;
}

void right_encoder_reset(void) {
	right_ticks = 0;
}

#if ENCODER_EVAL == 4
inline void interruptA(volatile long *ticks, int *last_value_A, int last_value_B, int pin) {
	int new_value;
	new_value = digitalRead(pin);
	if(new_value == 1)
		if(last_value_B == 1)
			(*ticks)--;
		else
			(*ticks)++;

	else
		if(last_value_B == 1)
			(*ticks)++;
		else
			(*ticks)--;
	*last_value_A = new_value;
}

inline void interruptB(volatile long *ticks, int *last_value_B, int last_value_A, int pin) {
	bool new_value;
	new_value = digitalRead(pin);
	if(new_value == 1)
		if(last_value_A == 1)
			(*ticks)++;
		else
			(*ticks)--;

	else
		if(last_value_A == 1)
			(*ticks)--;
		else
			(*ticks)++;
	*last_value_B = new_value;
}

void leftInterruptA(void) {
	interruptA(&left_ticks, &left_last_value_A,
		       left_last_value_B, PIN_ENC_LEFT_A);
}

void rightInterruptA(void) {
	interruptA(&right_ticks, &right_last_value_A,
		       right_last_value_B, PIN_ENC_RIGHT_A);
}

void leftInterruptB(void) {
	interruptA(&left_ticks, &left_last_value_B,
		       left_last_value_A, PIN_ENC_LEFT_A);
}

void rightInterruptB(void) {
	interruptA(&right_ticks, &right_last_value_B,
		       right_last_value_A, PIN_ENC_RIGHT_A);
}
#elif ENCODER_EVAL == 2
void interruptA(volatile long *ticks, int pin_a, int pin_b){
	int value_A, value_B;
	value_A = digitalRead(pin_a);
	value_B = digitalRead(pin_b);
	if(value_A == 1)
		if(value_B == 1)
			(*ticks)--;
		else
			(*ticks)++;
	else
		if(value_B == 1)
			(*ticks)++;
		else
			(*ticks)--;
}

void leftInterruptA(void) {
	interruptA(&left_ticks, PIN_ENC_LEFT_A, PIN_ENC_LEFT_B);
}

void rightInterruptA(void) {
	interruptA(&right_ticks, PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B);
}
#elif ENCODER_EVAL == 1
void interruptA(volatile long *ticks, int pin_b){
	if(digitalRead(pin_b) == 1)
		(*ticks)--;
	else
		(*ticks)++;
}

void leftInterruptA(void) {
	interruptA(&left_ticks, PIN_ENC_LEFT_B);
}

void rightInterruptA(void) {
	interruptA(&right_ticks, PIN_ENC_RIGHT_B);
}
#endif
