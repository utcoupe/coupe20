//Par Quentin pour UTCoupe2013 01/04/2013
//Commande de shield arduino brushless by UTCoupe

#include <Arduino.h>
#include "brushlessMotor.h"

#define NO_PWM 127

/******************************************
               MOTORS
******************************************/
/*********************************
DIG1 and DIG2 defined to 0 :
linear adjustment of the speed

might be configured to smth else in order use speed control
see datasheet of DEC-MODULE-24/2
***********************************/

void BrushlessMotorsInit() {
		pinMode(MOTOR1_SPD, OUTPUT);
		pinMode(MOTOR1_EN, OUTPUT);
		pinMode(MOTOR1_BRK, OUTPUT);
		pinMode(MOTOR2_SPD, OUTPUT);
		pinMode(MOTOR2_EN, OUTPUT);
		pinMode(MOTOR2_BRK, OUTPUT);

		digitalWrite(MOTOR1_BRK, HIGH);
		digitalWrite(MOTOR2_BRK, HIGH);
		digitalWrite(MOTOR1_EN, LOW);
		digitalWrite(MOTOR2_EN, LOW);

		analogWrite(MOTOR1_SPD, NO_PWM);
		analogWrite(MOTOR2_SPD, NO_PWM);
}

void BrushlessMotorSetPwm(int motor_side, int pwm) {
	static int last_pwms[2] = {NO_PWM, NO_PWM};
	int *last_pwm;
	if (motor_side == MOTOR_LEFT) {
		last_pwm = &last_pwms[0];
	} else {
		last_pwm = &last_pwms[1];
	}
	if (pwm == *last_pwm) {
		return;
	}
	else {
		*last_pwm = pwm;
	}
	switch (motor_side) {
		case MOTOR_LEFT:{
			analogWrite(MOTOR1_SPD, pwm);
			if (pwm == NO_PWM) {
				digitalWrite(MOTOR1_EN,LOW);
			}
			else {
				digitalWrite(MOTOR1_EN,HIGH);
			}
            break;
		}
		case MOTOR_RIGHT:{
			analogWrite(MOTOR2_SPD, pwm);
			if (pwm == NO_PWM) {
				digitalWrite(MOTOR2_EN,LOW); 
			}
			else {
				digitalWrite(MOTOR2_EN,HIGH);
			}
			break;
		}
	}
}
