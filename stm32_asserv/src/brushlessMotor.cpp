//Par Quentin pour UTCoupe2013 01/04/2013
//Commande de shield arduino brushless by UTCoupe


#include "brushlessMotor.h"

#define NO_PWM 0
extern Pwm g_right_pwm;
extern Pwm g_left_pwm;
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
		
		//TODO: gérer les pins de brake
		//HAL_GPIO_WritePin()
		// digitalWrite(MOTOR1_BRK, LOW);
		// digitalWrite(MOTOR2_BRK, LOW);
		//TODO: avoir un PWM externe
		g_right_pwm.set_duty_cycle(NO_PWM);
		g_left_pwm.set_duty_cycle(NO_PWM);
		// analogWrite(MOTOR1_EN, NO_PWM);
		// analogWrite(MOTOR2_EN, NO_PWM);
		// SPD is the new DIR
		HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port,MOT_L_DIR_Pin,LOW);
		HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port,MOT_R_DIR_Pin,LOW);
		// HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port,MOT_R_DIR_Pin, LOW);
		// HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port,MOT_R_DIR_Pin, LOW);

		// digitalWrite(MOTOR1_REF, HIGH);
		// digitalWrite(MOTOR2_REF, HIGH);
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
			if (pwm == NO_PWM) {
				//TODO:
				g_left_pwm.set_duty_cycle(NO_PWM);
				// digitalWrite(MOTOR1_BRK, LOW);
			}
			else
			{
				//TODO:
				g_left_pwm.set_duty_cycle( ABS(pwm));
				// digitalWrite(MOTOR1_BRK, HIGH);
			}
			if (pwm > 0) {
				HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port,MOT_L_DIR_Pin,HIGH);
				
			}
			else {
				HAL_GPIO_WritePin(MOT_L_DIR_GPIO_Port,MOT_L_DIR_Pin,LOW);
			}
		    	break;
		}
		case MOTOR_RIGHT:{
			if (pwm == NO_PWM) {
				//TODO:
				g_right_pwm.set_duty_cycle( NO_PWM);
				// digitalWrite(MOTOR2_BRK, LOW);
			}
			else
			{
				//TODO:
				g_right_pwm.set_duty_cycle( ABS(pwm));
				// digitalWrite(MOTOR2_BRK, HIGH);
			}
			if (pwm > 0) {
				HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port,MOT_R_DIR_Pin,HIGH);
			}
			else {
				HAL_GPIO_WritePin(MOT_R_DIR_GPIO_Port,MOT_R_DIR_Pin,LOW);
			}
			break;
		}
	}
}
