/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/

#include "parameters.h"
#include "motor.h"

//Controleur :
//-255:0   : Marche arriere
//0:255 : Marche avant

void set_pwm(int side, int pwm) {
	if (side == MOTOR_LEFT) {
		//les moteurs sont faces à face, pour avancer 
		//il faut qu'il tournent dans un sens différent
		pwm = -pwm;
	}

	if(pwm > 255)
		pwm = 255;
	else if(pwm < -255)
		pwm = -255;
	BrushlessMotorSetPwm(side, pwm);
}

void MotorsInit(void) {
	BrushlessMotorsInit();
}

void set_pwm_left(int pwm){
	set_pwm(MOTOR_LEFT, pwm);
}

void set_pwm_right(int pwm){
	set_pwm(MOTOR_RIGHT, pwm);
}
