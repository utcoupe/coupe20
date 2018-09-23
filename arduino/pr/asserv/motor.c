/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/

#include "parameters.h"
#include "brushlessMotor.h"

//Controleur :
//0:127   : Marche arriere
//127:255 : Marche avant

void set_pwm(int side, int pwm) {
	if (side == MOTOR_LEFT) {
		//les moteurs sont faces à face, pour avancer 
		//il faut qu'il tournent dans un sens différent
		pwm = -pwm;
	}

	pwm = (int)(pwm/2.0) + 127;

	if(pwm > 255)
		pwm = 255;
	else if(pwm < 0)
		pwm = 0;

	BrushlessMotorSetPwm(side, pwm);
}
