/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/

#include "parameters.h"
#include "brushlessMotor.h"

//Controleur brushless (ancienne carte):
// SPD : 0-127 Marche arriere ; 127:255 Marche avant
// EN : 0 motors off ; 1 motors on

// Controleur brushless (carte noire):
// SPD 0-255 : Vitesse
// EN 0 motors off ; 1 motors on

void set_pwm(int side, int pwm) {
	if (side == MOTOR_LEFT) {
		//les moteurs sont faces à face, pour avancer 
		//il faut qu'il tournent dans un sens différent
		pwm = -pwm;
	}

	/*pwm = (int)(pwm/2.0) + 127;

	if(pwm > 255)
		pwm = 255;
	else if(pwm < 0)
		pwm = 0;*/

	BrushlessMotorSetPwm(side, pwm);
}
