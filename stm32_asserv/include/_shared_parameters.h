/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/

#ifndef _SHARED_PARAMETERS_H
#define _SHARED_PARAMETERS_H

#define STM32_ID "ard_asserv"
#define DEBUG_TARGET_SPEED 0
#define DEBUG_MAINLOOP 0
#define DEBUG_LEVEL 2

#define ENCODER_EVAL 4

#define USE_SHARP 0
#define EMERGENCY_STOP_DISTANCE 0.3 // m

#define HZ 200
#define DT (1.0/HZ)
#define AUTO_STATUS_HZ 5 // must be a divider a HZ or 0 to disable
#define AUTO_STATUS_DT (int)1000.0*(1.0/AUTO_STATUS_HZ)

#define SPD_MAX 1000 //mm/s 1000
#define ACC_MAX 1500 //mm/s2
#define RATIO_ROT_SPD_MAX 1
#define K_DISTANCE_REDUCTION 15 // réduction de la vitesse linéaire quand on tourne
#define MAX_ANGLE_DIFF (M_PI/10.0) // rad - linear speed = 0 if angle > MAX_ANGLE_DIFF

#define EMERGENCY_WAIT_TIME 30 // seconds
#define EMERGENCY_SLOW_GO_RATIO 0.3 // spd = 0.3*max_spd in slow_go mode

#define BLOCK_TIME 5000 // ms - time between each block check
#define BLOCK_TIME_AUTO_STOP 250 // ms - time before pwm stops when hitting a wall in auto_stop mode

#define BLOCK_MIN_DIST 5 // mm - distance to move to consider we moved

#define ENC_RESOLUTION 1024 //resolution du codeur

#define ENC_LEFT_RADIUS 31.40 //REGLE PAR TEST - rayon de la roue codeuse (31.38 origin 31.15 good)
#define ENC_RIGHT_RADIUS 31.58 //REGLE PAR TEST - rayon de la roue codeuse
#define ENTRAXE_ENC 200.8 // PR 200.8 GR 303 // REGLE PAR TEST - Distance entre chaque roue codeuse en mm, base 200,8

#define ERROR_ANGLE 0.030 //erreur en angle(radians) maximale pour considérer l'objectif comme atteint
#define ERROR_POS 5 // erreur en position (mm)  maximale pour considérer l'objectif comme atteint
#define ERROR_INTERMEDIATE_POS 100 // mm Position error to consider an intermediate pose to be reached
#define SPD_TO_STOP 10

#define CONE_ALIGNEMENT (M_PI/2.0) 

#define PID_P 0.26
#define PID_I 250.0
#define PID_D 0.5
#define PID_BIAS 0
#define PID_I_MAX 150
#define PID_OUT_MAX 255

#define BRK_COEFF 3.0


// Control feed-forward, pwm = a*spd + b
#define SPD_TO_PWM_A 0.15
#define SPD_TO_PWM_B 5

#define LEFT_P (PID_P)
#define LEFT_I (PID_I)
#define LEFT_D (PID_D)
#define LEFT_BIAS (PID_BIAS)

#define RIGHT_P (PID_P)
#define RIGHT_I (PID_I)
#define RIGHT_D (PID_D)
#define RIGHT_BIAS (PID_BIAS)

#define PID_I_RATIO (1/1000.0)
#define PID_D_RATIO (1/1000.0)

#define TIME_BETWEEN_ORDERS 0 // s
#define KEEP_LAST_GOAL 0

//#define SERIAL_DEBUG

#endif
