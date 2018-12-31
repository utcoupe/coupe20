/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/
#ifndef PARAMETERS_H
#define PARAMETERS_H

#define DEBUG_TARGET_SPEED 0
#define DEBUG_MAINLOOP 0
#define DEBUG_LEVEL 2

#define ENCODER_EVAL 1

#define USE_SHARP 0
#define EMERGENCY_STOP_DISTANCE 0.3 // m

#define HZ 200
#define DT (1.0/HZ)
#define AUTO_STATUS_HZ 2 // must be a divider a HZ or 0 to disable
#define AUTO_STATUS_DT (int)1000.0*(1.0/AUTO_STATUS_HZ)

#define SPD_MAX 1000 //mm/s 1000
#define ACC_MAX 1500 //mm/s2
#define RATIO_ROT_SPD_MAX 0.6
#define K_DISTANCE_REDUCTION 20 // réduction de la vitesse linéaire quand on tourne
#define EMERGENCY_WAIT_TIME 30 // seconds
#define EMERGENCY_SLOW_GO_RATIO 0.3 // spd = 0.3*max_spd in slow_go mode

#define BLOCK_TIME 5000 // ms - time between each block check
#define BLOCK_MIN_DIST 5 // mm - distance to move to consider we moved

#define ENC_RESOLUTION 4096 //resolution du codeur

// 31,24 and 31,38 initial
//31,05 and 31,20
#define ENC_LEFT_RADIUS 26 //REGLE PAR TEST - rayon de la roue codeuse (31.38 origin 31.15 good)
#define ENC_RIGHT_RADIUS 26.15 //REGLE PAR TEST - rayon de la roue codeuse (31.24)
#define ENTRAXE_ENC 215.25 //(201.35)REGLE PAR TES - Distance entre chaque roue codeuse en mm (202.37), base 200,8

#define ERROR_ANGLE 0.030 //erreur en angle(radians) maximale pour considérer l'objectif comme atteint
#define ERROR_POS 5 // erreur en position (mm)  maximale pour considérer l'objectif comme atteint
#define SPD_TO_STOP 10

#define CONE_ALIGNEMENT (M_PI/2.0) 

#define PID_P 0.32 //0.25
#define PID_I 11.0 //130
#define PID_D 0 // 0.5
#define PID_BIAS 130

// Control feed-forward, pwm = a*spd + b
#define SPD_TO_PWM_A 0.15
#define SPD_TO_PWM_B 5

#define LEFT_P 0.53
#define LEFT_I (PID_I)
#define LEFT_D (PID_D)
#define LEFT_BIAS 115

#define RIGHT_P 0.35
#define RIGHT_I (PID_I)
#define RIGHT_D (PID_D)
#define RIGHT_BIAS 115

#define PID_I_RATIO (1/1000.0)
#define PID_D_RATIO (1/1000.0)

#define TIME_BETWEEN_ORDERS 0 // s
#define KEEP_LAST_GOAL 0

//#define SERIAL_DEBUG

#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : -1))

#endif
