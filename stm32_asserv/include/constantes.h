// #ifndef CONSTANTES_H
// #define CONSTANTES_H

// #include "stm32f3xx_hal.h"
// /**
//  * Communication
//  */

// #define MAX_TAILLE 30 // caractères

// /**
//  * Mécanique
//  */
// //ATTENTION: Pour un asserv en vitesse, il faut 1 diamètre par roue codeuse
//  // + un entre axe pour le calcul de la vitesse angulaire
// #define DIAMETRE    52                                  // mm
// #define DIAMETRE_GAUCHE		52
// #define DIAMETRE_DROITE		52

// #define FACTEUR     1
// #define RESOLUTION  FACTEUR  * 1024                     
// #define PRECISION  (double)((PI*DIAMETRE) / (double)(RESOLUTION)) // mm.tick^-1
// #define PRECISION_GAUCHE  (double)((PI*DIAMETRE_GAUCHE) / (double)(RESOLUTION)) // mm.tick^-1
// #define PRECISION_DROITE  (double)((PI*DIAMETRE_DROITE) / (double)(RESOLUTION)) // mm.tick^-1
// #define MIN_PWM 	40 

// /**
//  * Asservissement
//  */

// #define DT          50                                  // ms
// #define V_MAX      (int)((750.0 / PRECISION))           // tick.s^-1
// #define A_MAX      (int)((500.0 / PRECISION))           // tick.s^-2
// #define ORDER      2                                    // commande trapézoïdale

// #define L_ENC_TIM TIM3
// #define R_ENC_TIM TIM2


// #define SERIAL_DELAY 50

// /**
//  * A modifier pour changer le sens de comptage des codeurs
//  */
// // #if L_ENC_TIM == TIM3
//  #define TIM3_IC1_POLARITY = TIM_ICPOLARITY_RISING
//  #define TIM3_IC2_POLARITY = TIM_ICPOLARITY_RISING

//  #define TIM2_IC1_POLARITY = TIM_ICPOLARITY_RISING
//  #define TIM2_IC2_POLARITY = TIM_ICPOLARITY_FALLING
// // #else
// //  #define TIM3_IC1_POLARITY = TIM_ICPOLARITY_RISING
// //  #define TIM3_IC2_POLARITY = TIM_ICPOLARITY_FALLING

// //  #define TIM2_IC1_POLARITY = TIM_ICPOLARITY_RISING
// //  #define TIM2_IC2_POLARITY = TIM_ICPOLARITY_RISING
// // #endif




// /**
//  * A modifier pour changer le sens de rotation des moteurs
//  */

// #define MOT_L_FORW  GPIO_PIN_RESET
// #define MOT_L_BACK GPIO_PIN_SET
// #define MOT_D_FORW  GPIO_PIN_SET
// #define MOT_D_BACK GPIO_PIN_RESET



// #endif
