//
// Created by tfuhrman on 13/04/18.
//

#ifndef ARDUINO_CONFIG_ACTUATORS_H
#define ARDUINO_CONFIG_ACTUATORS_H

#include "config_robots.h"

//********************************************************************************************************************//
//
// Regulated actuators
//
//********************************************************************************************************************//

#define REGULATED_ACTUATORS_INTERRUPT_PIN       2
#define REGULATED_ACTUATORS_PIN                 8
// other coeff for control with integral instead of proportional : P = 25 and I = 0.15
#define REGULATED_ACTUATORS_PID_P               50.0f
#define REGULATED_ACTUATORS_PID_I               0.08f
#define REGULATED_ACTUATORS_PID_D               0.0f
#define REGULATED_ACTUATORS_CONTROL_LOOP_MS     100
#define REGULATED_ACTUATORS_TICS_PER_REVOLUTION 10.0f
#define REGULATED_ACTUATORS_PWM_MIN             10
#define REGULATED_ACTUATORS_PWM_MAX             255


#endif //ARDUINO_CONFIG_ACTUATORS_H
