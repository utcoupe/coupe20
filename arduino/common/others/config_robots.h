//
// Created by tfuhrman on 27/04/18.
//

#ifndef ARDUINO_CONFIG_ROBOTS_H
#define ARDUINO_CONFIG_ROBOTS_H

// You have to select a robot and only one !!!
#define PR_ROBOT
//#define GR_ROBOT

/*
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!                                                                              !!!
 * !!! DON'T FORGET TO PUT THE PIN NUMBERS AND INIT VALUES IN config_robot.cpp FILE !!!
 * !!!                                                                              !!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

// If a stepper shield is connected, don't use the following pins : 3 4 7 8 11 12 13

//********************************************************************************************************************//
//
// PR ROBOT
//
//********************************************************************************************************************//

#if defined(PR_ROBOT) && !defined(GR_ROBOT)

// ---- DIGITAL ACTUATORS ----
#define NUM_DIGITAL_ACTUATORS 1

// ---- PWM ACTUATORS ----
#define NUM_PWM_ACTUATORS 1
#define REGULATED_ACTUATORS_ENABLED

// ---- SERVO ACTUATORS ----
#define NUM_SERVO_ACTUATORS 4

// ---- STEPPER ACTUATORS ----
#define NUM_STEPPER_ACTUATORS 0

// ---- BELT SENSOR ----
#define SENSOR_BELT_ENABLED
#define NUM_BELT_SENSORS 2

// ---- COLOR SENSOR ----
//#define SENSOR_COLOR_ENABLED
#define SENSOR_COLOR_S0              53
#define SENSOR_COLOR_S1              51
#define SENSOR_COLOR_S2              49
#define SENSOR_COLOR_S3              47
#define SENSOR_COLOR_LED             50
#define SENSOR_COLOR_SENSOR_VALUE    45

#endif

//********************************************************************************************************************//
//
// GR ROBOT
//
//********************************************************************************************************************//

#if defined(GR_ROBOT) && !defined(PR_ROBOT)

// ---- DIGITAL ACTUATORS ----
#define NUM_DIGITAL_ACTUATORS 1

// ---- PWM ACTUATORS ----
#define NUM_PWM_ACTUATORS 0

// ---- SERVO ACTUATORS ----
#define NUM_SERVO_ACTUATORS 1

// ---- STEPPER ACTUATORS ----
#define NUM_STEPPER_ACTUATORS 2

// ---- BELT SENSOR ----
#define SENSOR_BELT_ENABLED
#define NUM_BELT_SENSORS 2

#endif

#endif //ARDUINO_CONFIG_ROBOTS_H
