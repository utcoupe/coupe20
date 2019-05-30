//
// Created by tfuhrman on 28/04/18.
//

#include "config_robots.h"
#include "stdint.h"
#include "Arduino.h"

// TODO this way to do is kind of ugly, think how to not duplicate variable declaration
// TODO find a way to just declares pins as define and to automatically fill the variable structures

//********************************************************************************************************************//
//
// PR ROBOT
//
//********************************************************************************************************************//

#if defined(PR_ROBOT) && !defined(GR_ROBOT)

uint8_t pins_belt_sensors_shut[NUM_BELT_SENSORS]        = {1, 3};
uint8_t belt_sensors_addresses[NUM_BELT_SENSORS]        = {22, 24};
String belt_sensors_names[NUM_BELT_SENSORS]             = {"sensor1", "sensor2"};

#endif

//********************************************************************************************************************//
//
// GR ROBOT
//
//********************************************************************************************************************//

#if defined(GR_ROBOT) && !defined(PR_ROBOT)

uint8_t pins_belt_sensors_shut[NUM_BELT_SENSORS]        = {1, 3};
uint8_t belt_sensors_addresses[NUM_BELT_SENSORS]        = {22, 24};
String belt_sensors_names[NUM_BELT_SENSORS]             = {"sensor1", "sensor2"};

#endif
