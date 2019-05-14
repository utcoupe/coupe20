//
// Created by tfuhrman on 13/04/18.
//

#ifndef ARDUINO_CONFIG_SENSORS_H
#define ARDUINO_CONFIG_SENSORS_H

#include "config_robots.h"

//********************************************************************************************************************//
//
// Belt sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_BELT_ENABLED
//TODO all this stuff !
#endif

//********************************************************************************************************************//
//
// Color sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_COLOR_ENABLED

// Pins
#define S0              SENSOR_COLOR_S0
#define S1              SENSOR_COLOR_S1
#define S2              SENSOR_COLOR_S2
#define S3              SENSOR_COLOR_S3
#define LED             SENSOR_COLOR_LED
#define SENSOR_VALUE    SENSOR_COLOR_SENSOR_VALUE

// Component configuration
#define COLOR_ACCUMULATE_NB     4
#define COLOR_MEDIAN_SIZE       5
#define COLOR_SENSOR_TIMEOUT    100000//in µs
#define SATURATION_MAX_VALUE    360
#define HUE_MAX_VALUE           340

#endif

#endif //ARDUINO_CONFIG_SENSORS_H
