//
// Created by tfuhrman on 13/04/18.
//

#ifndef ARDUINO_SENSOR_H
#define ARDUINO_SENSOR_H

#include "config_sensors.h"
#include <Arduino.h>
#include <ros.h>

//********************************************************************************************************************//
//
// Belt sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_BELT_ENABLED

void init_belt_sensors(ros::NodeHandle* nh);
void loop_belt_sensors();

#endif

//********************************************************************************************************************//
//
// Color sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_COLOR_ENABLED

// Public interface
enum rgb_name_enum {
    RGB_RED = 0,
    RGB_GREEN,
    RGB_BLUE
};

enum tsl_name_enum {
    TSL_HUE = 0,
    TSL_SATURATION,
    TSL_LIGHTNESS
};

void color_sensor_init(ros::NodeHandle* nh);
void color_sensor_loop();

// Private stuff
void color_sensor_values_capture();
void color_sensor_filter_apply(rgb_name_enum color);
void color_sensor_rgb_to_tsl(uint8_t rgbValues[3], uint16_t tslColors[3]);
void color_sensor_values_publish(uint16_t tslColors[3]);
void color_sensor_mean_value_compute(uint16_t tslColors[3]);

#endif

#endif //ARDUINO_SENSOR_H