//
// Created by tfuhrman on 13/04/18.
//

#include "sensors.h"
#include "VL53L0X.h"
#include <drivers_ard_others/Color.h>
#include <drivers_ard_others/BeltRange.h>

//********************************************************************************************************************//
//
// Belt sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_BELT_ENABLED

ros::NodeHandle* node_handle_belt_sensor = NULL;

extern uint8_t pins_belt_sensors_shut[];
extern uint8_t belt_sensors_addresses[];
extern  String belt_sensors_names[];
VL53L0X belt_sensors[NUM_BELT_SENSORS];

drivers_ard_others::BeltRange belt_range_msg;
ros::Publisher belt_ranges_pub("/drivers/ard_others/belt_ranges", &belt_range_msg);

void init_belt_sensors(ros::NodeHandle* nh) {
    node_handle_belt_sensor = nh;
    if (node_handle_belt_sensor) {
        node_handle_belt_sensor->advertise(belt_ranges_pub);
    }
    for(uint8_t i = 0; i < NUM_BELT_SENSORS; i++) {
        pinMode(pins_belt_sensors_shut[i], OUTPUT);
        digitalWrite(pins_belt_sensors_shut[i], LOW);
    }

    for(uint8_t i = 0; i < NUM_BELT_SENSORS; i++) {
        pinMode(pins_belt_sensors_shut[i], INPUT);
        delay(50);
        belt_sensors[i].init(true);
        delay(100);
        belt_sensors[i].setAddress(belt_sensors_addresses[i]);
    }

    for(uint8_t i = 0; i < NUM_BELT_SENSORS; i++) {
        belt_sensors[i].setTimeout(500);
        // Do not use this as the time of first measurement will be too big
//        belt_sensors[i].setMeasurementTimingBudget(200000);
        belt_sensors[i].startContinuous();
    }
}

void loop_belt_sensors() {
    for(uint8_t i = 0; i < NUM_BELT_SENSORS; i++) {
        belt_range_msg.sensor_id = belt_sensors_names[i].c_str();
        unsigned long now = millis();
        belt_range_msg.range = belt_sensors[i].readRangeContinuousMillimeters() / 1000.0; //in meters
        unsigned long diff = millis() - now;
        if (belt_range_msg.range > 65534)
            belt_range_msg.range = -1;
        belt_ranges_pub.publish(&belt_range_msg);
//        node_handle_belt_sensor->loginfo(String("Belt : " + String(diff)).c_str());
    }
}

#endif

//********************************************************************************************************************//
//
// Color sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_COLOR_ENABLED

ros::NodeHandle* node_handle_color_sensor = NULL;

drivers_ard_others::Color color_msg;
ros::Publisher color_pub("/drivers/ard_others/color", &color_msg);

// This structure is used to store the current values
uint8_t color_sensor_rgb_values[3];

// This structure is used to store the median computed values in order to compute a mean value
uint16_t color_sensor_previous_tsl_values[COLOR_ACCUMULATE_NB][3];
uint8_t color_sensor_previous_tsl_values_index = 0;

//used to map rawFrequency read from sensor to a RGB value on 8 bytes
//those data have to be calibrated to be optimal
//the index is rgb_name_enum
uint8_t rgbMinMaxFrequency[3][2] = {
        {25, 54},
        {15, 100},
        {10, 90}
};

void color_sensor_init(ros::NodeHandle* nh) {
    node_handle_color_sensor = nh;
    if (node_handle_color_sensor) {
        node_handle_color_sensor->advertise(color_pub);
    }
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(SENSOR_VALUE, INPUT);
    // Setting frequency-scaling to 20%
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);
    // TODO LED never powered off
    digitalWrite(LED, HIGH);
}

void color_sensor_loop() {
//    unsigned long start_time = micros();
    uint16_t rgbColorAccumulator[3] = {0, 0, 0};
    uint8_t rgbMeanValues[3] = {0, 0, 0};
    uint16_t tslValues[3] = {0, 0, 0};
    uint8_t color_sensor_rgb_values_array[COLOR_MEDIAN_SIZE][3];
    uint8_t color_sensor_rgb_values_sorted_array[COLOR_MEDIAN_SIZE][3];
    // First accumulate color sensor values to be more accurate
    for (uint8_t accumulator_nb = 0; accumulator_nb < COLOR_MEDIAN_SIZE; accumulator_nb++) {
        color_sensor_values_capture();
        color_sensor_rgb_values_array[accumulator_nb][RGB_RED] = color_sensor_rgb_values[RGB_RED];
        color_sensor_rgb_values_array[accumulator_nb][RGB_GREEN] = color_sensor_rgb_values[RGB_GREEN];
        color_sensor_rgb_values_array[accumulator_nb][RGB_BLUE] = color_sensor_rgb_values[RGB_BLUE];
    }
    // Sort the values to compute the median
    for (uint8_t accumulator_nb = 0; accumulator_nb < COLOR_MEDIAN_SIZE; ++accumulator_nb) {
        uint8_t min_values_rgb[3] = {color_sensor_rgb_values_array[accumulator_nb][RGB_RED],
                                     color_sensor_rgb_values_array[accumulator_nb][RGB_GREEN],
                                     color_sensor_rgb_values_array[accumulator_nb][RGB_BLUE]};
        for (uint8_t min_nb = accumulator_nb; min_nb < COLOR_MEDIAN_SIZE; ++min_nb) {
            if (color_sensor_rgb_values_array[min_nb][RGB_RED] < min_values_rgb[RGB_RED]) {
                min_values_rgb[RGB_RED] = color_sensor_rgb_values_array[min_nb][RGB_RED];
            }
            if (color_sensor_rgb_values_array[min_nb][RGB_GREEN] < min_values_rgb[RGB_GREEN]) {
                min_values_rgb[RGB_GREEN] = color_sensor_rgb_values_array[min_nb][RGB_GREEN];
            }
            if (color_sensor_rgb_values_array[min_nb][RGB_BLUE] < min_values_rgb[RGB_BLUE]) {
                min_values_rgb[RGB_BLUE] = color_sensor_rgb_values_array[min_nb][RGB_BLUE];
            }
        }
        color_sensor_rgb_values_sorted_array[accumulator_nb][RGB_RED] = min_values_rgb[RGB_RED];
        color_sensor_rgb_values_sorted_array[accumulator_nb][RGB_GREEN] = min_values_rgb[RGB_GREEN];
        color_sensor_rgb_values_sorted_array[accumulator_nb][RGB_BLUE] = min_values_rgb[RGB_BLUE];
    }
    // Extract the median
    rgbMeanValues[RGB_RED] = color_sensor_rgb_values_sorted_array[COLOR_MEDIAN_SIZE / 2][RGB_RED];
    rgbMeanValues[RGB_GREEN] = color_sensor_rgb_values_sorted_array[COLOR_MEDIAN_SIZE / 2][RGB_GREEN];
    rgbMeanValues[RGB_BLUE] = color_sensor_rgb_values_sorted_array[COLOR_MEDIAN_SIZE / 2][RGB_BLUE];
    // Compute the corresponding tsl colors
    color_sensor_rgb_to_tsl(rgbMeanValues, tslValues);
    // Add the data to the previous values
    color_sensor_previous_tsl_values[color_sensor_previous_tsl_values_index][TSL_HUE] = tslValues[TSL_HUE];
    color_sensor_previous_tsl_values[color_sensor_previous_tsl_values_index][TSL_SATURATION] = tslValues[TSL_SATURATION];
    color_sensor_previous_tsl_values[color_sensor_previous_tsl_values_index][TSL_LIGHTNESS] = tslValues[TSL_LIGHTNESS];
    color_sensor_previous_tsl_values_index = ++color_sensor_previous_tsl_values_index % COLOR_ACCUMULATE_NB;
    // Compute the shifting mean value of tsl values
    uint16_t tsl_mean_values[3] = {0, 0, 0};
    color_sensor_mean_value_compute(tsl_mean_values);
    // Finally send the data
    color_sensor_values_publish(tsl_mean_values);
//    unsigned long diff_time = micros() - start_time;
//    String tmp_str = String(diff_time, DEC);
//    node_handle->loginfo(tmp_str.c_str());
}

void color_sensor_values_capture() {
    uint8_t rawFrequency = 0;
    for (uint8_t color_id = 0; color_id < 3; color_id++) {
        color_sensor_filter_apply((rgb_name_enum)color_id);
        rawFrequency = pulseIn(SENSOR_VALUE, LOW);
        if (rawFrequency > 0) {
            color_sensor_rgb_values[color_id] = constrain(map(rawFrequency, rgbMinMaxFrequency[color_id][0], rgbMinMaxFrequency[color_id][1], 255, 0), 0, 255);
        } else {
            node_handle_color_sensor->logwarn("Color sensor timed out, no values...");
        }
    }
}

void color_sensor_filter_apply(rgb_name_enum color) {
    switch (color) {
        case RGB_RED:
            digitalWrite(S2,LOW);
            digitalWrite(S3,LOW);
            break;
        case RGB_GREEN:
            digitalWrite(S2,HIGH);
            digitalWrite(S3,HIGH);
            break;
        case RGB_BLUE:
            digitalWrite(S2,LOW);
            digitalWrite(S3,HIGH);
            break;
        default:
            break;
    }
}

void color_sensor_rgb_to_tsl(uint8_t rgbValues[3], uint16_t tslColors[3]) {
    uint16_t maxColorValue = (uint16_t)rgbValues[RGB_RED];
    uint16_t minColorValue  = (uint16_t)rgbValues[RGB_RED];
    uint8_t maxColorValueIndex = 0;
    // Compute min and max color
    for (uint8_t index = 1; index < 3; index++) {
        if ((uint16_t)rgbValues[index] > maxColorValue) {
            maxColorValue = (uint16_t)rgbValues[index];
            maxColorValueIndex = index;
        }
        if ((uint16_t)rgbValues[index] < minColorValue) {
            minColorValue = (uint16_t)rgbValues[index];
        }
    }
    // Use float values, not best idea but standard TSL calculation can't be done in integers...
    uint16_t delta = maxColorValue - minColorValue;
    if (delta != 0) {
        float deltaf = (float)delta;
        float hue;
        if ((rgb_name_enum)maxColorValueIndex == RGB_RED)
        {
            hue = (float)(rgbValues[RGB_GREEN] - rgbValues[RGB_BLUE]) / delta;
        }
        else
        {
            if ((rgb_name_enum)maxColorValueIndex == RGB_GREEN)
            {
                hue = 2.0 + (float)(rgbValues[RGB_BLUE] - rgbValues[RGB_RED]) / delta;
            }
            else
            {
                hue = 4.0 + (float)(rgbValues[RGB_RED] - rgbValues[RGB_GREEN]) / delta;
            }
        }
        hue *= 60.0;
        if (hue < 0) hue += 360;
        if (hue >= HUE_MAX_VALUE) {
            hue = 0;
        }
        tslColors[TSL_HUE] = (uint16_t)hue;
    }
    // Compute the tsl values
    uint16_t saturation = (uint16_t)((uint16_t)100 * (uint16_t)((maxColorValue - minColorValue)) / (uint16_t)maxColorValue);
    if (saturation >= SATURATION_MAX_VALUE) {
        saturation = 0;
    }
    tslColors[TSL_SATURATION] = saturation;
    tslColors[TSL_LIGHTNESS] = (uint16_t)((uint16_t)100 * maxColorValue) / (uint16_t)255;
}

void color_sensor_values_publish(uint16_t tslColors[3]) {
    color_msg.hue = tslColors[TSL_HUE];
    color_msg.saturation = tslColors[TSL_SATURATION];
    color_msg.lightness = tslColors[TSL_LIGHTNESS];
    if (node_handle_color_sensor) {
        color_pub.publish(&color_msg);
    }
}

void color_sensor_mean_value_compute(uint16_t tslColors[3]) {
    uint16_t tsl_accumulator[3] = {0, 0, 0};
    for(uint8_t counter = 0; counter < COLOR_ACCUMULATE_NB; ++counter) {
        tsl_accumulator[TSL_HUE] += color_sensor_previous_tsl_values[counter][TSL_HUE];
        tsl_accumulator[TSL_SATURATION] += color_sensor_previous_tsl_values[counter][TSL_SATURATION];
        tsl_accumulator[TSL_LIGHTNESS] += color_sensor_previous_tsl_values[counter][TSL_LIGHTNESS];
    }
    tslColors[TSL_HUE] = tsl_accumulator[TSL_HUE] / COLOR_ACCUMULATE_NB;
    tslColors[TSL_SATURATION] = tsl_accumulator[TSL_SATURATION] / COLOR_ACCUMULATE_NB;
    tslColors[TSL_LIGHTNESS] = tsl_accumulator[TSL_LIGHTNESS] / COLOR_ACCUMULATE_NB;
}

#endif
