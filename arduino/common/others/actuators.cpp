//
// Created by tfuhrman & Elwan Héry on 29/04/18.
//

#include "actuators.h"
#include "config_actuators.h"
#include <Arduino.h>

//********************************************************************************************************************//
//
// Regulated actuators
//
//********************************************************************************************************************//

#ifdef REGULATED_ACTUATORS_ENABLED

ros::NodeHandle* node_handle_regulated_actuators = NULL;

void interrupt_regulated_actuators();
void reset_internal_values();

unsigned long ticks_counter = 0;
uint8_t regulation_activated = 0;
float regulation_reference_value = 0;
unsigned long last_ticks = 0;
float sum_speed_error = 0.0;
float last_speed_error = 0.0;
unsigned long last_control_time = 0;

void init_regulated_actuators(ros::NodeHandle* nh) {
    node_handle_regulated_actuators = nh;
    pinMode(REGULATED_ACTUATORS_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(REGULATED_ACTUATORS_INTERRUPT_PIN), interrupt_regulated_actuators, RISING);
    pinMode(REGULATED_ACTUATORS_PIN, OUTPUT);
}

void loop_regulated_actuators() {
    unsigned long current_control_time = millis();
    if (regulation_activated > 0) {
        unsigned long delta_time = current_control_time - last_control_time;
        if (delta_time >= REGULATED_ACTUATORS_CONTROL_LOOP_MS) {
            last_control_time = current_control_time;
            float nbr_revolution = (float)((float)(ticks_counter - last_ticks) / float(REGULATED_ACTUATORS_TICS_PER_REVOLUTION));
            float wheel_speed = (float)(nbr_revolution / (float)((float)delta_time / 1000.0));
            last_ticks = ticks_counter;
            float speed_error = regulation_reference_value - wheel_speed;
            sum_speed_error += speed_error;
            float pwm_p = REGULATED_ACTUATORS_PID_P * speed_error;
            if (pwm_p > 255) {
                pwm_p = 255;
            } else if (pwm_p < -255) {
                pwm_p = -255;
            }
            float pwm_i = REGULATED_ACTUATORS_PID_I * sum_speed_error;
            if (pwm_i > 255) {
                pwm_i = 255;
            }
            float pwm_d = REGULATED_ACTUATORS_PID_D * (speed_error - last_speed_error);
            int pwm_to_apply = pwm_p + pwm_i + pwm_d;
            last_speed_error = speed_error;
            if (pwm_to_apply > REGULATED_ACTUATORS_PWM_MAX) {
                pwm_to_apply = REGULATED_ACTUATORS_PWM_MAX;
            } else if (pwm_to_apply < REGULATED_ACTUATORS_PWM_MIN) {
                pwm_to_apply = 0;
            }
            analogWrite(REGULATED_ACTUATORS_PIN, pwm_to_apply);
//            String debug("pwm:" + String(pwm_to_apply) + " spd:" + String(wheel_speed) + " p:" + String(pwm_p) + " i:" + String(pwm_i) + " rev:" + String(nbr_revolution) + " tick:" + String(ticks_counter));
//            node_handle_regulated_actuators->loginfo(debug.c_str());
        }
    } else {
        analogWrite(REGULATED_ACTUATORS_PIN, 0);
    }
}

void activate_regulated_actuators(float reference_value) {
    regulation_reference_value = reference_value;
    regulation_activated = 1;
    reset_internal_values();
}

void deactivate_regulated_actuators() {
    regulation_activated = 0;
    regulation_reference_value = 0;
}

void interrupt_regulated_actuators() {
    ticks_counter++;
}

void reset_internal_values() {
    last_ticks = 0;
    sum_speed_error = 0.0;
    last_speed_error = 0.0;
    last_control_time = 0;
}

#endif