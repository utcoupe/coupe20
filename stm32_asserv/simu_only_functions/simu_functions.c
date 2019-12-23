#include "simu_functions.h"

control_t control;

int getPWMLeft() {
    return control.speeds.pwm_left;
}

int getPWMRight() {
    return control.speeds.pwm_right;
}