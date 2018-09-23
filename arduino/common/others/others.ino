//ROS includes
#include <ros.h>

//Arduino includes
#include "Arduino.h"
#include <Timer.h>
#include <Wire.h>

//Sensors includes
#include "sensors.h"
#include "config_sensors.h"

//Actuators includes
#include "Servo.h"
#include "AFMotor.h"
#include <drivers_ard_others/Move.h>
#include <drivers_ard_others/MoveResponse.h>
#include <drivers_ard_others/ActDigitalStates.h>
#include <drivers_ard_others/ActPWMStates.h>
#include <drivers_ard_others/ActServoStates.h>
#include "actuators.h"
#include "config_actuators.h"

#include "config_robots.h"

ros::NodeHandle nh;

// ---- SENSORS DEPARTMENT ----

void loop_sensors() {
#ifdef SENSOR_BELT_ENABLED
    loop_belt_sensors();
#endif
#ifdef SENSOR_COLOR_ENABLED
    color_sensor_loop();
#endif
}

Timer sensors_loop_timer = Timer(50, &loop_sensors);

void init_sensors() {
#ifdef SENSOR_BELT_ENABLED
    init_belt_sensors(&nh);
#endif
#ifdef SENSOR_COLOR_ENABLED
    color_sensor_init(&nh);
#endif
    sensors_loop_timer.Start();
}

// ---- ACTUATORS DEPARTMENT ----

extern const uint8_t pins_digital_actuators[];
extern bool digital_actuators_states[];

extern const uint8_t pins_pwm_actuators_pwm[];
extern uint8_t pwm_actuators_states[];

extern const uint8_t pins_servo_actuators_pwm[];
extern int16_t servo_actuators_states[];
Servo servo_actuators_objects[NUM_SERVO_ACTUATORS];

extern int16_t stepper_actuators_states[];
AF_Stepper stepper_actuators_objects[NUM_STEPPER_ACTUATORS];

// Actuators ROS callbacks

drivers_ard_others::MoveResponse move_response_msg;
ros::Publisher move_responses_pub("/drivers/ard_others/move_response", &move_response_msg);

void send_move_response(int16_t order_nb, bool success) {
    if(success) nh.loginfo("Move request succeeded.");
    else nh.logerror("Move request failed.");

    move_response_msg.order_nb = order_nb;
    move_response_msg.success = success;
    move_responses_pub.publish(&move_response_msg);
}

void on_move(const drivers_ard_others::Move& msg){
    bool success = true;

    switch(msg.type) {
        case msg.TYPE_DIGITAL:
            if(msg.id >= 0 && msg.id <= NUM_DIGITAL_ACTUATORS) {
                if(msg.dest_value == 0 || msg.dest_value == 1)
                    digital_actuators_states[msg.id] = bool(msg.dest_value);
                else {
                    nh.logerror("MOVE failed : dest_value invalid (0 or 1).");
                    success = false;
                }
            } else {
                nh.logerror("MOVE failed : invalid id.");
                success = false;
            }
            break;
        case msg.TYPE_PWM:
            if(msg.id >= 0 && msg.id <= NUM_PWM_ACTUATORS) {
                if(msg.dest_value >= 0 && msg.dest_value <= 255) {
                    pwm_actuators_states[msg.id] = msg.dest_value;
#ifdef REGULATED_ACTUATORS_ENABLED
                    if (msg.dest_value == 0) {
                        deactivate_regulated_actuators();
                        nh.loginfo("Deactivate regulation");
                    } else {
                        activate_regulated_actuators(msg.dest_value);
                        nh.loginfo("Activate regulation");
                    }
#endif
                }
                else {
                    nh.logerror("MOVE failed : dest_value invalid (0 to 255).");
                    success = false;
                }
            } else {
                nh.logerror("MOVE failed : invalid id.");
                success = false;
            }
            break;
        case msg.TYPE_SERVO:
            if(msg.id >= 0 && msg.id <= NUM_SERVO_ACTUATORS) {
                if(msg.dest_value >= 0 && msg.dest_value <= 180)
                    servo_actuators_states[msg.id] = msg.dest_value;
                else {
                    nh.logerror("MOVE failed : dest_value invalid (0 to 180).");
                    success = false;
                }
            } else {
                nh.logerror("MOVE failed : invalid id.");
                success = false;
            }
            break;
        case msg.TYPE_STEPPER:
            if(msg.id >= 0 && msg.id <= NUM_STEPPER_ACTUATORS) {
                stepper_actuators_states[msg.id] = msg.dest_value;
                if (msg.dest_value == 0) {
                    stepper_actuators_objects[msg.id].release();
                }
                if (msg.dest_value < 0) {
                    stepper_actuators_objects[msg.id].setSpeed(120);
                    stepper_actuators_objects[msg.id].step(-msg.dest_value, FORWARD, SINGLE);
                }
                else {
                    stepper_actuators_objects[msg.id].setSpeed(120);
                    stepper_actuators_objects[msg.id].step(msg.dest_value, BACKWARD, SINGLE);
                }
                success = true;
            } else {
                nh.logerror("MOVE failed : invalid id.");
                success = false;
            }
            break;
    }
    nh.loginfo("Finished move order handling.");
    if(msg.order_nb != 0) send_move_response(msg.order_nb, success); // send response if order_nb provided.
}

ros::Subscriber<drivers_ard_others::Move> sub_move("/drivers/ard_others/move", &on_move);

drivers_ard_others::ActDigitalStates digital_states_msg;
ros::Publisher digital_states_pub("/drivers/ard_others/digital_act_states", &digital_states_msg);
drivers_ard_others::ActPWMStates pwm_states_msg;
ros::Publisher pwm_states_pub("/drivers/ard_others/pwm_act_states", &pwm_states_msg);
drivers_ard_others::ActServoStates servo_states_msg;
ros::Publisher servo_states_pub("/drivers/ard_others/servo_act_states", &servo_states_msg);
drivers_ard_others::ActServoStates stepper_states_msg;
ros::Publisher stepper_states_pub("/drivers/ard_others/stepper_act_states", &stepper_states_msg);

// Digital actuators
void init_digital_actuators() {
    for(uint8_t i = 0; i < NUM_DIGITAL_ACTUATORS; i++) {
        pinMode(pins_digital_actuators[i], OUTPUT);
    }
}

void loop_digital_actuators() {
    for(uint8_t i = 0; i < NUM_DIGITAL_ACTUATORS; i++)
        digitalWrite(pins_digital_actuators[i], digital_actuators_states[i]);

    digital_states_msg.states_length = NUM_DIGITAL_ACTUATORS;
    digital_states_msg.states = digital_actuators_states;
    digital_states_pub.publish(&digital_states_msg);
}

// PWM actuators
void init_pwm_actuators() {
    for(uint8_t i = 0; i < NUM_PWM_ACTUATORS; i++) {
#ifdef REGULATED_ACTUATORS_ENABLED
        init_regulated_actuators(&nh);
#else
        pinMode(pins_pwm_actuators_pwm[i], OUTPUT);
#endif
    }
}

void loop_pwm_actuators() {
    for(uint8_t i = 0; i < NUM_PWM_ACTUATORS; i++) {
#ifdef REGULATED_ACTUATORS_ENABLED
        loop_regulated_actuators();
#else
        analogWrite(pins_pwm_actuators_pwm[i], pwm_actuators_states[i]);
#endif
    }

    pwm_states_msg.states_length = NUM_PWM_ACTUATORS;
    pwm_states_msg.states = pwm_actuators_states;
    pwm_states_pub.publish(&pwm_states_msg);
}

// Servo actuators
void init_servo_actuators() {
    for(uint8_t i = 0; i < NUM_SERVO_ACTUATORS; i++) {
        pinMode(pins_digital_actuators[i], OUTPUT);
        servo_actuators_objects[i].attach(pins_servo_actuators_pwm[i]);
    }
}

void loop_servo_actuators() {
    for(uint8_t i = 0; i < NUM_SERVO_ACTUATORS; i++)
        servo_actuators_objects[i].write(servo_actuators_states[i]);

    servo_states_msg.states_length = NUM_SERVO_ACTUATORS;
    servo_states_msg.states = servo_actuators_states;
    servo_states_pub.publish(&servo_states_msg);
}

// Stepper actuators
void init_stepper_actuators() {
    for(uint8_t i = 0; i < NUM_STEPPER_ACTUATORS; i++) {
        //TODO better initialisation with different stepper_number
        stepper_actuators_objects[i].init(200, 1);
        stepper_actuators_objects[i].setSpeed(60);
    }
}

void loop_stepper_actuators() {
    stepper_states_msg.states_length = NUM_STEPPER_ACTUATORS;
    stepper_states_msg.states = stepper_actuators_states;
    stepper_states_pub.publish(&stepper_states_msg);
}

void loop_actuators() {
    loop_digital_actuators();
    loop_pwm_actuators();
    loop_servo_actuators();
    loop_stepper_actuators();
}

Timer actuators_loop_timer = Timer(50, &loop_actuators);

void init_actuators() {
    init_digital_actuators();
    init_pwm_actuators();
    init_servo_actuators();
    init_stepper_actuators();
    actuators_loop_timer.Start();
}

// ---- MAIN FUCNTIONS ----

void setup() {
    // ROS init
    nh.initNode();

    nh.subscribe(sub_move);
    nh.advertise(move_responses_pub);
    nh.advertise(digital_states_pub);
    nh.advertise(pwm_states_pub);
    nh.advertise(servo_states_pub);
    nh.advertise(stepper_states_pub);

    // Libs init
    Wire.begin();

    // Components init
    init_sensors();
    init_actuators();

    nh.loginfo("Node '/arduinos/others' initialized correctly.");
}

void loop() {
    // Components loop
    sensors_loop_timer.Update();
    actuators_loop_timer.Update();

    // ROS loop
    nh.spinOnce();
}
