//ROS includes
#include <ros.h>

//Arduino includes
#include "Arduino.h"
#include <Timer.h>
#include <Wire.h>

//Sensors includes
#include "sensors.h"
#include "config_sensors.h"

#include "config_robots.h"

ros::NodeHandle nh;

// ---- SENSORS ----

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

// ---- MAIN FUCNTIONS ----

void setup() {
    // ROS init
    nh.initNode();

    // Libs init
    Wire.begin();

    // Components init
    init_sensors();

    nh.loginfo("Node '/arduinos/others' initialized correctly.");
}

void loop() {
    // Components loop
    sensors_loop_timer.Update();

    // ROS loop
    nh.spinOnce();
}
