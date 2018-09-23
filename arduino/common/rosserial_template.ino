#include <ros.h>
#include <drivers_ard_hmi/SetStrategies.h>
#include <drivers_ard_hmi/HMIEvent.h>
ros::NodeHandle nh;

/*
This is a simple template that shows how to create a subscriber and publisher
with rosserial. This template works with coupe18's system.

Created by Pierre Laclau, UTCoupe 2018.
*/


// ROS methods
void on_set_strategies(const drivers_ard_hmi::SetStrategies& msg){
    int array_count = msg.strategies_names_length;
    for(int i=0; i < msg.strategies_names_length; i++) {
        String a = msg.strategies_names[i];
    }
}

// Subscriber example
ros::Subscriber<drivers_ard_hmi::SetStrategies> sub_strats("/feedback/ard_hmi/set_strategies", &on_set_strategies);

// Publisher example
drivers_ard_hmi::HMIEvent hmi_event_msg;
ros::Publisher hmi_event_pub("/feedback/ard_hmi/hmi_event", &hmi_event_msg);


void setup() {
    nh.initNode();
    nh.subscribe(sub_strats);
    nh.advertise(hmi_event_pub);
}

void loop() {
    // publish example
    hmi_event_msg.event = hmi_event_msg.EVENT_JACK_FIRED;
    hmi_event_msg.chosen_strategy_id = 0; 
    hmi_event_msg.chosen_team_id     = 0;
    hmi_event_pub.publish(&hmi_event_msg);

    // spin Once to sync publishes and subscriptions with ros.h
    nh.spinOnce();
    delay(30);
}