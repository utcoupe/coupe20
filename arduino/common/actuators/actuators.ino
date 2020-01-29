#include <ros.h>
#include <std_msgs/Bool.h>

const char *TOPIC_NAME = "/actuators/take_ecocups";

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Bool> sub_actions(TOPIC_NAME, &on_action);

void activatePumps() {
    digitalWrite(10,HIGH);
    digitalWrite(11,HIGH);
    digitalWrite(12,HIGH);  
}

void deactivatePupms() {
    digitalWrite(10,LOW);
    digitalWrite(11,LOW);
    digitalWrite(12,LOW);
}

void on_action(const std_msgs::Bool &msg) {
    bool activate = msg.data;
    if (activate) {
        activatePumps();
    }
    else {
        deactivatePumps();
    }
}

void setup() {
    // ROS
    nh.initNode();
    nh.subscribe(sub_actions);

    // ARDUINO
    Serial.begin(57600);
    
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);
}

void loop() {
  nh.spinOnce();
}
