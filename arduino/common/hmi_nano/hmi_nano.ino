//-----------------------------------------------
// Arduino : PR tower 
// Author : Floriane ALLAIRE MAY 2019 , UTCoupe 
// Need PololuA4984.h et PololuA4984.ino and tower_variables.h in folder 
// search for : TODO , DANGER and URGENT 
//------------------------------------------------

// Include rosserial 
#include <ros.h>
#include <game_manager/GameStatus.h>    // ROS sends game and init status (to  determine when RPi ready).
#include <ard_hmi/HMIEvent.h>           // HMI sends events : JACK, GAME_STOP
#include <ard_hmi/ROSEvent.h>           // ROS sends events : ASK_JACK, GAME_STOP
ros::NodeHandle nh ; 

// include variables 
#include "pin.h"


// ----------------------------------------------------------------
// ---------------------------- ROS methods ----------------------- 
// ----------------------------------------------------------------

// ~ Publisher ~ 
ard_hmi::HMIEvent hmi_event_msg;
ros::Publisher hmi_event_pub("feedback/ard_hmi/hmi_event", &hmi_event_msg);

void on_game_status(const game_manager::GameStatus& msg){
    game_status = msg.game_status; 
    init_status = msg.init_status ; 
}


void on_ros_event(const ard_hmi::ROSEvent& msg){
    if(msg.event == 0) //asked to respond for JACK
        ask_jack_status = 1 ; 
}


// ~ Subscriber ~ 
ros::Subscriber<game_manager::GameStatus>  sub_game_status  ("ai/game_manager/status", &on_game_status)  ;
ros::Subscriber<ard_hmi::ROSEvent>         sub_ros_events   ("feedback/ard_hmi/ros_event", &on_ros_event);

// --------------------------------------------------------------
// --------------------- FUNCTIONS ------------------------------ 
// --------------------------------------------------------------
void chosen_stratety_team() {
    nh.loginfo("strategy_chosen"); 
    first_msg = 2 ; 
    nh.loginfo("chosen_stratety_team"); 
    hmi_event_msg.event = 1 ; 
    hmi_event_msg.chosen_strategy_id = chosen_strategy_id ; 
    chosen_team_id = digitalRead(button_team); 
    hmi_event_msg.chosen_team_id     = chosen_team_id ; 
    hmi_event_pub.publish(&hmi_event_msg);
}

void msg_intialize() {
    nh.loginfo("msg_init"); 
    first_msg = 1 ; 
    hmi_event_msg.event = 0 ;   
    hmi_event_pub.publish(&hmi_event_msg);  // send msg to tell that it's been initialize with ROS
    delay(1000); 
}


void update_leds() {
    digitalWrite(PIN_LED_ALIVE, game_status != -1);

    if(init_status == -1) {
      digitalWrite(PIN_LED_INIT, LOW);
    }
    else {
        if(init_status == 0) // some nodes have not responded yet
            analogWrite(PIN_LED_INIT, 127 * (1 + cos(float(millis()) / 200.0))); // wave
        else if(init_status == 1) // init finished and all nodes are ready
            analogWrite(PIN_LED_INIT, 255); //solid ON
        else if(init_status == 2) // init finished but at least one node failed
            analogWrite(PIN_LED_INIT, 255 * (millis() % 1000 < 700)); // blink
     }


    if (jack_state == 1 ) 
        digitalWrite(PIN_LED_JACK,HIGH);
    else {
        if (jack_state == -1 ) 
          analogWrite(PIN_LED_JACK, 127 * (1 + cos(float(millis()) / 200.0))); // jack message not arrived yet 
        else 
          digitalWrite(PIN_LED_JACK,LOW);
    }
}

void ask_jack() {
  nh.loginfo("ask_jack"); 
    if (  digitalRead(jackPin) == HIGH ) { 
        ask_jack_status = 0 ; // jack received 
        jack_state = 1 ; 
    }
    else {
        ask_jack_status = 1 ; // ask for jack 
        jack_state = 0 ; 
    } 
}

void jack_pulled() {
     if ( digitalRead(jackPin) == 0) { // jack pulled 
        nh.loginfo("jack_pulled"); 
        hmi_event_msg.event = 2 ;   
        hmi_event_pub.publish(&hmi_event_msg);
        jack_state = 1 ; 
     }
}

// ----------------------------------------------------------------------
// --------------------------- MAIN -------------------------------------
// ----------------------------------------------------------------------



void setup() {
    // ROS init 
    nh.initNode();
    nh.subscribe(sub_game_status);
    nh.subscribe(sub_ros_events);
    nh.advertise(hmi_event_pub);

    // Led init 
    pinMode(PIN_LED_ALIVE, OUTPUT);
    pinMode(PIN_LED_INIT,  OUTPUT); 
    pinMode(PIN_LED_JACK,  OUTPUT); 

    // Switch init 
    pinMode(button_team, INPUT);
    pinMode(jackPin,     INPUT); 
}

void loop() {
    update_leds() ; 
    if (first_msg == -1 && game_status != -1 )      msg_intialize() ; 
    if (first_msg == 1 && game_status != -1   )     chosen_stratety_team() ; 
    if (ask_jack_status == 1 && game_status != -1 ) ask_jack() ; 
    if (ask_jack_status == 0 && game_status != -1)  jack_pulled() ; 
    nh.spinOnce() ; 
}
