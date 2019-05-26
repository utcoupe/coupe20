#include "consts.h"
#include "PololuA4983.h"
#include <Servo.h>

#include <ros.h>
#include <game_manager/GameStatus.h> 
#include <ard_gr_front/PucksTake.h>
#include <ard_gr_front/PucksRaiseSort.h>
#include <ard_gr_front/RaiseScaleDoor.h>
#include <ard_gr_front/RaiseTower.h>
#include <ard_gr_front/ArduinoToAI.h>


ros::NodeHandle nh;
PololuA4983 stepper_pucks_door = PololuA4983(PUCKS_DOOR_STEP_PIN, PUCKS_DOOR_DIR_PIN,
                                            PUCKS_DOOR_EN_PIN, PUCKS_DOOR_MIN_DELAY);
Servo selector;

int game_status = GAME_OFF;
int pucks_door_steps_taken = 0;

bool puck_to_scale[3];
static int PUMP[3] = {PUMP_1,PUMP_2,PUMP_3};


// Publisher 
ard_gr_front::ArduinoToAI  event_msg ; 
ros::Publisher pub_response  ("actionneurs/ard_gr_front/event", &event_msg); 

void publish_response (int event_type, bool success){
    event_msg.type = event_type;
    event_msg.success = success;
    pub_response.publish(&event_msg);
}

void initialize(){
    selector.write(SELECTOR_TO_TOWER);
    stepper_pucks_door.disable();
}

void on_game_status(const game_manager::GameStatus& msg){
    game_status = GAME_ON;//msg.game_status;

    if (msg.init_status == 1 && game_status == GAME_ON)
        initialize(); 
}
  
void suck_up_pucks() {
    digitalWrite(PUMP_ENABLE, HIGH);
    delay(PUMP_DELAY);
    for(int i = 0; i < 3; i++){
        digitalWrite(PUMP[i], HIGH);
    }
    delay(PUMP_DELAY);  
}

void on_take_pucks(const ard_gr_front::PucksTake& msg){
    //nh.loginfo("Taking pucks...");
    if (game_status != GAME_ON){
        publish_response(EVENT_PUCKS_TAKE, true);
        return;
    }
    
    puck_to_scale[0] = msg.P1;
    puck_to_scale[1] = msg.P2;
    puck_to_scale[2] = msg.P3;

    suck_up_pucks();
}

void free_puck_to_sort(int i) {
    digitalWrite(PUMP[i], LOW);
    delay(PUCK_TIME_TO_MOVE);
}

void pucks_door_goes_up(bool go_up) {
    //nh.loginfo("Pucks door going up/down...");
    stepper_pucks_door.enable();
    if (go_up)
        stepper_pucks_door.moveStep(PUCKS_DOOR_STEP_NB, true);
    else
        stepper_pucks_door.moveStep(PUCKS_DOOR_STEP_NB, false);
    
    while(stepper_pucks_door.getRemainingStep() != 0 && game_status == GAME_ON) {
        stepper_pucks_door.update();

    }
    
    stepper_pucks_door.disable();
    //nh.loginfo("Pucks door has moved");
}

void dump_in_scale(){
    selector.write(SELECTOR_TO_SCALE);
    delay(SELECTOR_TIME_TO_MOVE);

    for(int i = 0; i < 3; i++){
        if(puck_to_scale[i])
           free_puck_to_sort(i);  

    }
}

void dump_in_tower(){
    selector.write(SELECTOR_TO_TOWER);
    for(int i = 0; i < 3; i++){
        if(!puck_to_scale[i])
            free_puck_to_sort(i);
    }
}

void on_raise_and_sort_pucks(const ard_gr_front::PucksRaiseSort& msg) {
    //nh.loginfo("Raising and sorting...");

    if (game_status != GAME_ON){
        publish_response(EVENT_PUCKS_RAISE_SORT, true);
        return;
    }

    pucks_door_goes_up(true);
    
    dump_in_tower();
    dump_in_scale();
    

    digitalWrite(PUMP_ENABLE, LOW);

    pucks_door_goes_up(false);

    publish_response(EVENT_PUCKS_RAISE_SORT, true);
}


void on_raise_scale_door(const ard_gr_front::RaiseScaleDoor& msg){
    //nh.loginfo("Raising scale door...");
    if (game_status != GAME_ON){
        publish_response(EVENT_RAISE_SCALE_DOOR, false);
        return; 
    }
    digitalWrite(SCALE_DOOR_PIN, HIGH);
    while (digitalRead(SCALE_DOOR_LIMIT_PIN)!= HIGH) {
        delay(50);
    }
    digitalWrite(SCALE_DOOR_PIN, LOW);
    
    publish_response(EVENT_RAISE_SCALE_DOOR, true);
}   

void on_raise_tower(const ard_gr_front::RaiseTower& msg){
    //nh.loginfo("Raising tower....");
    if (game_status != GAME_ON){
        publish_response(EVENT_RAISE_TOWER, false);
        return;
    }

    digitalWrite(TOWER_PIN, HIGH);
    while (digitalRead(TOWER_LIMIT_PIN)!= HIGH) {   
        delay(50);
    }
    digitalWrite(TOWER_PIN, LOW);
    
    publish_response(EVENT_RAISE_TOWER, true);
}


//Subscribers
ros::Subscriber<game_manager::GameStatus>      sub_game_status("ai/game_manager/status",&on_game_status);

ros::Subscriber<ard_gr_front::PucksRaiseSort>  sub_raise_sort_pucks("actionneurs/raise_and_sort_pucks",&on_raise_and_sort_pucks);
ros::Subscriber<ard_gr_front::PucksTake>       sub_take_pucks("actionneurs/take_pucks",&on_take_pucks);

ros::Subscriber<ard_gr_front::RaiseScaleDoor>  sub_raise_scale_door("actionneurs/raise_scale_door", &on_raise_scale_door);
ros::Subscriber<ard_gr_front::RaiseTower>      sub_raise_tower("actionneurs/raise_tower", &on_raise_tower);


void connectPins(){

    pinMode(PUMP_ENABLE, OUTPUT);
    pinMode(PUMP_1, OUTPUT);
    pinMode(PUMP_2, OUTPUT);
    pinMode(PUMP_3, OUTPUT);

    pinMode(PUCKS_DOOR_DIR_PIN, OUTPUT);
    pinMode(PUCKS_DOOR_STEP_PIN, OUTPUT);
    pinMode(PUCKS_DOOR_EN_PIN, OUTPUT);

    pinMode(SCALE_DOOR_PIN, OUTPUT);
    pinMode(TOWER_PIN, OUTPUT);

    pinMode(SCALE_DOOR_LIMIT_PIN, INPUT);
    pinMode(TOWER_LIMIT_PIN, INPUT);

    selector.attach(SELECTOR_PIN);

}

void connectROS(){
    nh.initNode();

    nh.subscribe(sub_game_status);
    
    nh.subscribe(sub_take_pucks);
    nh.subscribe(sub_raise_sort_pucks);

    nh.subscribe(sub_raise_scale_door);
    nh.subscribe(sub_raise_tower);
    nh.advertise(pub_response);

}
void setup(){
    connectPins();
    connectROS();
}


void loop(){
    nh.spinOnce();
}
