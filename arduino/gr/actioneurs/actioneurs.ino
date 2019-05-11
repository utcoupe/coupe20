#include "consts.h"
//#include "AccelStepper.h"
#include "PoluluA4983.h"

// Including rosserial
#include <ros.h>
#include <game_manager/GameStatus.h>      // ROS sends game and init status (to  determine when RPi ready).
#include <ard_gr_front/PucksTake.h>
#include <ard_gr_front/PucksDump.h>
#include <ard_gr_front/PucksRaiseSort.h>
#include <ard_gr_front/ArduinoToAI.h>

#include <ard_gr_front/MoveBackDoor.h>
#include <ard_gr_front/MoveScaleDoor.h>
#include <ard_gr_front/MoveTower.h>

ros::NodeHandle nh;

int game_status = -1;
int init_status = -1;
bool puckGoesToScale[3];
static int VENTS_TO_PUMPS[3] = {VENT1_TO_PUMP,VENT2_TO_PUMP,VENT3_TO_PUMP};
static int VENTS_TO_AIR[3] = {VENT1_TO_AIR,VENT2_TO_AIR,VENT3_TO_AIR};
//Stepper motors
PoluluA4983 stepper_tower = PoluluA4983(TOWER_STEP_PIN, TOWER_DIR_PIN, TOWER_EN_PIN, TOWER_MIN_DELAY)
PoluluA4983 stepper_scale_door = PoluluA4983(SCALE_DOOR_STEP_PIN, SCALE_DOOR_DIR_PIN, 
                                            SCALE_DOOR_EN_PIN, SCALE_DOOR_MIN_DELAY)

void send_ros_msg(int msg) {
    if(msg==AWAITING_ORDER_MSG){

    }else if(msg==PUCKS_SUCKED_UP_MSG){

    }else if(msg==PUCKS_SORTED_MSG){

    }else if(msg==PUCKS_DOOR_RESET_MSG){

    }//else if(msg==TOWER_PUSHED_OUT){

    //}

}

// TO DO cartes ponts en H pour pompes
void suck_up_pucks() {
    digitalWrite(PUMP, HIGH);
    delay(PNEU_DELAY);
    for(int i = 0; i < 3; i++){
        digitalWrite(VENTS_TO_PUMPS[i], HIGH);
    }
    delay(PNEU_DELAY);  
}

void free_puck_to_sort(int index) {
    delay(SELECTOR_TIME_TO_MOVE);
    digitalWrite(VENTS_TO_PUMPS[index], LOW);
    digitalWrite(VENTS_TO_AIR[index], HIGH);
    delay(PUCK_TIME_TO_MOVE);
    digitalWrite(VENTS_TO_AIR[index], LOW);
}
void on_raise_and_sort_pucks() {
    if (game_status!=1) return;
    pucks_door_goes_up(true);
    analogWrite(SELECTOR_PWM, SELECTOR_SCALE_POS);
    for(int i = 0; i < 3; i++){
         if(puckGoesToScale[i]){
           free_puck_to_sort(i);
        }
    }
    analogWrite(SELECTOR_PWM, SELECTOR_PILE_POS);
    for(int i = 0; i < 3; i++){
        if(!puckGoesToScale[i]){
            free_puck_to_sort(i);
        }
    }
    digitalWrite(PUMP, LOW);
    analogWrite(SELECTOR_PWM, SELECTOR_SCALE_POS);
    pucks_door_goes_up(false);
}


void pucks_door_goes_up(bool goUp) {

    // si servomoteur
    const int PUCKS_DOOR_UP_CONST = 255; //TODO :a changer
    const int PUCKS_DOOR_DOWN_CONST = 0; //TODO :a changer

    //front pucks_door_goes_up that raise to take puks to sorting
    int setpoint = goUp?PUCKS_DOOR_UP_CONST:PUCKS_DOOR_DOWN_CONST;
    analogWrite(PUCKS_DOOR_PWM,setpoint);
    delay(PUCKS_DOOR_TIME_TO_MOVE);
    //TO DO : 
}

void on_dump_pucks() {
    if (game_status!=1) return;
    //int setpoint = goUp?TOWER_CLOSURE_OPEN_POS:TOWER_CLOSURE_OPEN_POS;  //??????????
    int setpoint = TOWER_CLOSURE_OPEN_POS;
    analogWrite(TOWER_CLOSURE_PWM,TOWER_CLOSURE_OPEN_POS);
    delay(TOWER_CLOSURE_DELAY);
    send_ros_msg(TOWER_OPEN);
}



void on_game_status(const game_manager::GameStatus& msg){
    game_status = msg.game_status;
    init_status = msg.init_status;
    //TODO maybe change ?
}


void on_take_pucks(const ard_gr_front::PucksTake& msg){
    int puckGoesToScale[3] = {msg.P1,msg.P2,msg.P3};//{P1,P2,P3};
    suck_up_pucks();
    send_ros_msg(PUCKS_SUCKED_UP_MSG);
}

void on_move_scale_door(const ard_gr_front::MoveScaleDoor& msg){
    if (msg.door_status == SCALE_DOOR_CLOSE) {
        stepper_scale_door.moveStep(SCALE_DOOR_CLOSE_POS, true);
        while(stepper_scale_door.getRemainingStep() >0 ) {
            stepper_scale_door.update() ; 
    }
    }
    if (msg.door_status == SCALE_DOOR_OPEN){
        stepper_scale_door.moveStep(SCALE_DOOR_OPEN_POS, true);
        while(stepper_scale_door.getRemainingStep() >0 ) {
            stepper_scale_door.update() ; 
    }
}

void on_move_tower(const ard_gr_front::MoveTower& msg){
    if (msg.tower_status == TOWER_DOWN) {
        stepper_tower.moveStep(SCALE_DOOR_OPEN_POS, true);
        while(stepper_tower.getRemainingStep() >0 ) {
            stepper_tower.update() ; 

    }
    if (msg.tower_status == TOWER_UP){
        stepper_tower.moveStep(SCALE_DOOR_OPEN_POS, true);
        while(stepper_tower.getRemainingStep() >0 ) {
            stepper_tower.update() ; 
    }
}

ros::Subscriber<game_manager::GameStatus>     sub_game_status("ai/game_manager/status",&on_game_status);

ros::Subscriber<ard_gr_front::PucksRaiseSort> sub_raise_sort_pucks("actionneurs/raise_and_sort_pucks",&on_raise_and_sort_pucks);
ros::Subscriber<ard_gr_front::PucksDump>      sub_dump_pucks("actionneurs/dump_pucks",&on_dump_pucks);
ros::Subscriber<ard_gr_front::PucksTake>      sub_take_pucks("actionneurs/take_pucks",&on_take_pucks);

ros::Subscriber<ard_gr_front::MoveScaleDoor>  sub_move_scale_door("actionneurs/move_scale_door", &on_move_scale_door);
ros::Subscriber<ard_gr_front::MoveTower>      sub_move_tower("actionneurs/move_tower", &on_move_tower);

//TODO créer event de feedback avec un champs msg.type et un msg.
//drivers_ard_hmi::HMIEvent hmi_event_msg;
//ros::Publisher hmi_event_pub("feedback/ard_actionneurs/", &hmi_event_msg);



void setup() {
    nh.initNode();
    nh.subscribe(sub_game_status);
    
    nh.subscribe(sub_take_pucks);
    nh.subscribe(sub_dump_pucks);
    nh.subscribe(sub_raise_sort_pucks);

    nh.subscribe(sub_move_scale_door);
    nh.subscribe(sub_move_tower);
    //nh.advertise(hmi_event_pub);

}


void loop() {

    nh.spinOnce();
    delay(5); // seen on a blog to not overwhelm master
}
