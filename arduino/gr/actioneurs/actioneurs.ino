

#include "consts.h"
#include "AccelStepper.h"

// Including rosserial
#include <ros.h>
#include <game_manager/GameStatus.h>      // ROS sends game and init status (to  determine when RPi ready).
#include <ard_gr_front/RosPucksTake.h>
#include <ard_gr_front/RosArduinoToAI.h>
ros::NodeHandle nh;


int game_status = -1;
int init_status = -1;
bool puckGoesToScale[3];
static int VENTS_TO_PUMPS[3] = {VENT1_TO_PUMP,VENT2_TO_PUMP,VENT3_TO_PUMP};
static int VENTS_TO_AIR[3] = {VENT1_TO_AIR,VENT2_TO_AIR,VENT3_TO_AIR};
//static AccelStepper DOOR_STEPPER =  AccelStepper(uint8_t interface = AccelStepper::FULL4WIRE, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4, 5,false);


void sendROSmsg(int msg) {
    if(msg==AWAITING_ORDER_MSG){

    }else if(msg==PUCKS_SUCKED_UP_MSG){

    }else if(msg==PUCKS_SORTED_MSG){

    }else if(msg==DOOR_RESET_MSG){

    }//else if(msg==TOWER_PUSHED_OUT){

    //}

}
// inutile si la nouvelle version de la tour est confirmée
void elevatorDown() {
    

    // analogWrite(TOWER_TOP_PWM,TOWER_TOP_OPEN_POS)
    // delay(TOWER_TOP_DELAY)
    // analogWrite(TOWER_TOP_PWM,TOWER_TOP_CLOSED_POS)
    // delay(TOWER_TOP_DELAY)
}
// TO DO cartes ponts en H pour pompes
void suckUpPucks() {
    digitalWrite(PUMP, HIGH);
    delay(PNEU_DELAY);
    for(int i = 0; i < 3; i++){
        digitalWrite(VENTS_TO_PUMPS[i], HIGH);
    }
    delay(PNEU_DELAY);  
}

void freePuckToSort(int index) {
    delay(SELECTOR_TIME_TO_MOVE);
    digitalWrite(VENTS_TO_PUMPS[index], LOW);
    digitalWrite(VENTS_TO_AIR[index], HIGH);
    delay(PUCK_TIME_TO_MOVE);
    digitalWrite(VENTS_TO_AIR[index], LOW);
}
void onRaiseAndSortPucks() {
    if (game_status!=1) return;
    door(true);
    analogWrite(SELECTOR_PWM, SELECTOR_SCALE_POS);
    for(int i = 0; i < 3; i++){
         if(puckGoesToScale[i]){
           freePuckToSort(i);
        }
    }
    analogWrite(SELECTOR_PWM, SELECTOR_PILE_POS);
    for(int i = 0; i < 3; i++){
        if(!puckGoesToScale[i]){
            freePuckToSort(i);
            elevatorDown();
        }
    }
    digitalWrite(PUMP, LOW);
    analogWrite(SELECTOR_PWM, SELECTOR_SCALE_POS);
    door(false);
}


void door(bool goUp) {

    // si servomoteur
    const int DOOR_UP_CONST = 255; //TODO :a changer
    const int DOOR_DOWN_CONST = 0; //TODO :a changer

    //front door that raise to take puks to sorting
    int setpoint = goUp?DOOR_UP_CONST:DOOR_DOWN_CONST;
    analogWrite(DOOR_PWM,setpoint);
    delay(DOOR_TIME_TO_MOVE);
    //TO DO : 
}

void onRosDumpPucks() {
    if (game_status!=1) return;
    //int setpoint = goUp?TOWER_CLOSURE_OPEN_POS:TOWER_CLOSURE_OPEN_POS;  //??????????
    int setpoint = TOWER_CLOSURE_OPEN_POS;
    analogWrite(TOWER_CLOSURE_PWM,TOWER_CLOSURE_OPEN_POS);
    delay(TOWER_CLOSURE_DELAY);
    sendROSmsg(TOWER_OPEN);
}



void on_game_status(const game_manager::GameStatus& msg){
    game_status = msg.game_status;
    init_status = msg.init_status;
    //TODO à changer peut etre
}


void onRosTakePucks(const ard_gr_front::RosPucksTake& msg){
    // ,const game_manager::RosPucksTaker2& P2,
    // const game_manager::RosPucksTaker3& P3) {
    nh.loginfo("Take pucks !!!!!!");
    int puckGoesToScale[3] = {msg.P1,msg.P2,msg.P3};//{P1,P2,P3};
    suckUpPucks();
    sendROSmsg(PUCKS_SUCKED_UP_MSG);
}


ros::Subscriber<game_manager::GameStatus> sub_on_raise_sort_pucks("actionneurs/raise_and_sort_pucks",&onRaiseAndSortPucks);
ros::Subscriber<game_manager::GameStatus> sub_dump_pucks("actionneurs/dump_pucks",&onRosDumpPucks);
ros::Subscriber<game_manager::GameStatus> sub_take_pucks("actionneurs/TakePucks",&onRosTakePucks);
ros::Subscriber<game_manager::GameStatus> sub_game_status("ai/game_manager/status",&on_game_status);

//TODO créer event de feedback avec un champs msg.type et un msg.
//drivers_ard_hmi::HMIEvent hmi_event_msg;
//ros::Publisher hmi_event_pub("feedback/ard_actionneurs/", &hmi_event_msg);



void setup() {
    nh.initNode();
    nh.subscribe(sub_game_status);
    nh.subscribe(sub_take_pucks);
    nh.subscribe(sub_dump_pucks);
    nh.subscribe(sub_game_status);
    //nh.advertise(hmi_event_pub);

}


void loop() {

    nh.spinOnce();
    delay(5); // seen on a blog to not overwhelm master
}
