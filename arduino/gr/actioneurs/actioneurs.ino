

#include "consts.h"
#include "AccelStepper.h"

// Including rosserial
#include <ros.h>
#include <ai_game_manager/GameStatus.h>      // ROS sends game and init status (to  determine when RPi ready).
ros::NodeHandle nh;


int game_status = -1;
int init_status = -1;

// code de swan


/*
Sequence diagram:https://www.websequencediagrams.com

title ventouse
ROS->Arduino: Game On
note right of ROS: initialisation sensor
Arduino->MotoPompe:High
Arduino->VENT1:High
Arduino->VENT2:High
Arduino->VENT3:High
Arduino->MotoPompe:Low
Arduino->VENT1:Low
Arduino->VENT2:Low
Arduino->VENT3:Low
Arduino->SELECT:goPile
Arduino->SELECT:goScale
Arduino->ASS:GoUP
ASSSensor->Arduino:AssUP
Arduino->ROS: Waiting
note right of ROS: FOR PICKING UP PUCKS
ROS->Arduino: Take pucks [Scale; ASS; ASS] \n (direction on witch to sort the pucks)
Arduino->Arduino: setPucksDest
Arduino->MotoPompe:Start
Arduino->VENT1:High
Arduino->VENT2:High
Arduino->VENT3:High
Arduino->Arduino: delay for time to take pucks
Arduino->ROS: pucksTook
Arduino->Arduino: delay for time to go backwards
Arduino->ROS: pucksTook
Arduino->RisingBridge:GoUP
Arduino->SELECT:goPos1
Arduino->VENT1:Low
Arduino->Arduino: delay for time for pucks to go down
Arduino->ASS: if Pos == ASS : assDown
Arduino->SELECT:goPos2
Arduino->VENT2:Low
Arduino->Arduino: delay for time for pucks to go down
Arduino->ASS: if Pos == ASS : assDown
Arduino->SELECT:goPos3
Arduino->VENT3:Low
Arduino->Arduino: delay for time for pucks to go down
Arduino->ASS: if Pos == ASS : assDown
Arduino->MotoPompe:Stop
Arduino->RisingBridge:GoDOWN
*/

// PINS


//messages to ROS
bool puckGoesToScale[3];
static final int VENTS_TO_PUMPS = {VENT1_TO_PUMP,VENT2_TO_PUMP,VENT3_TO_PUMP}
static final int VENTS_TO_AIR = {VENT1_TO_AIR,VENT2_TO_AIR,VENT3_TO_AIR}


void sendROSmsg(int msg) {
    if(msg == AWAITING_ORDER_MSG){

    }else if(msg == PUCKS_SUCKED_UP_MSG){

    }else if(msg == PUCKS_SORTED_MSG){

    }else if(msg == DOOR_RESET_MSG){

    }else if(msg == TOWER_PUSHED_OUT){

    }
    
}
void suckUpPucks() {
    digitalWrite(PUMP, HIGH);
    delay(PNEU_DELAY);
    for(int i = 0; i < 3; i++){
        digitalWrite(VENTS_TO_PUMP[i], HIGH);
    }
    delay(PNEU_DELAY);  
}
void raiseAndSortPucks() {
    door(true);
    for(int i = 0; i < 3; i++){
        if(puckGoesToScale[i]){
            analogWrite(SELECTOR_PWM, SELECTOR_SCALE_POS);
        }else{
            analogWrite(SELECTOR_PWM, SELECTOR_PILE_POS);
        }
        delay(SELECTOR_TIME_TO_MOVE);
        digitalWrite(VENTS_TO_PUMP[i], LOW);
        digitalWrite(VENTS_TO_AIR[i], HIGH);
        delay(PUCK_TIME_TO_MOVE);
        digitalWrite(VENTS_TO_AIR[i], LOW);
        if(!puckGoesToScale[i]){
            elevatorDown();
        }
    }
    digitalWrite(PUMP, LOW);
    door(false);
}
void door(bool goUp) {
    int setpoint = goUp?DOOR_UP_CONST:DOOR_DOWN_CONST;
    analogWrite(setpoint)
    delay(DOOR_TIME_TO_MOVE)
    //TO DO : 

}
void elevatorDown() {
    analogWrite(TOWER_TOP_PWM,TOWER_TOP_OPEN_POS)
    delay(TOWER_TOP_DELAY)
    analogWrite(TOWER_TOP_PWM,TOWER_TOP_CLOSED_POS)
    delay(TOWER_TOP_DELAY)
}
void onRosDumpPucks() {
    analogWrite(TOWER_CLOSURE_PWM,TOWER_CLOSURE_OPEN_POS)
    delay(TOWER_CLOSURE_DELAY)
    analogWrite(TOWER_CLOSURE_PWM,TOWER_CLOSURE_CLOSED_POS)
    delay(TOWER_CLOSURE_DELAY)
    sendROSmsg(TOWER_PUSHED_OUT)
}
ros::Subscriber<ai_game_manager::GameStatus> sub_game_status("ai/game_manager/status",     &on_game_status);



void on_game_status(const ai_game_manager::GameStatus& msg){
    game_status = msg.game_status;
    init_status = msg.init_status;
    //TODO à changer peut etre
}
ros::Subscriber<ai_game_manager::GameStatus> sub_game_status("ai/game_manager/status",     &on_game_status);

void onRosTakePucks(const ai_game_manager::RosPucksTake& msg){
    // ,const ai_game_manager::RosPucksTaker2& P2,
    // const ai_game_manager::RosPucksTaker3& P3) {
    puckGoesToScale = {msg.P1,msg.P2,msg.P3};//{P1,P2,P3};
    suckUpPucks();
    sendROSmsg(PUCKS_SUCKED_UP_MSG);
}
ros::Subscriber<ai_game_manager::GameStatus>    sub_game_status("ai/game_manager/status",     &onRosTakePucks);





void setup() {
    nh.initNode();
    nh.subscribe(sub_game_status);
    nh.subscribe(onRosTakePucks);
    nh.subscribe(onRosDumpPucks);
    nh.subscribe(sub_game_status);
    nh.advertise(hmi_event_pub);
}


void loop() {
    if (game_status == 2) while (true);
    nh.spinOnce();
}





