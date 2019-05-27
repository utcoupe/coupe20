//-----------------------------------------------
// Arduino : PR door 
// Author : Floriane ALLAIRE Feb 2019 , UTCoupe 
// Need door_variables.h 
// Very for TODO, URGENT and DANGER
//------------------------------------------------

// -----------------------------------------------------------
// --------------------- Includes ----------------------------
// -----------------------------------------------------------

//ROS includes
#include <ros.h> 
#include <game_manager/GameStatus.h>
#include <ard_door/Door.h> 

ros :: NodeHandle nh ; 

// Actuators includes 
#include <Servo.h> 

// Code includes 
#include "door_variables.h"


// Servo motor 
Servo servo_door_1 ; // create servo object to control a servo
Servo servo_door_2 ;
Servo servo_flipper_1 ; 
Servo servo_flipper_2 ;  

// ----------------------------------------------------------------
// ---------------------------- ROS methods ----------------------- 
// ----------------------------------------------------------------
void on_game_status(const game_manager::GameStatus& msg){
     //game_status = msg.game_status; // DANGER game status for test 
}

void on_door(const ard_door::Door& msg){
    door_status     = msg.door_status ; 
    flipper_status  = msg.flipper_status ; 
    sucker_status   = msg.sucker_status  ; 
    if ( msg.init_status == 1 && game_status == 1 ) {
        door_initialize(); 
        flipper_initialize(); 
        sucker_initialize() ; 
    }
}


// ~ Subscriber ~ 
ros::Subscriber<game_manager::GameStatus> sub_game_status  ("ai/game_manager/status", &on_game_status)  ;
ros::Subscriber<ard_door::Door>           sub_door         ("actuators/ard_door",     &on_door); 

// ~ Publisher ~   // None here 

// --------------------------------------------------------------
// --------------------- FUNCTIONS ------------------------------ 
// --------------------------------------------------------------

void door_initialize() { 
    nh.loginfo("door_initialize"); 
    servo_door_1.write(POS_DOOR_1_CLOSE); 
    servo_door_2.write(POS_DOOR_2_CLOSE); 
    digitalWrite(sucker_pin_1,HIGH); 
    door_status = 0 ;
}

void flipper_initialize() {
    nh.loginfo("flipper_initialize"); 
    servo_flipper_1.write(POS_FLIPPER_1_IN); 
    servo_flipper_2.write(POS_FLIPPER_2_IN); 

}


void sucker_initialize() {
    digitalWrite(sucker_pin_1,HIGH); // constant 
    digitalWrite(sucker_pin_2,HIGH); // off 
}

void flipper(){
  nh.loginfo("flipper"); 
    if (side_status == 0 && game_status == 1) { //move left flipper
        servo_flipper_1.write(POS_FLIPPER_1_OUT);
        delay(500);
        servo_flipper_1.write(POS_FLIPPER_1_IN);
    }
    if (side_status == 1 && game_status == 1 ) { //move right flipper 
        servo_flipper_2.write(POS_FLIPPER_2_OUT);
        delay(500); 
        servo_flipper_2.write(POS_FLIPPER_2_IN);
    }
    flipper_status = 0 ; 
}

void close_door() {
    nh.loginfo("close_door"); 
    servo_door_1.write(POS_DOOR_1_CLOSE); 
    servo_door_2.write(POS_DOOR_2_CLOSE); 
    current_door_status = 0 ; 
    door_status = 0 ; 
}

void open_door() {
    nh.loginfo("open_door"); 
    servo_door_1.write(POS_DOOR_1_OPEN); 
    servo_door_2.write(POS_DOOR_2_OPEN); 
    current_door_status = 1 ; 
    door_status = 1 ; 
}


void sucker_on () {
    digitalWrite(sucker_pin_1,HIGH); // constant 
    digitalWrite(sucker_pin_2,LOW); // on  
}

void sucker_off () {
    digitalWrite(sucker_pin_1,HIGH); // constant 
    digitalWrite(sucker_pin_2,HIGH); // off  
}


// ----------------------------------------------------------------------
// --------------------------- MAIN -------------------------------------
// ----------------------------------------------------------------------

void setup() {
    // ROS init 
    nh.initNode(); 

    nh.subscribe(sub_game_status);
    nh.subscribe(sub_door); 
 
    // Servo Actuator init 
    servo_door_1.attach(servo_door_1_pin) ;
    servo_door_2.attach(servo_door_2_pin) ; 
    servo_flipper_1.attach(servo_flipper_1_pin) ; 
    servo_flipper_2.attach(servo_flipper_2_pin) ; 

    // Sucker Actuator init 
    pinMode(sucker_pin_1,INPUT); 
    pinMode(sucker_pin_2,INPUT);   
}

void loop() {  
    // ROS loop 
    if ( door_status != current_door_status ) {
        if (door_status == 0 && game_status == 1 ) close_door() ; 
        if (door_status == 1 && game_status == 1 ) open_door() ; 
    }
    if ( flipper_status == 1 ) flipper() ; 
    if ( sucker_status  == 1 ) sucker_on() ; 
    else sucker_off() ; 

    nh.spinOnce() ; 
}
