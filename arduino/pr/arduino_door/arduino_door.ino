//-----------------------------------------------
// Arduino : PR door 
// Author : Floriane ALLAIRE Feb 2019 , UTCoupe 
// Need door_variables.h 
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
    game_status = msg.game_status;
}

void on_door(const ard_door::Door& msg){
    door_status     = msg.door_status ; 
    flipper_status  = msg.flipper_status ; 
    side_status     = msg.side_status ; 
    if ( msg.init_status == 1 && game_status == 1 ) {
        door_initialize(); 
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

}

void loop() {  
    // ROS loop 
    nh.spinOnce() ; 
}
