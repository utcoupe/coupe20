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
    servo_flipper_1.write(POS_FLIPPER_1_INIT); 
    servo_flipper_2.write(POS_FLIPPER_2_INIT); 
}

void flipper(){
    if (side_status == 0 && game_status == 1) { //move left flipper
        servo_flipper_1.write(POS_FLIPPER_1);
        servo_flipper_1.write(POS_FLIPPER_1_INIT);
    }
    if (side_status == 1 && game_status == 1 ) { //move right flipper 
        servo_flipper_2.write(POS_FLIPPER_2);
        servo_flipper_2.write(POS_FLIPPER_2_INIT);
    }
}

void close_door() {
    servo_door_1.write(POS_DOOR_1_CLOSE); 
    servo_door_2.write(POS_DOOR_2_CLOSE); 
    current_door_status = 0 ; 
}

void open_door() {
    servo_door_1.write(POS_DOOR_1_OPEN); 
    servo_door_2.write(POS_DOOR_2_OPEN); 
    current_door_status = 1 ; 
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
    if ( door_status != current_door_status ) {
        if (door_status == 0 && game_status == 1 ) close_door() ; 
        if (door_status == 1 && game_status == 1 ) open_door() ; 
    }
    if ( flipper_status == 1 ) flipper() ; 
    nh.spinOnce() ; 
}
