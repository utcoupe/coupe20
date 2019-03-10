//-----------------------------------------------
// Arduino : PR tower 
// Author : Floriane ALLAIRE Feb 2019 , UTCoupe 
// Need AFMotor. h and AFMotor.cpp in folder 
//------------------------------------------------

// ----- Includes --------

//ROS includes
#include <ros.h> 
#include <ard_tower/TowerUnload.h>
#include <ard_tower/TowerInit.h> 
#include <ard_tower/TowerInitResponse.h>
#include <game_manager/GameStatus.h>
ros :: NodeHandle nh ; 

// Actuators includes 
#include <Servo.h> 
#include "PololuA4983.h"

// Code includes 
#include "tower_variables.h" 

// Serco motor 
Servo servo_unload ; // create servo object to control a servo



//----- ROS methods ------ 
void on_game_status(const game_manager::GameStatus& msg){
  game_status = msg.game_status;
  nh.loginfo("game_status");
}

void on_tower_init(const ard_tower::TowerInit& msg) { 
  // tower_initialize(); 
  nh.loginfo("tower_init") ; 
}

void on_tower_unload (const ard_tower::TowerUnload& msg){
  unload_content = msg.unload_content ; 
  nh.loginfo("tower_unload") ;
}

// ~ Subscriber ~ 
ros::Subscriber<game_manager::GameStatus> sub_game_status  ("ai/game_manager/status",     &on_game_status)  ;
ros::Subscriber<ard_tower::TowerInit>     sub_tower_init   ("actuators/ard_tower/init",   &on_tower_init)   ;  
ros::Subscriber<ard_tower::TowerUnload>   sub_tower_unload ("actuators/ard_tower/unload", &on_tower_unload) ;

// ~ Publisher ~ 
ard_tower::TowerInitResponse  init_event_msg ; 
ros::Publisher pub_tower_init   ("actuators/ard_tower/init_event",   &init_event_msg); 

// ------ MAIN FUNCTIONS ----- 

void tower_initialize() { 
  servo_unload.write(POS_UNLOAD_INIT) ;   // initialize stepper 
  init_event_msg.init_success = 1 ; 
  pub_tower_init.publish(&init_event_msg); 
}

void unload_atom() {
  int success ; 
  if (unload_content == 1 && game_status == 1 ) { //unload atom with slider 
    success = unload_atom_slider() ; 
  }

  if ( unload_content == 2 && game_status == 1 ) { //unload atom with pliers  
    success = unload_atom_pliers() ; 
  }
}

int unload_atom_slider() {
  nh.loginfo("unload_atom_slider") ; 
  servo_unload.write(POS_UNLOAD); 
  delay(500); 
  servo_unload.write(POS_UNLOAD_INIT); 
  unload_content = 0 ; 
  return 1 ; 
}

int unload_atom_pliers() { 
  //something 
  return 1 ; 
}

void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(sub_game_status);
  nh.subscribe(sub_tower_init); 
  nh.subscribe(sub_tower_unload); 

  nh.advertise(pub_tower_init) ; 

  // Servo Actuator init 
  servo_unload.attach(servo_unload_pin) ; 
  
}

void loop() {
  unload_atom() ; 
  
  // ROS loop 
  nh.spinOnce() ; 
}
