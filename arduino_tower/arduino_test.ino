//-----------------------------------------------
// Arduino : PR tower 
// Author : Floriane ALLAIRE Feb 2019 , UTCoupe 
// Need AFMotor. h and AFMotor.cpp in folder 
//------------------------------------------------

// ----- Includes --------

//ROS includes
#include <ros.h> 
#include <ard_tower/TowerUnloadResponse.h>
#include <ard_tower/TowerLoadResponse.h> 
#include <ard_tower/TowerLoad.h> 
#include <ard_tower/TowerUnload.h>
#include <ard_tower/TowerInit.h> 
#include <ard_tower/TowerInitResponse.h>
#include <game_manager/GameStatus.h>
ros::NodeHandle nh1;
//ros :: NodeHandle nh ; 

// Actuators includes 
#include <Servo.h> 
#include "PololuA4983.h"

// Code includes 
#include "tower_variables.h" 


// Servo motor 
Servo servo_unload ; // create servo object to control a servo

// Stepper motor 
//PololuA4983 stepper_load = PololuA4983(step_pin, dir_pin, en_pin, min_delay); 

//----- ROS methods ------ 
void on_tower_init(const ard_tower::TowerLoad& msg) { //change TowerLoad to TowerInit
  tower_initialize(); 
  //Serial.print("tower_initialize_msg");
}

void on_game_status(const game_manager::GameStatus& msg){
  game_status = msg.game_status;
  //Serial.print("game_status_msg");
}

void on_tower_load  (const ard_tower::TowerLoad& msg){
  load_content          = msg.load_content ; 
  load_content_nb       = msg.load_content_nb ; 
  load_content_position = msg.load_content_nb ; 
  //Serial.print("on_tower_load_msg");
}

void on_tower_unload (const ard_tower::TowerUnload& msg){
  unload_content = msg.unload_content ; 
  //Serial.print("on_tower_unload_msg");
}

// ~ Subscriber ~ 

ros::Subscriber<ard_tower::TowerInit>     sub_tower_init   ("actuators/ard_tower/init",   &on_tower_init)   ; 
ros::Subscriber<game_manager::GameStatus> sub_game_status  ("ai/game_manager/status",     &on_game_status)  ; 
ros::Subscriber<ard_tower::TowerLoad>     sub_tower_load   ("actuators/ard_tower/load",   &on_tower_load)   ;
ros::Subscriber<ard_tower::TowerUnload>   sub_tower_unload ("actuators/ard_tower/unload", &on_tower_unload) ;

// ~ Publisher ~ 
ard_tower::TowerLoadResponse load_event_msg ; 
ard_tower::TowerUnloadResponse unload_event_msg ; 
ard_tower::TowerInitResponse  init_event_msg ; 
ros::Publisher pub_tower_load   ("actuators/ard_tower/load_event",   &load_event_msg); 
ros::Publisher pub_tower_unload ("actuators/ard_tower/unload_event", &unload_event_msg); 
ros::Publisher pub_tower_init   ("actuators/ard_tower/init_event",   &init_event_msg); 


// ---- Function -----

void unload_atom() { 
  int success ; 
  if ( unload_content == 1 &&  game_status == 1 ) { //unload atom with slider 
    servo_unload.write(POS_UNLOAD); 
    delay(500); 
    servo_unload.write(POS_UNLOAD_INIT); 
  }
}

// ------ MAIN FUNCTIONS ----- 

void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(sub_tower_init); 
  nh.subscribe(sub_game_status); 
  nh.subscribe(sub_tower_load); 
  nh.subscribe(sub_tower_unload); 

  nh.advertise(pub_tower_init) ; 
  nh.advertise(pub_tower_load) ; 
  nh.advertise(pub_tower_unload); 
  
  // Servo Actuator init 
  servo_unload.attach(servo_unload_pin) ;  // attaches the servo on pin 22 to the servo object

  //Arduino 
  //Serial.begin(9600);
}

void loop() {
  unload_atom() ; 
  //stepper_load.update() ; 
  //Serial.print(3);
  
  
  //ROS loop 
  nh.spinOnce() ; 
}
