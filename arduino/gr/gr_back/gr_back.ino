//-----------------------------------------------
// Arduino : GR back actuators
// Author : Paul CONSTANT , UTCoupe 
// Need AFMotor. h and AFMotor.cpp in folder 
//------------------------------------------------

// ----- Includes --------

//ROS includes
#include <ros.h> 
#include <arg_gr_back/DoorStatus.h>
#include <ard_gr_back/ScaleRelease.h> 
#include <ard_gr_back/TowerRelease.h> 
#include <game_manager/GameStatus.h>
ros :: NodeHandle nh ; 

// Actuators includes 
#include <Servo.h>
//#include <Stepper.h>
#include "AFMotor.h"



// Servo motors
Servo door_servo ; // create servo object to control a servo

// ----- Variables -------
 
// int door_pos_init = 90 ;  //store servo position

int door_content = -1 ; // actions given to door 
int door_status    = -1 ; // info on door status
int game_status    = -1 ; // info on game status 

//----- ROS methods ------ 
void on_game_status(const game_manager::GameStatus& msg){
  game_status = msg.game_status;
  //init_status = msg.init_status;
}

void on_door_status(const ard_gr_back::DoorStatus& msg){
  door_status = msg.door_status ; 
}

ros::Subscriber<game_manager::GameStatus>  sub_game_status  ("ai/game_manager/status",     &on_game_status); 
ros::Subscriber<ard_gr_back::DoorStatus>   sub_door_status   ("actuators/ard_gr_back/open",   &on_door_status);

// ros::Publisher pub_door_status  ("actuators/ard_tower/load_event",   &load_event_msg); 


// ------ MAIN FUNCTIONS ----- 

void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(sub_game_status); 
  nh.subscribe(sub_door_status); 

 // nh.advertise(pub_tower_load) ; 
 // nh.advertise(pub_tower_unload); 
  
  // Door actuator init
  door_servo.attach(22) ;  // attaches the servo on pin 22 to the servo object
  door_servo.init(0,1) ;  // ??
  door_servo.setSpeed(120) ; // set the speed of the motor to 30 RPMs
}

void loop() {
  
  //ROS loop 
  nh.spinOnce() ; 
}
