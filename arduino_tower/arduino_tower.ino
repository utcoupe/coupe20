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
#include <game_manager/GameStatus.h>
ros :: NodeHandle nh ; 

// Actuators includes 
#include <Servo.h>
//#include <Stepper.h>
#include "AFMotor.h"



// Servo motor 
Servo servo_unload ; // create servo object to control a servo

// Stepper motor 
//#define STEPS 200 //number of steps on your motor  ?????
//Stepper stepper_load (STEPS, 8, 10, 9, 11) ; // specify the pins 
extern int16_t stepper_actuators_states[];
AF_Stepper stepper_load;

//AF_Stepper stepper_load ; 


// ----- Variables -------
 
int pos_unload_init = 90 ;  //store servo position
int pos_unload      = 0 ; 

int pos_load_init = 0 ; // ??? 
int pos_load      = 100 ;  // ???? 

float nb_atom_in  = 0 ; //number of atom loaded 
float nb_atom_out = 0 ; //number of atom unloaded 

int load_content   = -1 ; // actions given to stepper 
int unload_content = -1 ; // actions given to stepper 
int load_stauts    = -1 ; // info on stepper status 
int unload_status  = -1 ; // info on stepper status 
int game_status    = -1 ; // info on game status 

//----- ROS methods ------ 
void on_game_status(const game_manager::GameStatus& msg){
  game_status = msg.game_status;
  //init_status = msg.init_status;
}

void on_tower_load  (const ard_tower::TowerLoad& msg){
  load_content = msg.load_content ; 
}

void on_tower_unload (const ard_tower::TowerUnload& msg){
  unload_content = 1 ; 
}

ros::Subscriber<game_manager::GameStatus> sub_game_status  ("ai/game_manager/status",     &on_game_status); 
ros::Subscriber<ard_tower::TowerLoad>     sub_tower_load   ("actuators/ard_tower/load",   &on_tower_load);
ros::Subscriber<ard_tower::TowerUnload>   sub_tower_unload ("actuators/ard_tower/unload", &on_tower_unload);

//ros::Publisher pub_tower_load   ("actuators/ard_tower/load_event",   &load_event_msg); 
//ros::Publisher pub_tower_unload ("actuators/ard_tower/unload_event", &unload_event_msg); 


// ---- Function -----

void unload_atom() {
  if ( unload_content == 1 &&  game_status == 1 ) { //nedd to unload atom  and ingame 
    servo_unload.write(pos_unload); 
    delay(500); 
    servo_unload.write(pos_unload_init); 
    nb_atom_out = nb_atom_out + 1 ; 
    nb_atom_in = nb_atom_in - 1 ; 
    unload_content = 0 ; 
  }
  else { // wait for message 
    servo_unload.write(pos_unload_init); 
  }
}

void load_atom() {
  if (load_content == 1 && game_status == 1 ) { //load one atom and ingame 
    stepper_load.step(100, FORWARD, SINGLE);
  }
  if (load_content == 2 && game_status == 1 ) { //load tower of atoms and ingame 
    stepper_load.step(100, FORWARD, SINGLE);
  }
  if (load_content == 0 && game_status == 1 ) { //load nothing (go back down)
    stepper_load.step(100,BACKWARD,SINGLE); 
  }
  else { // not ingame 
    //do nothing 
  }
}

// ------ MAIN FUNCTIONS ----- 

void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(sub_game_status); 
  nh.subscribe(sub_tower_load); 
  nh.subscribe(sub_tower_unload); 

 // nh.advertise(pub_tower_load) ; 
 // nh.advertise(pub_tower_unload); 
  
  // Servo Actuator init 
  servo_unload.attach(22) ;  // attaches the servo on pin 22 to the servo object

  // Stepper Actuator init 
  stepper_load.init(200,1) ; 
  stepper_load.setSpeed(120) ; // set the speed of the motor to 30 RPMs
}

void loop() {
  unload_atom() ; 
  load_atom(); 
  
  //ROS loop 
  nh.spinOnce() ; 
}
