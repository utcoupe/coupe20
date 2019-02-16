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
#include "AFMotor.h"

// Variables includes 
#include "tower_variables.h"


// Servo motor 
Servo servo_unload ; // create servo object to control a servo

// Stepper motor 
extern int16_t stepper_actuators_states[];
AF_Stepper stepper_load;

//----- ROS methods ------ 
void on_tower_init(const ard_tower::TowerLoad& msg) { //change TowerLoad to TowerInit
  tower_initialize(); 
}

void on_game_status(const game_manager::GameStatus& msg){
  game_status = msg.game_status;
}

void on_tower_load  (const ard_tower::TowerLoad& msg){
  load_content = msg.load_content ; 
  load_content_nb = msg.load_content_nb ; 
}

void on_tower_unload (const ard_tower::TowerUnload& msg){
  unload_content = msg.unload_content ; 
}

// ~ Subscriber ~ 

ros::Subscriber<ard_tower::TowerLoad>     sub_tower_init   ("actuators/ard_tower/init",   &on_tower_init)   ; //change TowerLoad to TowerInit
ros::Subscriber<game_manager::GameStatus> sub_game_status  ("ai/game_manager/status",     &on_game_status)  ; 
ros::Subscriber<ard_tower::TowerLoad>     sub_tower_load   ("actuators/ard_tower/load",   &on_tower_load)   ;
ros::Subscriber<ard_tower::TowerUnload>   sub_tower_unload ("actuators/ard_tower/unload", &on_tower_unload) ;

// ~ Publisher ~ 
ard_tower::TowerLoadResponse load_event_msg ; 
ard_tower::TowerUnloadResponse unload_event_msg ; 
ard_tower::TowerUnloadResponse  init_event_msg ; //change TowerUnloadResponse to TowerInitResponse
ros::Publisher pub_tower_load   ("actuators/ard_tower/load_event",   &load_event_msg); 
ros::Publisher pub_tower_unload ("actuators/ard_tower/unload_event", &unload_event_msg); 
ros::Publisher pub_tower_init   ("actuators/ard_tower/init_event",   &init_event_msg); 


// ---- Function -----

void tower_initialize() { 
  // initialize stepper 
  servo_unload.write(POS_UNLOAD_INIT) ; 

}

void unload_atom() { 
  if ( unload_content == 1 &&  game_status == 1 ) { //unload atom with slider 
    unload_atom_slider() ; 
  }
  if ( unload_content == 2 && game_status == 1 ) { //unload atom with pliers 
    unload_atom_pliers(); 
  }
  pub_tower_unload.publish(&unload_event_msg); 
}

void unload_atom_slider() {
  if ( nb_atom_in_sas > 0 && game_status == 1 ){
    servo_unload.write(POS_UNLOAD); 
    delay(500); 
    servo_unload.write(POS_UNLOAD_INIT); 
    nb_atom_in     = nb_atom_in - 1 ; 
    nb_atom_in_sas = nb_atom_in_sas -  1 ; 
  }

  if ( nb_atom_in_sas == 0 && nb_atom_in > 0 && game_status == 1) {
    //bring up the rest of atoms to the sas 
    int y ; //number of atom not being brought up
    if (nb_atom_in > MAX_ATOM_SAS ) {
      y = nb_atom_in - MAX_ATOM_SAS ; 
      nb_atom_in_sas = MAX_ATOM_SAS ; 
    }
    else {
      y = 0 ; 
      nb_atom_in_sas = nb_atom_in ; 
    }
    int h = (H_SAS_LOW-positions)-y*H_ATOM ; // or (y-1/2)
    // the high that need to be brought up 
    // TODO 
  }
  
  if (nb_atom_in_sas == 0 && nb_atom_in == 0 && game_status == 1) {
    unload_content = 0 ; 
    unload_event_msg.unload_success = 1 ;
  }

  unload_event_msg.unload_success = 0 ; // in proogress     

}

void unload_atom_pliers() {
  if ( nb_atom_in - nb_atom_in_sas != 0 ) {
    // send message to AX12 and open gates 
    // TODO 
    nb_atom_in = nb_atom_in_sas ; 
  }
  else {
    // nothing to unload 
    unload_content = 0 ; 
    unload_event_msg.unload_success = 1 ; 
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

  load_event_msg.load_success = 0 ; // in progress 
  load_event_msg.load_success = 1 ;// finish 
  load_event_msg.nb_atom_in  = 0 ; // number of atom loaded (float32) 
  pub_tower_load.publish(&load_event_msg); 
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
