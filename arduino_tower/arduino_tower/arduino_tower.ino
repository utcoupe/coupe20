//-----------------------------------------------
// Arduino : PR tower 
// Author : Floriane ALLAIRE Feb 2019 , UTCoupe 
// Need AFMotor. h and AFMotor.cpp in folder 
//------------------------------------------------

// ----- Includes --------

//ROS includes
#include <ros.h> 
#include <game_manager/GameStatus.h>
#include <ard_tower/TowerInit.h> 
#include <ard_tower/TowerInitResponse.h>
#include <ard_tower/TowerUnload.h>
#include <ard_tower/TowerUnloadResponse.h>
#include <ard_tower/TowerLoad.h> 
ros :: NodeHandle nh ; 

// Actuators includes 
#include <Servo.h> 
#include "PololuA4983.h"

// Code includes 
#include "tower_variables.h" 

// Servo motor 
Servo servo_unload ; // create servo object to control a servo

// Stepper motor 
PololuA4983 stepper_load = PololuA4983(step_pin, dir_pin, en_pin, min_delay); 



//----- ROS methods ------ 
void on_game_status(const game_manager::GameStatus& msg){
  game_status = msg.game_status;
  nh.loginfo("game_status_msg");
}

void on_tower_init(const ard_tower::TowerInit& msg) { 
  // tower_initialize(); 
  nh.loginfo("tower_init_msg") ; 
}

void on_tower_unload (const ard_tower::TowerUnload& msg){
  unload_content = msg.unload_content ; 
  nh.loginfo("tower_unload_msg") ;
}

//void on_tower_load  (const ard_tower::TowerLoad& msg){
//  nh.loginfo("tower_load_msg") ;
//  //load_content          = msg.load_content ; 
//  //load_content_nb       = msg.load_content_nb ; 
//  //load_content_position = msg.load_content_nb ; 
//}


// ~ Subscriber ~ 
ros::Subscriber<game_manager::GameStatus> sub_game_status  ("ai/game_manager/status",     &on_game_status)  ;
ros::Subscriber<ard_tower::TowerInit>     sub_tower_init   ("actuators/ard_tower/init",   &on_tower_init)   ;  
ros::Subscriber<ard_tower::TowerUnload>   sub_tower_unload ("actuators/ard_tower/unload", &on_tower_unload) ;
//ros::Subscriber<ard_tower::TowerLoad>     sub_tower_load   ("actuators/ard_tower/load",   &on_tower_load)   ;


// ~ Publisher ~ 
ard_tower::TowerInitResponse  init_event_msg ; 
ard_tower::TowerUnloadResponse unload_event_msg ; 
ros::Publisher pub_tower_init   ("actuators/ard_tower/init_event",   &init_event_msg); 
ros::Publisher pub_tower_unload ("actuators/ard_tower/unload_event", &unload_event_msg); 


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

     if (( (nb_atom_in_sas == 0 && nb_atom_in == 0) || success == 0 ) && game_status == 1) {
      nh.loginfo("unload finished"); 
      //TODO clarify message if no success 
      unload_event_msg.unload_success = success ;
      unload_event_msg.nb_atom_out    = nb_atom_out ; 
      pub_tower_unload.publish(&unload_event_msg) ; 
      unload_content = 0 ; 
      nb_atom_out    = 0 ; 
    }

  }

  if ( unload_content == 2 && game_status == 1 ) { //unload atom with pliers  
    success = unload_atom_pliers() ; 

    if ( (nb_atom_in-nb_atom_in_sas == 0 || success == 0) && game_status == 1 ) {
      nh.loginfo("unload finished") ; 
      unload_event_msg.unload_success = success ; 
      unload_event_msg.nb_atom_out    = nb_atom_out ; 
      pub_tower_unload.publish(&unload_event_msg); 
      unload_content = 0 ; 
      nb_atom_out = 0 ;
    }
  }

  
}

int unload_atom_slider() {
  nh.loginfo("unload_atom_slider") ; 
  int success ; 
  if (nb_atom_in_sas > 0 && game_status == 1) { // sas not empty yet 
    nh.loginfo("atom in sas") ; 
    servo_unload.write(POS_UNLOAD); 
    delay(500); 
    servo_unload.write(POS_UNLOAD_INIT); 
    nb_atom_in     = nb_atom_in - 1 ; 
    nb_atom_in_sas = nb_atom_in_sas - 1 ; 
    nb_atom_out    = nb_atom_out + 1 ; 
    success = 1 ; 
  }




  return success ; 
}

int unload_atom_pliers() { 
  nh.loginfo("unload_atom_pliers") ; 
  //something 
  return 1 ; 
}


// ~ Load ~ 
void load_atom() {
  nh.loginfo("load_atom"); 
  int success ; 
  if (load_content == 1 && game_status == 1 ) {
    success = load_atom_single() ;  //load one atom and ingame 
  }  
  if (load_content == 2 && game_status == 1) {  
    success = load_atom_tower() ;   //load tower of atoms and ingame
  }
}

int load_atom_single() {
  nh.loginfo("load_atom_single") ; 
  nb_atom_in += 1 ; 
  nb_atom_in_sas += 1 ; 
  return 1 ; 
}

int load_atom_tower() {
  nh.loginfo("load_atom_tower") ; 
  return 1 ; 
}



void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(sub_game_status);
  nh.subscribe(sub_tower_init); 
  nh.subscribe(sub_tower_unload); 
  //nh.subscribe(sub_tower_load); 

  nh.advertise(pub_tower_init); 
  nh.advertise(pub_tower_unload); 

  // Servo Actuator init 
  servo_unload.attach(servo_unload_pin) ; 
  
}

void loop() {
  unload_atom() ; 
 
  
  // ROS loop 
  nh.spinOnce() ; 
}
