//-----------------------------------------------
// Arduino : PR tower 
// Author : Floriane ALLAIRE Feb 2019 , UTCoupe 
// Need AFMotor. h and AFMotor.cpp in folder 
//------------------------------------------------

// ----- Includes --------

//ROS includes
#include <ros.h> 
#include <game_manager/GameStatus.h>
#include <ard_tower/Tower.h> 
#include <ard_tower/TowerResponses.h> 

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
void on_tower(const ard_tower::Tower& msg){
  nh.loginfo("tower_msg");
  load_content    = msg.load_content ; 
  load_content_nb = msg.load_content_nb ; 
  unload_content  = msg.unload_content ; 
  if (msg.init_status == 1 && game_status == 1) {
    tower_initialize(); 
  }
}

// ~ Subscriber ~ 
ros::Subscriber<game_manager::GameStatus> sub_game_status  ("ai/game_manager/status", &on_game_status)  ;
ros::Subscriber<ard_tower::Tower>         sub_tower        ("actuators/ard_tower",    &on_tower)   ;  


// ~ Publisher ~ 
ard_tower::TowerResponses  event_msg ; 
ros::Publisher pub_tower_responses   ("actuators/ard_tower/event",   &event_msg); 



// ------ MAIN FUNCTIONS ----- 

void tower_initialize() { 
  nh.loginfo("tower_initialize"); 
  servo_unload.write(POS_UNLOAD_INIT) ;   
  // initialize stepper 
  event_msg.init_success = 1 ; 
  pub_tower_responses.publish(&event_msg); 
  event_msg.init_success = 0 ; 
}

int move_lift(int wanted_position) {  // move lift to the correct floor 
nh.loginfo("move_lift"); 
  if ( lift_position > wanted_position && wanted_position >= H_GROUND && game_status == 1 ){  // go down 
    nh.loginfo("goes down"); 
    //stepper_load.moveStep(wanted_position,true) ; 
    lift_position = wanted_position ; 
    return 1 ; 
  }

  if (lift_position < wanted_position && wanted_position <= H_SAS_LOW && game_status == 1 ) { // go up 
    nh.loginfo("goes up"); 
    //stepper_load.moveStep(wanted_position,false) ; 
    lift_position = wanted_position ; 
    return 1 ; 
  }

  if (lift_position == wanted_position && game_status == 1) { //nothing 
    nh.loginfo("do nothing)"); 
    return 1 ; 
  }

  //TODO => sucess of the move before closing or opening AX12 
  nh.logwarn("error in moving"); 
  return 0 ; 
}

int lift_atoms_to_sas() { //bring up the rest of atoms to the sas 
  nh.loginfo("lift_atoms_to_sas"); 
  int y ; //number of atom not being brought up to the sas 
  if (nb_atom_in > MAX_ATOM_SAS )
    y = nb_atom_in - ( MAX_ATOM_SAS - nb_atom_in_sas ) ; 
  else 
    y = 0 ; 
    
  int h = (H_SAS_LOW-lift_position)-y*H_ATOM ; // or (y-1/2)  // h the  high that need to be brought up 
  int wanted_position = lift_position + h ; 
  int success = move_lift(wanted_position) ; 
  //TODO open AX12 

  if (success == 0 ){ 
    return success ; 
  }
  else {
    if ( y != 0 )
      nb_atom_in_sas = MAX_ATOM_SAS ; 
    else 
      nb_atom_in_sas = nb_atom_in ; 

    return success ; 
  }
}

// ~ unload ~ 

void unload_atom() {
  int success ; 
  if (unload_content == 1 && game_status == 1 ) { //unload atom with slider 
    success = unload_atom_slider() ; 

     if (( (nb_atom_in_sas == 0 && nb_atom_in == 0) || success == 0 ) && game_status == 1) {
      nh.loginfo("unload finished"); 
      //TODO clarify message if no success 
      event_msg.unload_success = success ;
      event_msg.nb_atom_out    = nb_atom_out ; 
      pub_tower_responses.publish(&event_msg) ; 
      unload_content = 0 ; 
      nb_atom_out    = 0 ; 
    }

  }

  if ( unload_content == 2 && game_status == 1 ) { //unload atom with pliers  
    success = unload_atom_pliers() ; 

    if ( (nb_atom_in-nb_atom_in_sas == 0 || success == 0) && game_status == 1 ) {
      nh.loginfo("unload finished") ; 
      event_msg.unload_success = success ; 
      event_msg.nb_atom_out    = nb_atom_out ; 
      pub_tower_responses.publish(&event_msg); 
      unload_content = 0 ; 
      nb_atom_out = 0 ;
    }
  }

}

int unload_atom_slider() {
  nh.loginfo("unload_atom_slider") ; 
  int success = 0 ; 
  if (nb_atom_in_sas > 0 && game_status == 1) { // sas not empty yet 
    nh.loginfo("atom in sas") ; 
    servo_unload.write(POS_UNLOAD); 
    delay(500); 
    servo_unload.write(POS_UNLOAD_INIT); 
    delay(500) ; 
    nb_atom_in     = nb_atom_in - 1 ; 
    nb_atom_in_sas = nb_atom_in_sas - 1 ; 
    nb_atom_out    = nb_atom_out + 1 ; 
    success = 1 ; 
  }

  if (nb_atom_in_sas == 0 && nb_atom_in > 0 && game_status == 1) {
    success = lift_atoms_to_sas() ; // 1 good, 0 bad wasn't able to reload sas 
  }

  return success ; 
}

int unload_atom_pliers() { 
  nh.loginfo("unload_atom_pliers") ; 
  if ( nb_atom_in - nb_atom_in_sas != 0 ) {
    // send message to AX12 and open gates 
    // TODO 
    nb_atom_out =  nb_atom_in - nb_atom_in_sas ; 
    nb_atom_in  = nb_atom_in_sas ; 
    return 1 ; 
  }
  else { // nothing to unload 
    return 1 ; 
  }
}


// ~ Load ~ 
void load_atom() {
  //nh.loginfo("load_atom"); 
  int success ; 
  if (load_content == 1 && game_status == 1 ) {
    success = load_atom_single() ;  //load one atom and ingame 
  }  
  if (load_content == 2 && game_status == 1) {  
    success = load_atom_tower() ;   //load tower of atoms and ingame
  }
  load_content = 0 ; 
  load_event_msg.load_success = success ; // in progress 
  load_event_msg.nb_atom_in  = nb_atom_in; // number of atom in 
  pub_tower_load.publish(&load_event_msg); 
}

int load_atom_single() {
  nh.loginfo("load_atom_single") ; 
  int success ; 
  if ( nb_atom_in_sas == nb_atom_in && nb_atom_in_sas < MAX_ATOM_SAS )  { // can still load in sas 
    nh.loginfo("loading in sas"); 
    //TODO send message to AX12 (open)
    success = move_lift(H_GROUND) ; 
    //TODO send message to AX12 (close)
    // TODO make it while moving ??? 
    if (success == 1) {
      success = lift_atoms_to_sas() ; 
      nb_atom_in     += 1 ; 
      nb_atom_in_sas += 1 ; 
    }
  }

  if ( nb_atom_in_sas == nb_atom_in && nb_atom_in_sas == MAX_ATOM_SAS ) { // pliers empty but sas full 
    nh.loginfo("loading in pliers 1")
    //TODO send message to AX12 (open)
    success = move_lift(H_GROUND) ; 
    //TODO send message to AX12 (close)
    if (success == 1) {
      success = move_lift(H_FLOOR_1) ; 
      nb_atom_in += 1 ; 
    } 
  }

  if ( nb_atom_in_sas - nb_atom_in < 0 ) { // pliers not empty and sas full 
    success = unload_atom_pliers() ; 
    nb_atom_out = 0 ; //take them back after 
    success = move_lift(H_GROUND) ; 
    //TODO send message to AX12 (close)
    if (success==1) {
      success = move_lift(H_FLOOR_1) ; 
      nb_atom_in += 1 ; 
    }
    else 
      success = 0 ; 
}

int load_atom_tower() {
  nh.loginfo("load_atom_tower") ; 
  return 1 ; 
}



void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(sub_game_status);
  nh.subscribe(sub_tower); 
 
  nh.advertise(pub_tower_responses); 

  // Servo Actuator init 
  servo_unload.attach(servo_unload_pin) ; 
  //stepper_load.moveStep(5000,false);  // test 
  
}

void loop() {
  unload_atom() ; 
  load_atom() ; 
  stepper_load.update() ; 
 
  
  // ROS loop 
  nh.spinOnce() ; 
}
