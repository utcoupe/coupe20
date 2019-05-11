//-----------------------------------------------
// Arduino : PR tower 
// Author : Floriane ALLAIRE Feb 2019 , UTCoupe 
// Need AFMotor. h tower_variables.h and AFMotor.cpp in folder 
//------------------------------------------------

// -----------------------------------------------------------
// --------------------- Includes ----------------------------
// -----------------------------------------------------------

//ROS includes
#include <ros.h> 
#include <game_manager/GameStatus.h>
#include <ard_tower/Tower.h> 
#include <ard_tower/TowerResponses.h> 
#include <ard_door/Door.h> 
//#include <driver_ax12/Ax12Command.h> 


ros :: NodeHandle nh ; 

// Actuators includes 
#include <Servo.h> 
#include "PololuA4983.h"

// Code includes 
#include "tower_variables.h" 

// Servo motor 
Servo servo_unload_1 ; // create servo object to control a servo
Servo servo_unload_2 ; 

// Stepper motor 
PololuA4983 stepper_load = PololuA4983(step_pin, dir_pin, en_pin, min_delay); 


// ----------------------------------------------------------------
// ---------------------------- ROS methods ----------------------- 
// ----------------------------------------------------------------
void on_game_status(const game_manager::GameStatus& msg){
  game_status = msg.game_status;
  //nh.loginfo("game_status_msg");
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
ard_door::Door door_msg ; 
//driver_ax12::Ax12Command  ax12_event_msg ; 

ros::Publisher pub_tower_responses   ("actuators/ard_tower/event",   &event_msg); 
ros::Publisher pub_door_action       ("actuators/ard_door/action",   &door_msg ); 
//ros::Publisher pub_ax12_responses    ("actuators/ard_tower/ax12",    &ax12_event_msg); 


// --------------------------------------------------------------
// --------------------- FUNCTIONS ------------------------------ 
// --------------------------------------------------------------

void tower_initialize() { 
  nh.loginfo("tower_initialize"); 
  servo_unload_1.write(POS_UNLOAD_INIT_1) ;
  servo_unload_2.write(POS_UNLOAD_INIT_2) ;    
  // initialize stepper 
  event_msg.init_success = 1 ; 
  pub_tower_responses.publish(&event_msg); 
  event_msg.init_success = 0 ; 
}

// ~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~ AX12  ~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~

int move_ax12(bool open ) { // 1 : open AX12 and 0 : close AX12 
  nh.loginfo("move_ax12"); 
  //ax12_event_msg.speed = 1000 ; 
  //ax12_event_msg.mode = 0 ; 
  //ax12_event_msg.motor_id = 2 ; 
  //if ( open == 1 ) ax12_event_msg.position =135  ; // Open 
  //if ( open == 0 ) ax12_event_msg.position =170 ;  // Close 
  //pub_ax12_responses.publish(&ax12_event_msg) ; 
  return 1 ; 
}

// ~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~ Lift  ~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~

int move_lift(int wanted_position) {  // move lift to the correct floor 
nh.loginfo("move_lift"); 
  if ( lift_position > wanted_position && wanted_position >= H_GROUND && game_status == 1 ){  // go down 
    nh.loginfo("goes down"); 
    stepper_load.moveStep(lift_position - wanted_position,true); 
    while( stepper_load.getRemainingStep() >0 ) {
      stepper_load.update() ; 
    }
    delay(500) ; 
    lift_position = wanted_position ; 
    return 1 ; 
  }

  if (lift_position < wanted_position && wanted_position <= H_SAS_LOW && game_status == 1 ) { // go up 
    nh.loginfo("goes up"); 
    stepper_load.moveStep(wanted_position-lift_position,false); 
    while( stepper_load.getRemainingStep() >0 ) {
      stepper_load.update() ; 
    }
    delay(500) ; 
    lift_position = wanted_position ; 
    return 1 ; 
  }

  if (lift_position == wanted_position && game_status == 1) { //nothing 
    nh.loginfo("do nothing"); 
    return 1 ; 
  }

  nh.logwarn("error in moving"); 
  return 0 ; 
}


int unload_atom_sas () {
  nh.loginfo("unload_atom_sas"); 
  int success ; 
  if (nb_atom_in > MAX_ATOM_SAS && nb_atom_in_sas == 0 && game_status == 1 ) { // not enough space in sas for all atoms 
    nh.loginfo("not enough space"); 
    float nb_atom_out_right = nb_atom_out ; // difference between out with slider or with pliers 
    success = unload_atom_pliers() ;   // nb_atom_in = 0 at this point and nb_atom_out will change 
    float nb_atom_out_wrong = nb_atom_out - nb_atom_out_right ; 
    if (success != 0 ) success = load_atom_tower(nb_atom_out_wrong,MAX_ATOM_SAS) ; 
    if (success != 0 ) nb_atom_out_wrong = nb_atom_out_wrong - nb_atom_in ; // with nb_atom_in = MAX_ATOM_SAS
    if (success != 0 ) success = move_lift(H_SAS_LOW) ; 
    if (success != 0) nb_atom_in_sas = MAX_ATOM_SAS  ; 
    if (success != 0 ) success = move_ax12(1);  // open AX12 
    if (success != 0) success = move_lift(H_FLOOR_1) ; 
    if (success != 0)  success = load_atom_tower(nb_atom_out_wrong, nb_atom_out_wrong) ; 
  }
  else { // enough space for all the atoms 
    nh.loginfo(" enough space"); 
    if (success != 0 ) success = move_lift(H_SAS_LOW) ; 
    if (success != 0 ) nb_atom_in_sas = nb_atom_in ; 
    if (success != 0) success = move_ax12(1) ; // open AX12 
    if (success != 0 ) success = move_lift(H_FLOOR_1) ; 
  }
  
  return success ; 
}


int load_atom_sas() {
  nh.loginfo("load_atom_sas"); 
  if (nb_atom_in<MAX_ATOM_SAS){
    nh.loginfo("not enough atom.s"); 
    return 1 ; 
  }
  else {
    if (nb_atom_in == MAX_ATOM_SAS && nb_atom_in_sas == 0 ) {    
      nh.loginfo("enough atoms"); 
      int success = move_lift(H_SAS_LOW); 
      if (success != 0 ) success = move_ax12(1) ; // open AX12 
      if (success != 0) success = move_lift(H_FLOOR_1) ; 
      nb_atom_in_sas = nb_atom_in ; 
      return success ; 
    }
    else {
      nh.loginfo("sas already full"); 
      return 1 ; 
    }
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~ Unload ~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~

void unload_atom() {
  int success ;
  if (unload_content == 1 && game_status == 1 ) { //unload atom with slider 
    success = unload_atom_slider() ; 

     if (( nb_atom_in == 0 || success == 0 ) && game_status == 1) {
      nh.loginfo("unload finished"); 
      //TODO clarify message if no success 
      event_msg.unload_success = success ;
      event_msg.nb_atom_out    = nb_atom_out ; 
      event_msg.nb_atom_in     = nb_atom_in ;
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
    servo_unload_1.write(POS_UNLOAD_1);
    servo_unload_2.write(POS_UNLOAD_2); 
    //close 
    delay(500); 
    servo_unload_1.write(POS_UNLOAD_INIT_1); 
    servo_unload_2.write(POS_UNLOAD_INIT_2); 
    //open  
    delay(500) ; 
    nb_atom_in     -= 1 ; 
    nb_atom_in_sas -= 1 ; 
    nb_atom_out    += 1 ; 
    success = 1 ; 
  }

  if (nb_atom_in_sas == 0 && nb_atom_in > 0 && game_status == 1) {
    success = unload_atom_sas() ; // 1 good, 0 bad wasn't able to reload sas 
  }

  return success ; 
}

int unload_atom_pliers() { 
  nh.loginfo("unload_atom_pliers") ; 
  int success ; 
  if ( nb_atom_in - nb_atom_in_sas != 0 ) {
    nh.loginfo("atoms drop"); 
    success = move_ax12(1) ; // AX12 open 
    nb_atom_out =  nb_atom_in - nb_atom_in_sas ; 
    nb_atom_in  = nb_atom_in_sas ; 
    return 1 ; 
  }
  else { // nothing to unload 
    nh.loginfo("nothing to drop"); 
    return success ; 
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~ Load ~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~

void load_atom() {
  //nh.loginfo("load_atom"); 
  int success = -1 ; 
  if (load_content == 1 && game_status == 1 ) {
    success = load_atom_single() ;  //load one atom and ingame 
  }  
  if (load_content == 2 && game_status == 1) {  // TODO Specify size of tower the whole with messages (See with AI)
    success = load_atom_tower(load_content_nb,load_content_nb) ;   //load tower of atoms and ingame
  }
  if (success != -1 ) {
    event_msg.load_success = success ; 
    event_msg.nb_atom_in   = nb_atom_in; // number of atom in 
    pub_tower_responses.publish(&event_msg); 
    success = load_atom_sas() ; //if enough atomes and space bring atoms to sas 
    load_content = 0 ; 
  }
  
}

int load_atom_single() {
  nh.loginfo("load_atom_single") ; 
  int success ; 
  if (nb_atom_in-nb_atom_in_sas == 0){ //pliers empty 
    success = move_ax12(1) ; // open 
    if (success != 0 ) success = move_lift(H_GROUND) ; 
    if (success != 0 ) success = move_ax12(0); // close
  }
  else { //pliers not empty 
    success = unload_atom_pliers() ; 
    if (success != 0 ) success = move_lift(H_GROUND) ; 
    if (success != 0 ) success = move_ax12(0) ; //close 
  }
  if (success != 0 ) success = move_lift(H_FLOOR_1); 
  nb_atom_in    = nb_atom_in + nb_atom_out + 1 ; 
  nb_atom_out   = 0 ;
  return success ; // atom has been grabbed 
} 

int load_atom_tower(float nb_atom_tower, float nb_atom_wanted) {
  nh.loginfo("load_atom_tower") ; 
  int success ; 
  if (nb_atom_in - nb_atom_in_sas == 0 ) {  // will only do this case !! 
    success = move_lift(nb_atom_tower-nb_atom_wanted*300) ; // TODO replace 300 with constante 
    if (success != 0 ) success = move_ax12(0) ; //close
    if (success != 0) nb_atom_in = nb_atom_wanted ; 
    nh.loginfo("tower taken"); 
  }
  return success ; 
}


// ----------------------------------------------------------------------
// --------------------------- MAIN -------------------------------------
// ----------------------------------------------------------------------

void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(sub_game_status);
  nh.subscribe(sub_tower); 
 
  nh.advertise(pub_tower_responses); 
  nh.advertise(pub_door_action); 
//  nh.advertise(pub_ax12_responses); 

  // Servo Actuator init 
  servo_unload_1.attach(servo_unload_pin_1) ; 
  servo_unload_2.attach(servo_unload_pin_2) ; 
  
}

void loop() {
  unload_atom() ; 
  load_atom() ; 
  stepper_load.update() ; 
 
  
  // ROS loop 
  nh.spinOnce() ; 
}
