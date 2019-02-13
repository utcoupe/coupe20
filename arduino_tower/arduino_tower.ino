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
int pos_unload = 0 ; 

int pos_load_init = 0 ; // ??? 
int pos_load = 100 ;  // ???? 

float nb_atom_in = 0 ; //number of atom loaded 
float nb_atom_out = 0 ; //number of atom unloaded 

int load_content   = -1 ; // actions given to stepper 
int unload_content = -1 ; 
int load_stauts    = -1 ; 
int unload_status  = -1 ; nh


// ---- Function -----

void unload_atom() {nh
  servo_unload.write(pos_unload_init); 
  delay(700) ; 
  servo_unload.write(pos_unload); 
  delay(700); 
  servo_unload.write(pos_unload_init); 
  delay(700) ; 
  nb_atom_out = nb_atom_out + 1 ; 
  nb_atom_in = nb_atom_in - 1 ; 
}

void load_atom() {
stepper_load.step(100, FORWARD, SINGLE);
}




// ------ MAIN FUNCTIONS ----- 

void setup() {
  // ROS init 
  nh.initNode(); 

  nh.subscribe(TowerLoad); 
  nh.subscribe(TowerUnload); 


  //nh.subscribe(<name>) 
  //nh.advertise(<name>)
  
  // Servo Actuator init 
  servo_unload.attach(22) ;  // attaches the servo on pin 9 to the servo object

  // Stepper Actuator init 
  stepper_load.init(200,1) ; 
  stepper_load.setSpeed(120) ; // set the speed of the motor to 30 RPMs
}

void loop() {
  //read topics to unload when called 
  //unload_atom() ; 
  load_atom(); 
  
  // ROS loop 
  //nh.spinOnce() 
}
