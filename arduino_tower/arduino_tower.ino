// Arduino : PR tower 

// ----- Includes --------

//ROS includes
#include <ros.h> 

// Actuators includes 
#include <Servo.h>
//#include <Stepper.h>
#include "AFMotor.h"
//#include <ard_tower/tower_unload.msg>


// Servo motor 
Servo servo_unload ; // create servo object to control a servo


// Stepper motor 
//#define STEPS 200 //number of steps on your motor  ?????
//const int stepsPerRevolution = 200 ; // change this to fit the number of steps per revolution
//Stepper stepper_load (STEPS, 8, 10, 9, 11) ; // specify the pins 
extern int16_t stepper_actuators_states[];
AF_Stepper stepper_load;

//AF_Stepper stepper_load ; 


// ----- Variables -------
 
int pos_unload_init = 90 ;  //store servo position
int pos_unload = 0 ; 

int pos_load_init = 0 ; // ??? 
int pos_load = 100 ;  // ???? 

int nb_atom_in = 0 ; //number of atom loaded 
int nb_atom_out = 0 ; //number of atom unloaded 

// ---- Function -----

void unload_atom() {
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
//  // read the sensor value:
//  int sensorReading = analogRead(A0);
//  // map it to a range from 0 to 100:
//  int motorSpeed = map(sensorReading, 0, 1023, 0, 100);
//  // set the motor speed:
//  if (motorSpeed > 0) {
//    stepper_load.setSpeed(motorSpeed);
//    // step 1/100 of a revolution:
//    stepper_load.step(stepsPerRevolution / 100);
//  }
//
 // stepper_load.step(pos_load_init) ; 
 // delay(500); 
 // stepper_load.step(pos_load) ; 
 // nb_atom_in = nb_atom_in + 1 ; 
stepper_load.step(100, FORWARD, SINGLE);

}




// ------ MAIN FUNCTIONS ----- 

void setup() {
  // ROS init 
  //nh.initNode(); 

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
