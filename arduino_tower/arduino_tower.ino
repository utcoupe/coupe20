// Arduino : PR tower 

// ----- Includes --------

//ROS includes
#include <ros.h> 

// Actuators includes 
#include <Servo.h>
#include <Stepper.h>
// #include <ard_tower/tower_unload.msg>


// Servo motor 
Servo servo_unload ; // create servo object to control a servo


// Stepper motor 
#define STEPS 200 //number of steps on your motor  ?????
Stepper stepper (STEPS, 8, 10, 9, 11) ; // specify the pins 


// ----- Variables -------
 
int pos_unload_init = 90 ;  //store servo position
int pos_unload = 0 ; 

int pos_load_init = 0 ; // ??? 
int pos_load = 100 ;  // ???? 

int nb_atom_in = 0 ; //number of atom loaded 
int nb_atom_out = 0 ; //number of atom unloaded 

// ---- Function -----

void unload_atom() {
  servo_unload.write(pos_unload); 
  delay(500); 
  servo_unload.write(pos_unload_init); 
  nb_atom_out = nb_atom_out + 1 ; 
  nb_atom_in = nb_atom_in - 1 ; 
}

void load_atom() {
  stepper.step(pos_load_init) ; 
  nb_atom_in = nb_atom_in + 1 ; 
}

// ------ MAIN FUNCTIONS ----- 

void setup() {
  // ROS init 
  //nh.initNode(); 

  //nh.subscribe(<name>) 
  //nh.advertise(<name>)
  
  // Motor 
  servo_unload.attach(7) ;  // attaches the servo on pin 9 to the servo object
  stepper.setSpeed(30) ; // set the speed of the motor to 30 RPMs
}

void loop() {
  //read topics to unload when called 

  // ROS loop 
  //nh.spinOnce() 
}
