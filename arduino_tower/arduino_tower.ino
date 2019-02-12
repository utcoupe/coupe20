// Arduino : PR tower 

//ROS includes
#include <Servo.h>

// Servo motor 
Servo servo_tower;  // create servo object to control a servo

// Variable to store the servo position
int pos_init = 90 ;
int pos_unload = 0 ; 

void unload_atom() {
  servo_tower.write(pos_unload); 
  delay(500); 
  servo_tower.write(pos_init); 
}

void setup() {
  servo_tower.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  //read topics to unload when called 
}
