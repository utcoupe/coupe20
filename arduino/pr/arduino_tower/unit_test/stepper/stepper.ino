//-----------------------------------------------
// Arduino : Unite test 
// Test : Button uniquement 
// Author : Floriane ALLAIRE May 2019 , UTCoupe 
// Need PololuA4984.h et PololuA4984.ino
//------------------------------------------------


#include "PololuA4983.h"


#include <Servo.h> 
Servo servo_unload ;

#define POS_UNLOAD_INIT  100 //90   
#define POS_UNLOAD       73  //store servo position


// -----------------------------
// ------- Pin for stepper -----
// -----------------------------  
int dir_pin = 10;
int step_pin = 9;
int en_pin = 11;
uint16_t min_delay = 1000;
PololuA4983 stepper = PololuA4983(step_pin,dir_pin, en_pin, min_delay);

void setup(){

  // stepper init 
  stepper.enable() ; 
  //stepper.moveStep(50,true);

}



void simple_test_up() {
  //stepper.moveRevolution(8,0);  // 0 go up 
  stepper.moveStep(1800,1) ; 
  while( stepper.getRemainingStep() !=0 ) {
    stepper.update() ; 
  }
  delay(800) ; 
}




void loop(){
  simple_test_up(); 

}
