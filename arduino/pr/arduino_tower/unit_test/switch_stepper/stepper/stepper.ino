//-----------------------------------------------
// Arduino : Unite test 
// Test : stepper et button test 
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

int buttonPin_up = 2 ; 
int buttonPin_down = 3 ; 


void setup(){
  servo_unload.attach(22);

  // stepper init 
  stepper.enable() ; 
  //stepper.moveStep(50,true);

  // switch init 
  pinMode(buttonPin_up, INPUT); 
  pinMode(buttonPin_down, INPUT); 
}



void simple_test_up() {
  //stepper.moveRevolution(8,0);  // 0 go up 
  stepper.moveStep(1800,1) ; 
  while( stepper.getRemainingStep() !=0 ) {
    stepper.update() ; 
  }
  delay(800) ; 
}





// Use of switch : 

int test = 0 ; 

void test_button() {
  if (test == 0 ) {
    up() ; 
    test = 1 ; 
  }
}


void test_up_down() {
  up() ; 
  down() ; 
}


void up () { 
  while ( digitalRead(buttonPin_up) == LOW ) {
    stepper.moveStep(5,0); 
    while( stepper.getRemainingStep() !=0 ) {
      stepper.update() ; 
    }
  }
}


void down () { 
  while ( digitalRead(buttonPin_down) == LOW ) {
    stepper.moveStep(5,1); 
    while( stepper.getRemainingStep() !=0 ) {
      stepper.update() ; 
    }
  }
}


// Constants 
#define H_GROUND 0 
#define H_SAS_LOW 2000 
#define H_FLOOR_1 500 

int lift_position = 0 ; 


void up_down_variables() {
  // up floor 1 
  up_down(H_FLOOR_1) ; 
  // down ground 
  up_down(H_GROUND); 
  // up sas 
  up_down(H_SAS_LOW); 

} 

void up_down(int wanted_position) {  
  if ( lift_position > wanted_position && wanted_position >= H_GROUND ){  // go down 
    //nh.loginfo("goes down"); 
    stepper.moveStep(lift_position - wanted_position,true); 
    while( stepper.getRemainingStep() > 0 ) {
      stepper.update() ; 
    }
    delay(500) ; 
    lift_position = wanted_position ; 
    return 1 ; 
  }

  if (lift_position < wanted_position && wanted_position <= H_SAS_LOW ) { // go up 
    //nh.loginfo("goes up"); 
    stepper.moveStep(wanted_position-lift_position,false); 
    while( stepper.getRemainingStep() > 0 ) {
      stepper.update() ; 
    }
    delay(500) ; 
    lift_position = wanted_position ; 
    return 1 ; 
  }

  if (lift_position == wanted_position ) { //nothing 
    //nh.loginfo("do nothing"); 
    return 1 ; 
  }
  
  //nh.logwarn("error in moving"); 
  return 0 ; 
}



void loop(){
  //simple_test_up(); 
  //test_button() ; 
  test_up_down() ; 

}
