//Code très bien mais que dans le même sens 
//Fluide dans le mouvement 

#include "PololuA4983.h"


#include <Servo.h> 
Servo servo_unload ;

#define POS_UNLOAD_INIT  100 //90   
#define POS_UNLOAD       73  //store servo position

int dir_pin = 10;
int step_pin = 9;

int en_pin = 11;
uint16_t min_delay = 1000;
PololuA4983 stepper = PololuA4983(step_pin,dir_pin, en_pin, min_delay);


int step_motor = 0 ; 
 int i =  0 ;


void setup(){
  servo_unload.attach(22);


  stepper.enable() ; 
  //stepper.moveStep(50,true);
}



void test() {
  
  int i = 0 ; 
  if (stepper.getRemainingStep()>0 ) {
    
  }
  else {
    delay(500); 
    if ( i < 5 ) {
      step_motor += 10 ; 
      i += 1 ; 
    }
  }
}



void test_2() {
   stepper.update();
  if (stepper.getRemainingStep() !=0 ) {
    //wait ; 
  }
  else {
    delay(600) ; 
    if (i==0) {
      stepper.moveStep(1850,true); 
    }
    if (i==1) {
      stepper.moveStep(1850,true); 
    } 
    if (i==1) {
      stepper.moveStep(850,true); 
    } 
     i+=1 ; 
   }
}


void test_3() {
  //stepper.moveRevolution(8,1);
  stepper.moveStep(500,0); 
  while( stepper.getRemainingStep() !=0 ) {
    stepper.update() ; 
  }
  delay(1000) ; 
}




void loop(){
  //stepper.disable() ; 
  test_3(); 
  

}
