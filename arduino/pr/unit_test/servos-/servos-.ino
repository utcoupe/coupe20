//-----------------------------------------------
// Arduino : Unite test 
// Test : Servo uniquement 
// Author : Floriane ALLAIRE May 2019 , UTCoupe 
// Need nothing else 
//------------------------------------------------

int servo_door_1_pin      = 10 ; // ? left door 
int servo_door_2_pin      = 11 ; // ? right door 
int servo_flipper_1_pin   = 13 ; // ? left flipper 
int servo_flipper_2_pin   = 12 ; // ? right flipper 


// ~ Constant ~ 
#define POS_FLIPPER_1_IN        0     // left flipper 
#define POS_FLIPPER_1_OUT       50   
#define POS_FLIPPER_2_IN        150   // right flipper 
#define POS_FLIPPER_2_OUT       50  
#define POS_DOOR_1_CLOSE        100   // left door 
#define POS_DOOR_1_OPEN         0 
#define POS_DOOR_2_CLOSE        50    // ? right door  
#define POS_DOOR_2_OPEN         50    // ? 



#include <Servo.h> 
Servo servo_door_1 ;
Servo servo_door_2 ; 
Servo servo_flipper_1 ;
Servo servo_flipper_2 ;  


void setup() {
  servo_door_1.attach(servo_door_1_pin);   //22 // number 1 //pins to control servo 
  servo_door_2.attach(servo_door_2_pin);  // number 2 
  servo_flipper_1.attach(servo_flipper_1_pin) ; 
  servo_flipper_2.attach(servo_flipper_2_pin) ; 

}

void loop() {
  servo_door_2.write(50); 
  delay(500);
  servo_door_2.write(150); 
  delay(500); 
}
