//-----------------------------------------------
// Arduino : Unite test 
// Test : Servo uniquement 
// Author : Floriane ALLAIRE May 2019 , UTCoupe 
// Need nothing else 
//------------------------------------------------

#include <Servo.h> 
Servo servo_1 ;
Servo servo_2 ; 


#define POS_UNLOAD_INIT  130 //90   
#define POS_UNLOAD       74  //store servo position

#define POS_INIT_1 100 // 95 
#define POS_1      15 // 0 

#define POS_INIT_2 25   //15  // 50 
#define POS_2      100 //80  //170 


void setup() {
  servo_1.attach(12);   //22 // number 1 //pins to control servo 
  servo_2.attach(13);  // number 2 
}

void loop() {
  servo_1.write(POS_INIT_1); 
  servo_2.write(POS_INIT_2); 
  // Close 
  delay(500); 
  servo_1.write(POS_1); 
  servo_2.write(POS_2); 
  // Open 
  delay(5000); 
}
