
#include <Servo.h> 
Servo servo_1 ;
Servo servo_2 ; 


#define POS_UNLOAD_INIT  130 //90   
#define POS_UNLOAD       74  //store servo position

#define POS_INIT_1 95
#define POS_1      0

#define POS_INIT_2 50 //15
#define POS_2      170 //80


void setup() {
  servo_1.attach(22);  // number 1 //pins to control servo 
  servo_2.attach(24);  // number 2 
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
