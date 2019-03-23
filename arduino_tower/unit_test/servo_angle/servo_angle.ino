
#include <Servo.h> 
Servo servo_1 ;
Servo servo_2 ; 


#define POS_UNLOAD_INIT  130 //90   
#define POS_UNLOAD       74  //store servo position

#define POS_INIT_1 130 
#define POS_1      50

#define POS_INIT_2 15
#define POS_2      80


void setup() {
  servo_1.attach(22);  //pins to control servo 
  servo_2.attach(24); 
}

void loop() {
  servo_1.write(POS_INIT_1); 
  servo_2.write(POS_INIT_2); 
  delay(5000); 
  servo_1.write(POS_1); 
  servo_2.write(POS_2); 
  delay(700); 
}
