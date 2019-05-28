//-----------------------------------------------
// Arduino : Unite test 
// Test : Servo uniquement 
// Author : Floriane ALLAIRE May 2019 , UTCoupe 
// Need nothing else 
//------------------------------------------------


int sucker_pin_1 = 6 ; 
int sucker_pin_2 = 7 ; 

 // ~ CONSTANT ~


void setup() { 
  pinMode(sucker_pin_1,INPUT); 
  pinMode(sucker_pin_2,INPUT);   
    
 }


void loop() {
    digitalWrite(sucker_pin_1,HIGH); 
    
    //digitalWrite(sucker_pin_2,HIGH); // off 
    delay(5000); 
    digitalWrite(sucker_pin_2,LOW); // on  
    delay(5000); 
}
