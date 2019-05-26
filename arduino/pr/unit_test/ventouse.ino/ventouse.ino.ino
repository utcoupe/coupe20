//-----------------------------------------------
// Arduino : Unite test 
// Test : Servo uniquement 
// Author : Floriane ALLAIRE May 2019 , UTCoupe 
// Need nothing else 
//------------------------------------------------


int ventouse_pin_1 = 6 ; 
int ventouse_pin_2 = 7 ; 

 // ~ CONSTANT ~


void setup() { 
  pinMode(ventouse_pin_1,INPUT); 
  pinMode(ventouse_pin_2,INPUT);   
    
 }


void loop() {
    digitalWrite(ventouse_pin_1,HIGH); 
    
    digitalWrite(ventouse_pin_2,LOW); 
    delay(5000); 
    digitalWrite(ventouse_pin_2,HIGH); 
    delay(5000); 
}
