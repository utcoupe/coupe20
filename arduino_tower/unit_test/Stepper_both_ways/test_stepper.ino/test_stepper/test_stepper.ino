// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

byte directionPin = 10;
byte stepPin = 9;
int numberOfSteps = 9000;
byte ledPin = 11;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 1000 ; //250; // milliseconds - or try 1000 for slower steps


void setup() { 

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);
  
  //delay(2000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
 
}

void loop() { 
  digitalWrite(directionPin, HIGH);
  
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    //delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
    //digitalWrite(stepPin, LOW);
    
    delay(millisbetweenSteps);
    
    digitalWrite(ledPin, !digitalRead(ledPin));
  }

}
