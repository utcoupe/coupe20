//-----------------------------------------------
// Arduino : Unite test 
// Test : Button uniquement 
// Author : Floriane ALLAIRE May 2019 , UTCoupe 
// Need nothing else 
//------------------------------------------------

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int buttonPin2 = 3 ; 
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600); 
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input
  pinMode(buttonPin,INPUT) ; 
  //pinMode(buttonPin,INPUT_PULLUP); 
  pinMode(buttonPin2,INPUT) ; 
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
    Serial.print("On ");
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
    Serial.print("Off "); 
  }
}
