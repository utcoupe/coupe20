int Index;

#define EN  11
#define STEP 9 
#define DIR 10

void setup() 
{
  pinMode(EN, OUTPUT); //Enable
  pinMode(STEP, OUTPUT); //Step
  pinMode(DIR, OUTPUT); //Direction

  digitalWrite(EN,LOW);
}

void loop() 
{
  digitalWrite(DIR,HIGH);

  for(Index = 0; Index < 2000; Index++)
  {
    digitalWrite(STEP,HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP,LOW);
    delayMicroseconds(500);
  }
  delay(1000);

  digitalWrite(DIR,LOW);

  for(Index = 0; Index < 2000; Index++)
  {
    digitalWrite(STEP,HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP,LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
