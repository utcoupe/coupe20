// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
// Adapté pour UTCoupe2011 par Arthur, 19/01/2011
// Moteurs 1 et 2 correspondent à la carte Rugged


#include <avr/io.h>
#include <Arduino.h>


#include "AFMotor.h"


/******************************************
               MOTORS
******************************************/
inline void initPWM1(uint8_t freq) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2B (pin 3)
    TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
    TCCR2B = freq & 0x7;
    OCR2B = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 3 is now PE5 (OC3C)
    TCCR3A |= _BV(COM1C1) | _BV(WGM10); // fast PWM, turn on oc3c
    TCCR3B = (freq & 0x7) | _BV(WGM12);
    OCR3C = 0;
	
#else
   #error "This chip is not supported!"
#endif

    pinMode(3, OUTPUT);
}

inline void setPWM1(uint8_t s) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2A on PB3 (Arduino pin #11)
    OCR2B = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 11 is now PB5 (OC1A)
    OCR3C = s;
#else
   #error "This chip is not supported!"
#endif
}

inline void initPWM2(uint8_t freq) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2A on PB3 (Arduino pin #11)
    TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
    TCCR2B = freq & 0x7;
    OCR2A = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 11 is now PB5 (OC1A)
    TCCR1A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc1a
    TCCR1B = (freq & 0x7) | _BV(WGM12);
    OCR1A = 0;
#else
   #error "This chip is not supported!"
#endif
    pinMode(11, OUTPUT);
}

inline void setPWM2(uint8_t s) {
#if defined(__AVR_ATmega8__) || \
    defined(__AVR_ATmega48__) || \
    defined(__AVR_ATmega88__) || \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
    // use PWM from timer2A on PB3 (Arduino pin #11)
    OCR2A = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // on arduino mega, pin 11 is now PB5 (OC1A)
    OCR1A = s;
#else
   #error "This chip is not supported!"
#endif
}

AF_DCMotor::AF_DCMotor(uint8_t num, uint8_t freq) {
  motornum = num;
  pwmfreq = freq;

  switch (num) {
  case 1:
	pinMode(MOTOR1_DIR, OUTPUT);
	digitalWrite(MOTOR1_DIR, LOW);
    initPWM1(freq);
    break;
  case 2:
	pinMode(MOTOR2_DIR, OUTPUT);
	digitalWrite(MOTOR2_DIR, LOW);
    initPWM2(freq);
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t dirPin;
  switch (motornum) {
  case 1:
    dirPin = MOTOR1_DIR; break;
  case 2:
    dirPin = MOTOR2_DIR; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
	digitalWrite(dirPin, LOW);
    break;
  case BACKWARD:
	digitalWrite(dirPin, HIGH);
    break;
  case RELEASE:
    //TODO
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
  switch (motornum) {
  case 1:
    setPWM1(speed); break;
  case 2:
    setPWM2(speed); break;
  }
}


