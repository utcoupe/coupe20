#ifndef PINS_H
#define PINS_H

// nano
#ifdef __AVR_ATmega328P__
#define MOTOR1_EN 7
#define MOTOR2_EN 12

#define MOTOR1_SPD 9
#define MOTOR2_SPD 10

#define MOTOR1_BRK 8
#define MOTOR2_BRK 11

#define PIN_ENC_LEFT_A 2
#define PIN_ENC_LEFT_B 4
#define PIN_ENC_RIGHT_A 3
#define PIN_ENC_RIGHT_B 5

#define INTERRUPT_ENC_LEFT_A 0
#define INTERRUPT_ENC_RIGHT_A 1

#define PIN_SHARP_FORWARD 6
#define PIN_SHARP_BACKWARD 7

#define LED_MAINLOOP 14
#define LED_DEBUG 13
#define LED_BLOCKED 14
#endif

// mega
#ifdef __AVR_ATmega2560__
#define MOTOR1_EN 30
#define MOTOR2_EN 34

#define MOTOR1_SPD 3
#define MOTOR2_SPD 2

#define MOTOR1_RDY 32
#define MOTOR2_RDY 36

#define PIN_ENC_LEFT_A 19
#define PIN_ENC_LEFT_B 18
#define PIN_ENC_RIGHT_A 20
#define PIN_ENC_RIGHT_B 21

#define PIN_SHARP_FORWARD 14
#define PIN_SHARP_BACKWARD 15

#define INTERRUPT_ENC_LEFT_A 4
#define INTERRUPT_ENC_LEFT_B 5
#define INTERRUPT_ENC_RIGHT_A 3
#define INTERRUPT_ENC_RIGHT_B 2

//#define PIN_JACK 52
//#define LED_JACK 51
//
#define LED_DEBUG 16
#define LED_BLOCKED 16
#define LED_MAINLOOP 16
#endif

#endif
