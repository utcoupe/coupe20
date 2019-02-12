//
// Created by tfuhrman on 22/04/18.
//


// D[numbre] <=> https://escapequotes.net/wp-content/uploads/2016/02/d1-mini-esp8266-board-sh_fixled.jpg

#ifndef ARDUINO_PINS_H
#define ARDUINO_PINS_H

// LCD pins
#define LCD_SDA 19 //D2 //can not use another one
#define LCD_SCK 20 //D1 //can not use another one

// Buttons matrix pins
#define BTN_L1 6 //D6
#define BTN_L2 7 //D7
#define BTN_R1 5 //D5
#define BTN_R2 4 //D0

// LED pins
#define LED_RED       18 //D3
//#define LED_BLUE    D4
#define LED_GREEN     16 //D8  

#define JACK	        17 //D4
//#define EMERGENCY     D8

#endif //ARDUINO_PINS_H
