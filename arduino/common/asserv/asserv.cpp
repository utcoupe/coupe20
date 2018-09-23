/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 13/10/13			*
 ****************************************/
  
#include <Arduino.h>
#include "block.h"
#include "compat.h"
#include "parameters.h"
#include "protocol.h"
#include "control.h"
#include "pins.h"
#include "sender.h"
#include <Timer.h>

/**
 * Main loop function, check if emergency stop and computes the new command to apply.
 * The asserv is working with an internal fifo to execute orders.
 * The loop is activate through a Timer (see arduino library).
 */
void asservLoop();
void asservStatus();

// Run the loop for asserv at 100 Hz
Timer asservLoopTimer = Timer(10, &asservLoop);
Timer asservStatusTimer = Timer(100, &asservStatus);

/**
 * Read a \n ending string from serial port.
 * The timeout is 50ms.
 * When a string is received, execute the corresponding order.
 */
void serialRead() {
    String receivedString;
    if (Serial.available() > 0) {
        receivedString = Serial.readStringUntil('\n');
        // SerialSender::SerialSend(SERIAL_INFO, receivedString);
        receivedString.replace("\n", "");
        if (receivedString != "") {
            parseAndExecuteOrder(receivedString);
        }
    }
}

/**
 * Arduino setup function, initialize pins and registers.
 */
void setup() {
#ifdef __AVR_ATmega32U4__
    Serial.begin(BAUDRATE);
#else
    Serial.begin(BAUDRATE, SERIAL_TYPE);
#endif
    Serial.setTimeout(5);

#ifdef __AVR_ATmega2560__
	TCCR3B = (TCCR3B & 0xF8) | 0x01 ;
	TCCR1B = (TCCR1B & 0xF8) | 0x01 ;
#else
#ifdef __AVR_ATmega328P__
	TCCR1B = (TCCR1B & 0xF8) | 0x01 ;
#endif
#endif
	initPins();
	ControlInit();

    asservLoopTimer.Start();
    asservStatusTimer.Start();
}

/**
 * Arduino loop function, read from serial port and send internal serial port data.
 * If it is the time to execute asserv, execute it.
 */
void loop() {
    if (!flagArduinoConnected) {
        SerialSender::SerialSend(SERIAL_INFO, "%s", ARDUINO_ID);
        serialRead();
    } else {
        asservLoopTimer.Update();
        asservStatusTimer.Update();
    }
    SerialSender::SerialSendTask();
    if (!flagArduinoConnected) {
        delay(1000);
    }
}

void asservLoop() {
	//Action asserv
	ComputeIsBlocked();
	ControlCompute();

    // That's an ugly way to do it, but not working in another way...
    // lastReachedID is defined in control.h file
    if(lastReachedID != 0) {
        SerialSender::SerialSend(SERIAL_INFO, "%d;", (int)lastReachedID);
        lastReachedID = 0;
    }
}

void asservStatus() {
    serialRead();
    ProtocolAutoSendStatus();
}
