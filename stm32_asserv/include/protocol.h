//
// Created by tfuhrman on 09/05/17.
//

#ifndef ASSERV_PROTOCOL_H
#define ASSERV_PROTOCOL_H

#include "parameters.h"
#include <stdint.h>
#include "canSender.h"
#include "serial.h"

#define ALL_CAN_ADDR		0
#define BBB_CAN_ADDR		1
#define STM_CAN_ADDR		2
#define ARDUINO_CAN_ADDR	3
#define ZIGBEE_CAN_ADDR		4

#define STOP 				0
#define START 				1
#define PAUSE 				2
#define RESUME 				3
#define RESET_ID 			4
#define SETEMERGENCYSTOP 	5
#define NEXT_ORDER			6
#define RESET_ORDERS		7
#define UNSETEMERGENCYSTOP	8

#define HANDSHAKE			0
#define WHOAMI				1
#define SET_MODE			3
#define SPEED 				4
#define GET_CODER			5
#define MANAGEMENT			7
#define GOTOA				8
#define GOTO 				9
#define ROT 				10
#define ROTNOMODULO 		11
#define PIDLEFT				12
#define PIDRIGHT			13
#define PIDALL				14
#define PWM 				15
#define SET_POS				16
#define SET_PARAM			17
#define CURRENT_POS 		18
#define CURRENT_PWM			19
#define CURRENT_SPD			20
#define MOVE_PLIERS 		21
#define CLOSE_OPEN_PLIERS	22
#define SONAR_DISTANCE		23
#define THROW_BALLS			24
#define OBJECT_ON_MAP		25
#define ORDER_COMPLETED		26
#define SET_SERVO			27
#define ROBOT_BLOCKED		28

// END CAN ORDERS
#define AUTO_SEND	'~'		// x(int);y(int);a(decimal)
#define JACK 	'J'

#define JACK_SEND_NR 5
#define FLOAT_PRECISION 1000.0
#define FAILED_MSG "FAILED"
#define MAX_COMMAND_LEN 60
#define MAX_ID_VAL 32767
#define MAX_ID_LEN 5
#define ID_START_INDEX 2
#define MAX_RESPONSE_LEN 50

#define MAX_BYTES_PER_IT (0.9*BAUDRATE/(HZ*10))

#define ORDER_INDEX (uint8_t)0
#define ID_INDEX    (uint8_t)2

#ifdef DEBUG_TARGET_SPEED
#define MAX_AUTOSEND_SIZE (48)
#else
#define MAX_AUTOSEND_SIZE (24)
#endif

extern bool flagConnected;
extern Serial g_serial;
void parseAndExecuteOrder(uint8_t* message);
uint8_t getLog10(const uint16_t number);
void ProtocolAutoSendStatus();

#endif //ASSERV_PROTOCOL_H
