//
// Created by tfuhrman on 09/05/17.
//

#ifndef ASSERV_PROTOCOL_H
#define ASSERV_PROTOCOL_H

#include "parameters.h"
#include <stdint.h>


class String;

// 	COMMANDS :
// 'ordre;id;arg1;arg2;argn'
//  For example :
// 'c;3;120;1789;31400'
// 'j;0;'
// issues the order GOTOA with
// ID 3 to X=120, Y=1789 and angle = 3.14
//
// WARNING : order should be
//  ONE ascii char long
//
//  float are transmitted as integer
//  therefore any number refered as 
//  "decimal" is actually an int
//  multiplied by FLOAT_PRECISION

// BEGIN_ORDERS - Do not remove this comment
#define START       'S'     //no args, start the program
#define HALT        'H'     //no args, halt the program
#define	GOTOA 		'c' 	// x(int);y(int);a(decimal);direction(int) - (mm and radian), direction is optionnal : 1 is forward, -1 is backward, 0 is any
#define	GOTO 		'd' 	// x(int);y(int);direction(int) - (mm), direction is optionnal : 1 is forward, -1 is backward, 0 is any
#define	ROT 		'e' 	// a(decimal) - (radian), can't turn more than 1 turn
#define ROTNOMODULO	'a' 	// a(decimal) - radian, can turn more than 1 turn
#define	KILLG 		'f' 	// no args, go to next order
#define	CLEANG 		'g' 	// no args, cancel all orders
#define	PIDLEFT		'p' 	// p(decimal);i(decimal);d(decimal) - set left PID
#define	PIDRIGHT	'i' 	// p(decimal);i(decimal);d(decimal) - set right PID
#define PIDALL 		'u' 	// p(decimal);i(decimal);d(decimal) - set both PID
#define	GET_CODER 	'j' 	// no args, response : l(long);r(long)
#define	PWM 		'k' 	// l(int);r(int);duration(int) - set left and right pwm for duration ms
#define	SPD 		'b' 	// l(int);a(int);duration(int) - set linear and angular spd for duration ms
#define	ACCMAX 		'l' 	// a(int) - set max acceleration (mm/s-2)
#define	SPDMAX 		'x' 	// v(int),r(decimal) - set max spd (mm/s) and rotation ratio
#define	SET_POS		'm' 	// x(int);y(int);a(decimal) - set pos (mm / radians)
#define	GET_POS		'n' 	// no args, response : x(int);y(int);a(decimal) - get current pos (mm and radians)
#define GET_SPD 	'y' 	// no args, respond : l(int);r(int) - get wheels speed (mm/s)
#define GET_TARGET_SPD 	'v'	// no args, respond : l(int);r(int) - get target wheels speed (mm/s)
#define	GET_POS_ID 	'o'		// no args, response : x(int);y(int);a(decimal);id(int) - get current pos and last id (mm and radians)
#define GET_LAST_ID	't' 	// no args, response : id(int)
#define	PAUSE 		'q' 	// no args, pauses control
#define	RESUME 		'r'		// no args, resumes control
#define RESET_ID 	's' 	// no args, reset last finished id to 0
#define PINGPING 	'z'		// no args, switch led state
#define WHOAMI 		'w' 	// no args, answers 'ASSERV' or 'PAP'
#define SETEMERGENCYSTOP 'A'	// enable(int)
// END_ORDERS - Do not remove this comment

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

extern unsigned char flagArduinoConnected;
void parseAndExecuteOrder(const String& order);
uint8_t getLog10(const uint16_t number);
void ProtocolAutoSendStatus();
void emergencyStop(const uint8_t enable);

#endif //ASSERV_PROTOCOL_H
