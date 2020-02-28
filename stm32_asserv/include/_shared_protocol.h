/**
 * Author : Paul Constant
 * Date : 22/12/29
 * 
 * This file is used in both stm32_asserv and asserv_simu.py.
 * It regroups all functions linked to the protocol of the robot.
 * 
 * A shared library is generated from this file and imported in the python 
 * code of the simulation. This way, the stm32 command law can be tested without 
 * the robots.
 * The file is imported like any other file in stm32_asserv.
 *
 * Be careful when editing : 
 * 
 * - If you change a function which is imported in the simulation asserv (name, parameters, return value),
 *   update the simulation asserv so that it does not break.
 * 
 * Functions imported in the simulation asserv (the ones which could make it break): 
 * - Every function from this file.
 * 
 * 
 * - If you change the name of this file, make the change in the compile.sh script as well.
**/

#ifndef _SHARED_PROTOCOL_H
#define _SHARED_PROTOCOL_H

#include <stdint.h>

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
#define	GOTOA 		'c' 	// x(int);y(int);a(decimal);direction(int) - (mm and radian), direction is optionnal : 1 is forward, -1 is backward, 0 is any; slow_go(bool)
#define	GOTO 		'd' 	// x(int);y(int);direction(int) - (mm), direction is optionnal : 1 is forward, 0 is backward, 2 is any; slow_go(bool)
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
#define	SET_POS		'm' 	// x(int);y(int);a(decimal);mode(int) - set pos (mm / radians)
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

//SET_POS mode bits - similar to status bits in control.h
#define BIT_MODE_A   (1<<0)
#define BIT_MODE_Y   (1<<1)
#define BIT_MODE_X   (1<<2)

extern uint8_t flagSTM32Connected;

#ifdef __cplusplus
extern "C" {
#endif

void start();
void halt();

void parseGOTO(char *receivedOrderPtr, int order_id);
void parseGOTOA(char *receivedOrderPtr, int order_id);
void parseROT(char *receivedOrderPtr, int order_id);
void parseROTNMODULO(char *receivedOrderPtr, int order_id);
void parsePWM(char *receivedOrderPtr, int order_id);
void parseSPD(char *receivedOrderPtr, int order_id);
void parsePIDALL(char *receivedOrderPtr);
void parsePIDRIGHT(char *receivedOrderPtr);
void parsePIDLEFT(char *receivedOrderPtr);
void parseSETPOS(char *receivedOrderPtr);
void parseSPDMAX(char *receivedOrderPtr);
void parseACCMAX(char *receivedOrderPtr);
void parseSETEMERGENCYSTOP(char *receivedOrderPtr);

void emergencyStop(int enable);
void resetID();

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_SHARED_H