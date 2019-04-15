//
// Created by Floriane ALLAIRE on 10/03/2019
//

#ifndef ARDUINO_TOWER_PROTOCOL_H
#define ARDUINO_TOWER_PROTOCOL_H

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
#define INIT            'I'     //no args, initialize the arduino , respond : init_success(bool)
#define TOWER_UNLOAD    'u'     // args : unload_content(int)   1: slider 2:pliers , respond :  unload_success(bool);nb_atom_out(float)
#define	TOWER_LOAD		'l' 	// args : load_content (int),load_content_nb(int) 1: atom 2:atoms  , respond :  load_success (float);nb_atom_in(float)
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

//extern unsigned char flagArduinoConnected;
//void parseAndExecuteOrder(const String& order);
//uint8_t getLog10(const uint16_t number);
//void ProtocolAutoSendStatus();
//void emergencyStop(const uint8_t enable);

#endif //ARDUINO_TOWER_PROTOCOL_H