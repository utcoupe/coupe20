//
// Created by tfuhrman on 09/05/17.
//

#ifndef ASSERV_PROTOCOL_H
#define ASSERV_PROTOCOL_H

#include <stdint.h>
#include "protocol_shared.h"


extern uint8_t flagSTM32Connected;
uint8_t getLog10(const uint16_t number);
void ProtocolAutoSendStatus();

#endif //ASSERV_PROTOCOL_H

