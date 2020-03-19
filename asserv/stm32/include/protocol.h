//
// Created by tfuhrman on 09/05/17.
//

#ifndef ASSERV_PROTOCOL_H
#define ASSERV_PROTOCOL_H

// #include <stdint.h>
#include "shared_asserv/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t getLog10(const uint16_t number);
void ProtocolAutoSendStatus();

#ifdef __cplusplus
}
#endif

#endif //ASSERV_PROTOCOL_H

