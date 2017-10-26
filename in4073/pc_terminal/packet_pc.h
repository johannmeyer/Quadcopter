/*
   Johann Meyer
 */

#ifndef PACKET_PC_H
#define PACKET_PC_H

#include "packet.h"
#include "crc.h"
#include <stdint.h>

void encode(packet *my_packet, uint8_t mode, uint8_t p_incrementer);
void decode();

#endif //PACKET_PC_H
