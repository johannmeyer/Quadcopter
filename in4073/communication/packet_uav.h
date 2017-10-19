/*
   Johann Meyer
 */
#ifndef PACKET_UAV_H
#define PACKET_UAV_H

#include "packet.h"
void decode(core **logUserIn);
void encode(packet *my_packet, uint8_t packet_type);
#endif // PACKET_UAV_H
