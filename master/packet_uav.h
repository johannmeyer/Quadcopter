/*
   Johann Meyer
 */
#ifndef PACKET_UAV_H
#define PACKET_UAV_H

#include "packet.h"
int decode(uint8_t *mode, uint8_t *roll, uint8_t *pitch, uint8_t *yaw, uint8_t *lift);

#endif // PACKET_UAV_H
