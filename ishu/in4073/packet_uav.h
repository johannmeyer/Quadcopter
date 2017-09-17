/*
   Johann Meyer
 */
#ifndef PACKET_UAV_H
#define PACKET_UAV_H

#include "packet.h"
int decode(uint8_t *mode, int8_t *roll, int8_t *pitch, int8_t *yaw, uint8_t *lift);

#endif // PACKET_UAV_H
