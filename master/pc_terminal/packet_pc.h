/*
   Johann Meyer
 */

#ifndef PACKET_PC_H
#define PACKET_PC_H

#include "../packet.h"
void encode(packet *my_packet, uint8_t mode, int16_t roll_r, int16_t pitch_r, int16_t yaw_r, int16_t lift_r);

#endif //PACKET_PC_H
