/*
   Johann Meyer
 */
#include "packet.h"

void generate_core(core *my_packet_core, uint8_t mode, int8_t roll, int8_t pitch, int8_t yaw, uint8_t lift)
{
        // Construct Packet Header
        struct header my_packet_header;
        my_packet_header.mode = mode;

        // Construct Packet Body
        struct body my_packet_body;
        my_packet_body.roll=roll;
        my_packet_body.pitch=pitch;
        my_packet_body.yaw=yaw;
        my_packet_body.lift=lift;

        my_packet_core->header = my_packet_header;
        my_packet_core->body = my_packet_body;
}
