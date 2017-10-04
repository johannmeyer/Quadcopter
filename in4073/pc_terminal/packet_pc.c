/*
   Johann Meyer
 */
#include "packet_pc.h"
#include "pc_terminal.h"
#include <stdio.h>

/*
   PC-side encoding
 */
void encode(packet *my_packet, uint8_t mode, uint8_t p_incrementer)
{
        core *my_packet_core = &my_packet->packet_core;

        /*
           Convert the raw joystick and keyboard input into byte
           values to be transmitted.
         */

        int8_t  roll_c, pitch_c, yaw_c;
        uint8_t lift_c;
        roll_c = (int8_t)(((float)roll / 32768) * 127);
        pitch_c = (int8_t)(((float)pitch / 32768) * 127);
        yaw_c = (int8_t)(((float)yaw / 32768) * 127);
        lift_c = (uint8_t)((((float)lift + 32767) / 65536) * 255);

        /*
           Construct the Packet
         */

        // Package core of message for CRC function
        encode_data_command(my_packet_core->body, roll_c, pitch_c, yaw_c,
                            lift_c);

        encode_header_pc_uav(&my_packet_core->header, mode, p_incrementer);
        uint8_t crc = crc_core(my_packet_core); // TODO update crc to use sizeof

        my_packet->start = START_BYTE;
        my_packet->crc = crc;
}
