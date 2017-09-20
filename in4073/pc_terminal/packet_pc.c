/*
   Johann Meyer
 */
#include "packet_pc.h"
#include <stdio.h>
/*
   PC-side encoding
 */
void encode(packet *my_packet, uint8_t mode, int16_t roll_r, int16_t pitch_r, int16_t yaw_r, int16_t lift_r)
{

        // This line appears redundant but is here in case the mode needs to be
        // shrunk to 4 bits.
        uint8_t mode_c = mode;

        /*
           Convert the raw joystick and keyboard input into uint8_t values to be transmitted
         */
         int8_t roll_c, pitch_c, yaw_c;
         uint8_t lift_c;
         roll_c = (int8_t)(((float)roll_r/32768)*127);
         pitch_c =(int8_t)(((float)pitch_r/32768)*127);
         yaw_c = (int8_t)(((float)yaw_r/32768)*127);
         lift_c = (uint8_t)((((float)lift_r+32768)/65536)*255);
         //printf("roll_c: %x  roll_r : %x\n",roll_c, roll_r );
        /*
           Construct the Packet
         */

        // Package core of message for CRC function
        core my_packet_core;
        generate_core(&my_packet_core, mode_c, roll_c, pitch_c, yaw_c, lift_c);

        // TODO insert CRC calculation
        //uint8_t crc = 0xff; // TODO REMOVE
        uint8_t crc = crc_core(&my_packet_core);

        my_packet->start = START_BYTE;
        my_packet->packet_core = my_packet_core;
        my_packet->crc=crc;
        my_packet->end = END_BYTE;

}
