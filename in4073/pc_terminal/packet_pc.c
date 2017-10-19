/*
   Johann Meyer
 */
#include "packet_pc.h"
#include "pc_terminal.h"
#include <stdio.h>
#include <string.h>

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

        encode_header_pc_uav(&my_packet_core->header, mode, p_incrementer);
        // Package core of message for CRC function
        encode_data_command(my_packet_core->body, roll_c, pitch_c, yaw_c,
                            lift_c);
        uint8_t crc =
            crc_message((void *)my_packet_core, sizeof(*my_packet_core));

        my_packet->start = START_BYTE;
        my_packet->crc = crc;
}

void decode()
{
        static int buffer[7];
        static uint8_t buffer_size = 0;
        int c = rs232_getchar_nb();
        if (c == -1) // Data available
                return;

        switch (c)
        {
        case START_BYTE:
                buffer_size = 1;
                break;
        default:
                if (buffer_size > 0) // Only read if start byte has been read
                        buffer[buffer_size++] = c;
                if (buffer_size == 7)
                {
                        core my_packet_core;
                        my_packet_core.header = buffer[1];
                        int *data = &buffer[2];
                        memcpy(my_packet_core.body, data, BODY_LENGTH);
                        // CRC calculation
                        uint8_t tmp_crc = buffer[2 + BODY_LENGTH - 1];
                        uint8_t crc = crc_message((void *)&my_packet_core,
                                                  sizeof(my_packet_core));
                        if (crc == tmp_crc)
                        {
                                uint8_t mode, packet_type;
                                decode_header_uav_pc(&my_packet_core.header,
                                                     &mode, &packet_type);

                                fprintf(stderr, "Mode: %d | ", mode);
                                switch (packet_type)
                                {
                                case PACKET_TYPE_GAINS1:
                                        fprintf(stderr,
                                                "P: %d | P1: %d | P2: %d",
                                                data[P_OFFSET], data[P1_OFFSET],
                                                data[P2_OFFSET]);
                                        break;
                                case PACKET_TYPE_GAINS2:
                                        fprintf(
                                            stderr,
                                            "P3: %d | P4: %d | C1: %d | C2: %d",
                                            data[P3_OFFSET], data[P4_OFFSET],
                                            data[C1_OFFSET], data[C2_OFFSET]);
                                        break;
                                }
                        }
                        buffer_size = 0; // reset buffer
                }
                break;
        }
}
