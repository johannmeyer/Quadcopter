/*
   Johann Meyer
 */
#include "packet_pc.h"
#include "pc_terminal.h"
#include <stdio.h>
#include <string.h>

/*
Encode packets to be sent to the UAV
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
        // Bit shifting is not used to avoid 0xFF aka START_BYTE
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

        // Calculate CRC
        uint8_t crc = crc_message((void *)my_packet_core, sizeof(core));

        my_packet->start = START_BYTE;
        my_packet->crc = crc;
}

/*
Decode packets received from the UAV
 */
void decode()
{
        // Buffer to store bytes until all bytes have been received and the CRC
        // can be performed.
        static int     buffer[sizeof(packet)];
        static uint8_t buffer_size = 0;
        int            c = rs232_getchar_nb(); // Get character
        if (c == -1)                           // Check if data is available
                return;

        switch (c)
        {
        case START_BYTE:
                buffer_size = 1; // No need to actually store the START_BYTE
                break;
        default:
                if (buffer_size > 0) // Only read if start byte has been read
                        buffer[buffer_size++] = (uint8_t)c;

                // If entire message has arrived
                if (buffer_size == sizeof(packet))
                {
                        core my_packet_core;

                        // Read packet header
                        my_packet_core.header = buffer[1];

                        // Read packet body
                        for (int i = 0; i < BODY_LENGTH; i++)
                                my_packet_core.body[i] = (uint8_t)buffer[2 + i];

                        // CRC calculation
                        uint8_t tmp_crc = buffer[2 + BODY_LENGTH];
                        uint8_t crc =
                            crc_message((void *)&my_packet_core, sizeof(core));

                        if (crc == tmp_crc) // Check if CRCs match
                        {
                                uint8_t mode, packet_type;

                                // Decode header
                                decode_header_uav_pc(&my_packet_core.header,
                                                     &mode, &packet_type);

                                // Determine what the data means
                                switch (packet_type)
                                {
                                case PACKET_TYPE_GAINS1:;
                                        uint8_t P, P1, P2, battery;
                                        decode_data_gains1(my_packet_core.body,
                                                           &P, &P1, &P2,
                                                           &battery);
                                        fprintf(stderr, "Mode: %d | ", mode);
                                        fprintf(
                                            stderr,
                                            "P: %d | P1: %d | P2: %d | Battery: %d\n",
                                            P, P1, P2, battery);
                                        break;
                                case PACKET_TYPE_GAINS2:;
                                        uint8_t P3, P4;
                                        decode_data_gains2(my_packet_core.body,
                                                           &P3, &P4);
                                        fprintf(stderr, "Mode: %d | ", mode);
                                        fprintf(stderr, "P3: %d | P4: %d\n", P3,
                                                P4);
                                        break;
                                case PACKET_TYPE_ANGLES:;
                                        int8_t phi, theta, psi;
                                        decode_data_angles(my_packet_core.body,
                                                           &phi, &theta, &psi);
                                        fprintf(stderr, "Mode: %d | ", mode);
                                        fprintf(
                                            stderr,
                                            "phi: %d | theta: %d | psi: %d\n",
                                            phi, theta, psi);
                                        break;
                                case PACKET_TYPE_GYRO:;
                                        int8_t sp, sq, sr;
                                        decode_data_gyro(my_packet_core.body,
                                                         &sp, &sq, &sr);
                                        fprintf(stderr, "Mode: %d | ", mode);
                                        fprintf(stderr,
                                                "sp: %d | sq: %d | sr: %d\n",
                                                sp, sq, sr);
                                        break;
                                case PACKET_TYPE_ACCEL:;
                                        int8_t sax, say, saz;
                                        decode_data_accel(my_packet_core.body,
                                                          &sax, &say, &saz);
                                        fprintf(stderr, "Mode: %d | ", mode);
                                        fprintf(stderr,
                                                "sax: %d | say: %d | saz: %d\n",
                                                sax, say, saz);
                                        break;
                                case PACKET_TYPE_ACTUATOR:;
                                        int8_t ae[4];
                                        decode_data_motor(my_packet_core.body,
                                                          ae);
                                        fprintf(stderr, "Mode: %d | ", mode);
                                        fprintf(
                                            stderr,
                                            "ae[0]: %d | ae[1]: %d | ae[2]: %d | ae[3]: %d\n",
                                            ae[0], ae[1], ae[2], ae[3]);
                                        break;
                                case PACKET_TYPE_PRINTF:;
                                        char str[4];
                                        for (int i = 0; i < 4; i++)
                                                str[i] = my_packet_core.body[i];
                                        fprintf(stderr, "%s", str);
                                        break;
                                default:
                                        fprintf(
                                            stderr,
                                            "Packet type not mapped on PC\n");
                                        break;
                                }
                        }
                        buffer_size = 0; // reset buffer
                }
                break;
        }
}
