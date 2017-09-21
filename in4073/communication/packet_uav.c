/*
   Johann Meyer
 */

#include "crc.h"
#include "in4073.h"
#include "packet_uav.h"
#include <string.h>

/*
   UAV-side decoding
 */

int decode(core **logUserIn)
{
        /*
           TODO create another dequeue to support sizeof(). e.g. sizeof(body)
         */
        int discard_count = 0;

        // Wait until entire packet has arrived
        while (rx_queue.count >= sizeof(packet))
        {
                uint8_t curr_byte;
                // Read next byte and verify that it is the START_BYTE.
                if ((curr_byte = (uint8_t)dequeue(&rx_queue)) == START_BYTE)
                {
                        // Temporarily store the data in temp variables so that
                        // a CRC check can first be performed.
                        uint8_t header = (uint8_t)dequeue(&rx_queue);
                        uint8_t data[BODY_LENGTH];
                        for (int i = 0; i < BODY_LENGTH; i++)
                                data[i] = (uint8_t)dequeue(&rx_queue);

                        core my_packet_core;
                        my_packet_core.header = header;
                        memcpy(my_packet_core.body, data, sizeof(data));

                        // CRC calculation
                        uint8_t tmp_crc = (uint8_t)dequeue(&rx_queue);
                        uint8_t crc = crc_core(&my_packet_core); //TODO change to, sizeof(my_packet_core));

                        if (crc == tmp_crc)
                        {
                                uint8_t packet_type;
                                decode_header(&header, &mode, &packet_type);
                                switch (packet_type)
                                {
                                case PACKET_TYPE_COMMAND:
                                        decode_data_command(data, &roll, &pitch,
                                                            &yaw, &lift);
                                        break;
                                case PACKET_TYPE_GAINS:
                                        decode_data_gains(data, &P, &P1, &P2);
                                        break;
                                default:
                                        printf("Unknown Packet Type\n");
                                        break;
                                }
                                // Log the incoming data
                                memcpy(*logUserIn, &my_packet_core,
                                       sizeof(core));
                        }
                        else
                        {
                                discard_count++;
                                printf("Packet transmission error\n");
                        }
                }
        }

        return discard_count;
}
