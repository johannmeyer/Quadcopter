/*
   Johann Meyer
 */

#include "crc.h"
#include "in4073.h"
#include "packet_uav.h"
#include <string.h>

// Determines packet loss limits between receives and disconnection limits
#define MAX_LOST_PACKETS 3
#define MAX_NO_DATA_AVAILABLE 2
#define TIMEOUT_THRESHOLD 300000
uint8_t discard_count = 0;
uint8_t no_read_count = 0;

/*
   UAV-side decoding
 */

void decode(core **logUserIn)
{
        /*
           TODO create another dequeue to support sizeof(). e.g. sizeof(body)
         */
        static uint32_t last_received_message = 0;
        uint8_t         read_count = 0;

        // Wait until entire packet has arrived
        while (rx_queue.count >= sizeof(packet))
        {

                last_received_message = get_time_us();
                read_count++;

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
                        uint8_t crc = crc_message((void*) &my_packet_core, sizeof(my_packet_core));

                        if (crc == tmp_crc)
                        {
                                decode_header_pc_uav(&header, &mode, &P, &P1,
                                                     &P2);
                                decode_data_command(data, &roll, &pitch, &yaw,
                                                    &lift);

                                // Log the incoming data
                                memcpy(*logUserIn, &my_packet_core,
                                       sizeof(core));

                                // Reset discard count
                                discard_count = 0;
                        }
                        else
                        {
                                discard_count++;
                                printf("Packet transmission error\n");

                                // Too much packet loss between valid messages
                                // is unsafe
                                if (discard_count >= MAX_LOST_PACKETS)
                                {
                                        mode = PANIC_MODE;
                                        return;
                                }
                        }
                }
        }

        // Checks if cable is disconnected
        uint32_t curr_time = get_time_us();
        int32_t  time_diff = curr_time - last_received_message;
        if (time_diff > 300000)
        {
                if (last_received_message != 0)
                {
                        mode = PANIC_MODE;
                        printf(
                            "Panic mode TIMEOUT_THRESHOLD reached:%ld\t %ld\t %ld\n",
                            time_diff, curr_time, last_received_message);
                }
        }
        else
                no_read_count = 0;

        return;
}

// TODO
// void encode(core)
// {
//
// }
