/*
   Johann Meyer
 */

#include "packet_uav.h"
#include "in4073.h"
#include "crc.h"
#include <assert.h>
#include <string.h>

/*
   UAV-side decoding
 */

int decode(uint8_t *mode, int8_t *roll, int8_t *pitch, int8_t *yaw, uint8_t *lift, core **logUserIn)
{
        /*
           TODO replace asserts with actions like discard data.
           TODO return 1 if packet discarded.
           TODO change dequeue to support sizeof(). e.g. sizeof(body)
         */

        // Read start byte and verify that it is valid.
        uint8_t start = (uint8_t)dequeue(&rx_queue);
        assert(start==START_BYTE);

        // Temporarily store the data in temp variables so that a CRC check can
        // first be performed.
        int8_t tmp_mode = (int8_t)dequeue(&rx_queue);
        int8_t tmp_roll = (int8_t)dequeue(&rx_queue);
        int8_t tmp_pitch = (int8_t)dequeue(&rx_queue);
        int8_t tmp_yaw = (int8_t)dequeue(&rx_queue);
        uint8_t tmp_lift = (uint8_t)dequeue(&rx_queue);

        core my_packet_core;
        generate_core(&my_packet_core, tmp_mode, tmp_roll, tmp_pitch, tmp_yaw, tmp_lift);

        uint8_t tmp_crc = (uint8_t)dequeue(&rx_queue);
        // TODO insert CRC calculation
        uint8_t crc = crc_core(&my_packet_core); // TODO REMOVE
        // uint8_t crc = crc(my_packet_core);

        if (crc == tmp_crc)
        {
                *mode = tmp_mode;
                *roll = tmp_roll;
                *pitch = tmp_pitch;
                *yaw = tmp_yaw;
                *lift = tmp_lift;
                memcpy(*logUserIn, &my_packet_core, sizeof(core));
        }
        else
        {
          printf("Packet transmittion error\n");
        }

        // Read end byte and verify that it is valid.
        uint8_t end = (uint8_t)dequeue(&rx_queue);
        assert(end==END_BYTE);
        return 0;
}
