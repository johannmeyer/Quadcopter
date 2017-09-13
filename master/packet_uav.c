/*
   Johann Meyer
 */

#include "packet_uav.h"
#include "in4073.h"
#include <assert.h>

/*
   UAV-side decoding
 */

int decode(uint8_t *mode, uint8_t *roll, uint8_t *pitch, uint8_t *yaw, uint8_t *lift)
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
        uint8_t tmp_mode = (uint8_t)dequeue(&rx_queue);
        uint8_t tmp_roll = (uint8_t)dequeue(&rx_queue);
        uint8_t tmp_pitch = (uint8_t)dequeue(&rx_queue);
        uint8_t tmp_yaw = (uint8_t)dequeue(&rx_queue);
        uint8_t tmp_lift = (uint8_t)dequeue(&rx_queue);

        core my_packet_core;
        generate_core(&my_packet_core, tmp_mode, tmp_roll, tmp_pitch, tmp_yaw, tmp_lift);

        uint8_t tmp_crc = (uint8_t)dequeue(&rx_queue);
        // TODO insert CRC calculation
        uint8_t crc = 0xff; // TODO REMOVE
        // uint8_t crc = crc(my_packet_core);

        if (crc == tmp_crc)
        {
                *mode = tmp_mode;
                *roll = tmp_roll;
                *pitch = tmp_pitch;
                *yaw = tmp_yaw;
                *lift = tmp_lift;
        }

        // Read end byte and verify that it is valid.
        uint8_t end = (uint8_t)dequeue(&rx_queue);
        assert(end==END_BYTE);
        return 0;
}
