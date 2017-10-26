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
   UAV-side decoding of PC messages (Commands)
 */

void decode(core **logUserIn)
{

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
                        uint8_t crc =
                            crc_message((void *)&my_packet_core, sizeof(core));

                        if (crc == tmp_crc)
                        {
                                decode_header_pc_uav(&header, &mode, &P, &P1,
                                                     &P2, &P3, &P4);

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
                if (last_received_message != 0) // Checks if not very first read
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

/*
Encodes messages on the UAV to be sent to the PC
 */
void encode(packet *my_packet, uint8_t packet_type)
{
        core *my_packet_core = &my_packet->packet_core;

        /*
          Construct Body
         */
        switch (packet_type)
        {
        case PACKET_TYPE_GAINS1:;
                // Cannot be equal to START_BYTE except bat_volt
                uint8_t _bat_volt = bat_volt >> 8;
                if ((uint8_t)_bat_volt == START_BYTE)
                        _bat_volt--;
                encode_data_gains1(my_packet_core->body, P, P1, P2, _bat_volt);
                break;
        case PACKET_TYPE_GAINS2:
                // Cannot be equal to START_BYTE
                encode_data_gains2(my_packet_core->body, P3, P4);
                break;
        case PACKET_TYPE_ANGLES:;
                int8_t _phi = phi >> 8; // Reduce precision to fit in Byte
                int8_t _theta = theta >> 8;
                int8_t _psi = psi >> 8;
                // Check that data is not equal to START_BYTE
                if ((uint8_t)_phi == START_BYTE)
                        _phi = (int8_t)((uint8_t)_phi-1);
                if ((uint8_t)_theta == START_BYTE)
                        _theta = (int8_t)((uint8_t)_theta-1);
                if ((uint8_t)_psi == START_BYTE)
                        _psi = (int8_t)((uint8_t)_psi-1);

                encode_data_angles(my_packet_core->body, _phi, _theta, _psi);
                break;
        case PACKET_TYPE_GYRO:;
                int8_t _sp = sp >> 8; // Reduce precision to fit in Byte
                int8_t _sq = sq >> 8;
                int8_t _sr = sr >> 8;
                // Check that data is not equal to START_BYTE
                if ((uint8_t)_sp == START_BYTE)
                        _sp = (int8_t)((uint8_t)_sp-1);
                if ((uint8_t)_sq == START_BYTE)
                        _sq = (int8_t)((uint8_t)_sq-1);
                if ((uint8_t)_sr == START_BYTE)
                        _sr = (int8_t)((uint8_t)_sr-1);

                encode_data_gyro(my_packet_core->body, _sp, _sq, _sr);
                break;
        case PACKET_TYPE_ACCEL:;
                int8_t _sax = sax >> 8; // Reduce precision to fit in Byte
                int8_t _say = say >> 8;
                int8_t _saz = saz >> 8;
                // Check that data is not equal to START_BYTE
                if ((uint8_t)_sax == START_BYTE)
                        _sax = (int8_t)((uint8_t)_sax-1);
                if ((uint8_t)_say == START_BYTE)
                        _say = (int8_t)((uint8_t)_say-1);
                if ((uint8_t)_saz == START_BYTE)
                        _saz = (int8_t)((uint8_t)_saz-1);
                encode_data_accel(my_packet_core->body, _sax, _say, _saz);
                break;
        case PACKET_TYPE_ACTUATOR:;
                int8_t motor_vals[4];
                for (int i = 0; i < 4; i++)
                {
                        motor_vals[i] = ae[i] >> 8; // Reduce precision
                        // Check that data is not equal to START_BYTE
                        if ((uint8_t)motor_vals[i] == START_BYTE)
                                motor_vals[i] = (int8_t)((uint8_t)motor_vals[i]-1);
                }

                encode_data_motor(my_packet_core->body, motor_vals);
                break;
        case PACKET_TYPE_PRINTF:
                memcpy(my_packet_core->body, printf_data, BODY_LENGTH);
                break;
        default:
                printf("Packet type not mapped on UAV\n");
                break;
        }

        /*
           Construct the Packet
         */

        encode_header_uav_pc(&my_packet_core->header, prev_mode, packet_type);
        // Calculate CRC
        uint8_t crc = crc_message((void *)my_packet_core, sizeof(core));

        my_packet->start = START_BYTE;
        my_packet->crc = crc;
}
