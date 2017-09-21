/*
   Johann Meyer
 */
#include "packet.h"

void encode_header(uint8_t *header, uint8_t mode, uint8_t packet_type)
{
        // Construct Packet Header
        *header = 0;
        *header |= mode << MODE_OFFSET;
        *header |= packet_type << PACKET_TYPE_OFFSET;
}

void decode_header(uint8_t *header, uint8_t *mode, uint8_t *packet_type)
{
        *mode = (*header << MODE_OFFSET) & MODE_LENGTH;
        *packet_type = (*header << PACKET_TYPE_OFFSET) & PACKET_TYPE_LENGTH;
}

void encode_data_command(uint8_t *data, int8_t roll, int8_t pitch, int8_t yaw,
                         uint8_t lift)
{
        data[ROLL_OFFSET] = (uint8_t)roll;
        data[PITCH_OFFSET] = (uint8_t)pitch;
        data[YAW_OFFSET] = (uint8_t)yaw;
        data[LIFT_OFFSET] = lift;
}

void decode_data_command(uint8_t *data, int8_t *roll, int8_t *pitch, int8_t *yaw,
                         uint8_t *lift)
{
        *roll = (int8_t)data[ROLL_OFFSET];
        *pitch = (int8_t)data[PITCH_OFFSET];
        *yaw = (int8_t)data[YAW_OFFSET];
        *lift = data[LIFT_OFFSET];
}

void encode_data_gains(uint8_t *data, uint8_t P, uint8_t P1, uint8_t P2)
{
        data[P_GAIN_OFFSET] = P;
        data[P1_GAIN_OFFSET] = P1;
        data[P2_GAIN_OFFSET] = P2;
}

void decode_data_gains(uint8_t *data, uint8_t *P, uint8_t *P1, uint8_t *P2)
{
        *P = data[P_GAIN_OFFSET];
        *P1 = data[P1_GAIN_OFFSET];
        *P2 = data[P2_GAIN_OFFSET];
}
