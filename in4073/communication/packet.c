/*
   Johann Meyer
 */
#include "packet.h"
#include <string.h>

// HEADER
void encode_header(uint8_t *header, uint8_t mode, uint8_t packet_type)
{
        // Construct Packet Header
        *header = 0;
        *header |= mode << MODE_OFFSET;
        *header |= packet_type << PACKET_TYPE_OFFSET;
}

void decode_header(uint8_t *header, uint8_t *mode, uint8_t *packet_type)
{
      uint8_t data_pointer = *header << MODE_OFFSET;
      *mode = data_pointer & MODE_LENGTH;
      data_pointer = data_pointer >> PACKET_TYPE_OFFSET;
      *packet_type = data_pointer & PACKET_TYPE_LENGTH;
}

/*
PC -> UAV
 */

// PACKET_TYPE_COMMAND
void encode_data_command(uint8_t *data, int8_t roll, int8_t pitch, int8_t yaw,
                         uint8_t lift)
{
        data[ROLL_OFFSET] = (uint8_t)roll;
        data[PITCH_OFFSET] = (uint8_t)pitch;
        data[YAW_OFFSET] = (uint8_t)yaw;
        data[LIFT_OFFSET] = lift;
}

void decode_data_command(uint8_t *data, int8_t *roll, int8_t *pitch,
                         int8_t *yaw, uint8_t *lift)
{
        *roll = (int8_t)data[ROLL_OFFSET];
        *pitch = (int8_t)data[PITCH_OFFSET];
        *yaw = (int8_t)data[YAW_OFFSET];
        *lift = data[LIFT_OFFSET];
}

// PACKET_TYPE_GAINS
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

/*
UAV -> PC
 */

// PACKET_TYPE_ACTUATOR
void encode_data_motor(uint8_t *data, int8_t *ae) { memcpy(data, ae, 4); }

void decode_data_motor(uint8_t *data, int8_t *ae) { memcpy(ae, data, 4); }

// PACKET_TYPE_ANGLES_BAT
void encode_data_angles_bat(uint8_t *data, int8_t phi, int8_t theta, int8_t psi,
                            uint8_t battery)
{
        data[PHI_OFFSET] = (uint8_t)phi;
        data[THETA_OFFSET] = (uint8_t)theta;
        data[PSI_OFFSET] = (uint8_t)psi;
        data[BATTERY_OFFSET] = battery;
}

void decode_data_angles_bat(uint8_t *data, int8_t *phi, int8_t *theta,
                            int8_t *psi, uint8_t *battery)
{
        *phi = data[PHI_OFFSET];
        *theta = data[THETA_OFFSET];
        *psi = data[PSI_OFFSET];
        *battery = data[BATTERY_OFFSET];
}

// PACKET_TYPE_GYRO
void encode_data_gyro(uint8_t *data, int8_t phi_rate, int8_t theta_rate,
                      int8_t psi_rate)
{
        data[PHI_RATE_OFFSET] = phi_rate;
        data[THETA_RATE_OFFSET] = theta_rate;
        data[PSI_RATE_OFFSET] = psi_rate;
}

void decode_data_gyro(uint8_t *data, int8_t *phi_rate, int8_t *theta_rate,
                      int8_t *psi_rate)
{
        *phi_rate = data[PHI_RATE_OFFSET];
        *theta_rate = data[THETA_RATE_OFFSET];
        *psi_rate = data[PSI_RATE_OFFSET];
}

// PACKET_TYPE_ACCEL
void encode_data_accel(uint8_t *data, int8_t sx, int8_t sy, int8_t sz)
{
        data[SX_OFFSET] = sx;
        data[SY_OFFSET] = sy;
        data[SZ_OFFSET] = sz;
}

void decode_data_accel(uint8_t *data, int8_t *sx, int8_t *sy, int8_t *sz)
{
        *sx = data[SX_OFFSET];
        *sy = data[SY_OFFSET];
        *sz = data[SZ_OFFSET];
}
