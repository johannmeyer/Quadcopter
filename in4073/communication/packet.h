/*
   Johann Meyer
 */

#include <stdint.h>
#ifndef PACKET_H
#define PACKET_H

#define START_BYTE 0xff
#define BODY_LENGTH 4

/*
 Packet Definition PC -> UAV
 */
// HEADER
#define MODE_OFFSET 0             // bit offset
#define MODE_LENGTH 0b1111;       // used for anding
#define PACKET_TYPE_OFFSET 4      // bit offset
#define PACKET_TYPE_LENGTH 0b111; // used for anding

// Packet types
#define PACKET_TYPE_COMMAND 0
#define PACKET_TYPE_GAINS 1

// BODY
// PACKET_TYPE_COMMAND
#define ROLL_OFFSET 0  // byte offset
#define PITCH_OFFSET 1 // byte offset
#define YAW_OFFSET 2   // byte offset
#define LIFT_OFFSET 3  // byte offset

// PACKET_TYPE_GAINS
#define P_GAIN_OFFSET 0  // byte offset // Yaw controller
#define P1_GAIN_OFFSET 1 // byte offset
#define P2_GAIN_OFFSET 2 // byte offset

/*
 Packet Definition UAV -> PC
 */
// HEADER = HEADER OF PC -> UAV
#define PACKET_TYPE_ACTUATOR 0
#define PACKET_TYPE_ANGLES_BAT 1
#define PACKET_TYPE_GYRO 2
#define PACKET_TYPE_ACCEL 3
// BODY
// PACKET_TYPE_ACTUATOR
// offset = actuator number

// PACKET_TYPE_ANGLES_BAT
#define PHI_OFFSET 0
#define THETA_OFFSET 1
#define PSI_OFFSET 2
#define BATTERY_OFFSET 3

// PACKET_TYPE_GYRO
#define PHI_RATE_OFFSET 0
#define THETA_RATE_OFFSET 1
#define PSI_RATE_OFFSET 2

// PACKET_TYPE_ACCEL
#define SX_OFFSET 0
#define SY_OFFSET 1
#define SZ_OFFSET 2

/*
   Struct Definitions
 */

// Defined to easily pass data to Cyclic Redundancy Check after modifications to
// the size of body or header.
typedef struct core
{
        uint8_t header;
        uint8_t body[BODY_LENGTH];
} core;

typedef struct packet
{
        uint8_t start;
        core    packet_core;
        uint8_t crc;
} packet;

/*
   Function Prototypes
 */
void encode_header(uint8_t *header, uint8_t mode, uint8_t packet_type);
void decode_header(uint8_t *header, uint8_t *mode, uint8_t *packet_type);
// PC -> UAV
void encode_data_command(uint8_t *data, int8_t roll, int8_t pitch, int8_t yaw,
                         uint8_t lift);
void decode_data_command(uint8_t *data, int8_t *roll, int8_t *pitch,
                         int8_t *yaw, uint8_t *lift);
void encode_data_gains(uint8_t *data, uint8_t P, uint8_t P1, uint8_t P2);
void decode_data_gains(uint8_t *data, uint8_t *P, uint8_t *P1, uint8_t *P2);

// UAV -> PC
void encode_data_motor(uint8_t *data, int8_t *ae);
void decode_data_motor(uint8_t *data, int8_t *ae);
void encode_data_angles_bat(uint8_t *data, int8_t phi, int8_t theta, int8_t psi,
                            uint8_t battery);
void decode_data_angles_bat(uint8_t *data, int8_t *phi, int8_t *theta,
                            int8_t *psi, uint8_t *battery);
void encode_data_gyro(uint8_t *data, int8_t phi_rate, int8_t theta_rate,
                      int8_t psi_rate);
void decode_data_gyro(uint8_t *data, int8_t *phi_rate, int8_t *theta_rate,
                      int8_t *psi_rate);
void encode_data_accel(uint8_t *data, int8_t sx, int8_t sy, int8_t sz);
void decode_data_accel(uint8_t *data, int8_t *sx, int8_t *sy, int8_t *sz);
#endif // PACKET_H
