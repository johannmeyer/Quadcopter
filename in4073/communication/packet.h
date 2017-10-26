/*
   Johann Meyer
 */

#include <stdint.h>
#ifndef PACKET_H
#define PACKET_H

#define START_BYTE 0xff
#define BODY_LENGTH 4

#define GAIN_NO_INCREMENT 0
#define GAIN_P_INCREMENT 1
#define GAIN_P_DECREMENT 2
#define GAIN_P1_INCREMENT 3
#define GAIN_P1_DECREMENT 4
#define GAIN_P2_INCREMENT 5
#define GAIN_P2_DECREMENT 6
#define GAIN_P3_INCREMENT 7
#define GAIN_P3_DECREMENT 8
#define GAIN_P4_INCREMENT 9
#define GAIN_P4_DECREMENT 10

/*
 Packet Definition PC -> UAV
 */
// HEADER
#define MODE_OFFSET 0                  // bit offset
#define MODE_LENGTH 0b1111             // used for anding
#define GAIN_INCREMENTER_OFFSET 4      // bit offset
#define GAIN_INCREMENTER_LENGTH 0b1111 // used for anding

/* Only one Packet Type now as Gains are now implemented as incrementers */
// Packet types
// #define PACKET_TYPE_COMMAND 0
// #define PACKET_TYPE_GAINS 1

// BODY
// PACKET_TYPE_COMMAND
#define ROLL_OFFSET 0  // byte offset
#define PITCH_OFFSET 1 // byte offset
#define YAW_OFFSET 2   // byte offset
#define LIFT_OFFSET 3  // byte offset

// // PACKET_TYPE_GAINS
// #define P_GAIN_OFFSET 0  // byte offset // Yaw controller
// #define P1_GAIN_OFFSET 1 // byte offset
// #define P2_GAIN_OFFSET 2 // byte offset

/*
 Packet Definition UAV -> PC
 */
// HEADER
// HEADER
#define MODE_OFFSET 0            // bit offset
#define MODE_LENGTH 0b1111       // used for anding
#define PACKET_TYPE_OFFSET 4     // bit offset
#define PACKET_TYPE_LENGTH 0b111 // used for anding

// Packet types
#define PACKET_TYPE_ACTUATOR 0
#define PACKET_TYPE_ANGLES 1
#define PACKET_TYPE_GYRO 2
#define PACKET_TYPE_ACCEL 3
#define PACKET_TYPE_GAINS1 4 // INCLUDES BATTERY
#define PACKET_TYPE_GAINS2 5
#define PACKET_TYPE_PRINTF 6
// BODY
// PACKET_TYPE_ACTUATOR
// offset = actuator number

// PACKET_TYPE_ANGLES
#define PHI_OFFSET 0
#define THETA_OFFSET 1
#define PSI_OFFSET 2

// PACKET_TYPE_GYRO
#define PHI_RATE_OFFSET 0
#define THETA_RATE_OFFSET 1
#define PSI_RATE_OFFSET 2

// PACKET_TYPE_ACCEL
#define SX_OFFSET 0
#define SY_OFFSET 1
#define SZ_OFFSET 2

// PACKET_TYPE_GAINS1
#define P_OFFSET 0
#define P1_OFFSET 1
#define P2_OFFSET 2
#define BATTERY_OFFSET 3

// PACKET_TYPE_GAINS2
#define P3_OFFSET 0
#define P4_OFFSET 1

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

// PC -> UAV
void encode_header_pc_uav(uint8_t *header, uint8_t mode, uint8_t p_incrementer);
void decode_header_pc_uav(uint8_t *header, uint8_t *mode, uint8_t *P,
                          uint8_t *P1, uint8_t *P2, uint8_t *P3, uint8_t *P4);

void encode_data_command(uint8_t *data, int8_t roll, int8_t pitch, int8_t yaw,
                         uint8_t lift);
void decode_data_command(uint8_t *data, int8_t *roll, int8_t *pitch,
                         int8_t *yaw, uint8_t *lift);
// void encode_data_gains(uint8_t *data, uint8_t P, uint8_t P1, uint8_t P2);
// void decode_data_gains(uint8_t *data, uint8_t *P, uint8_t *P1, uint8_t *P2);

// UAV -> PC
void encode_header_uav_pc(uint8_t *header, uint8_t mode, uint8_t packet_type);
void decode_header_uav_pc(uint8_t *header, uint8_t *mode, uint8_t *packet_type);

void encode_data_motor(uint8_t *data, int8_t *ae);
void decode_data_motor(uint8_t *data, int8_t *ae);
void encode_data_angles(uint8_t *data, int8_t phi, int8_t theta, int8_t psi);
void decode_data_angles(uint8_t *data, int8_t *phi, int8_t *theta, int8_t *psi);
void encode_data_gyro(uint8_t *data, int8_t phi_rate, int8_t theta_rate,
                      int8_t psi_rate);
void decode_data_gyro(uint8_t *data, int8_t *phi_rate, int8_t *theta_rate,
                      int8_t *psi_rate);
void encode_data_accel(uint8_t *data, int8_t sx, int8_t sy, int8_t sz);
void decode_data_accel(uint8_t *data, int8_t *sx, int8_t *sy, int8_t *sz);
void encode_data_gains1(uint8_t *data, uint8_t P, uint8_t P1, uint8_t P2,
                        uint8_t battery);
void decode_data_gains1(uint8_t *data, uint8_t *P, uint8_t *P1, uint8_t *P2,
                        uint8_t *battery);
void encode_data_gains2(uint8_t *data, uint8_t P3, uint8_t P4);
void decode_data_gains2(uint8_t *data, uint8_t *P3, uint8_t *P4);
void encode_printf(uint8_t *data, char *text);
void decode_printf(uint8_t *data, char *text);
#endif // PACKET_H
