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
#define PACKET_TYPE_GYRO 3
#define PACKET_TYPE_ACCEL 4
// BODY

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
void encode_data_command(uint8_t *data, int8_t roll, int8_t pitch, int8_t yaw,
                         uint8_t lift);
void decode_data_command(uint8_t *data, int8_t *roll, int8_t *pitch,
                         int8_t *yaw, uint8_t *lift);
void encode_data_gains(uint8_t *data, uint8_t P, uint8_t P1, uint8_t P2);
void decode_data_gains(uint8_t *data, uint8_t *P, uint8_t *P1, uint8_t *P2);

#endif // PACKET_H
