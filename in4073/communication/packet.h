/*
   Johann Meyer
 */

#include <stdint.h>
#ifndef PACKET_H
#define PACKET_H

#define START_BYTE 0xe0
#define END_BYTE 0xf0

/*
   Struct Definitions
 */
struct header {
        uint8_t mode;
};

struct body {
        int8_t roll;
        int8_t pitch;
        int8_t yaw;
        uint8_t lift;
};

// Defined to easily pass data to Cyclic Redundancy Check after modifications to
// the size of body or header.
typedef struct core {
        struct header header;
        struct body body;
} core;


typedef struct packet {
        uint8_t start;
        core packet_core;
        uint8_t crc;
        uint8_t end;
} packet;

/*
   Function Prototypes
 */
void generate_core(core *my_packet_core, uint8_t mode, int8_t roll, int8_t pitch, int8_t yaw, uint8_t lift);

#endif // PACKET_H
