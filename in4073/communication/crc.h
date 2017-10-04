#ifndef CRC_H__
#define CRC_H__

#include <inttypes.h>

void crc_basic_byte(uint8_t *partial, uint8_t byte);

void crc_lut_byte(uint8_t *partial, uint8_t byte);

uint8_t crc_message(void *message, uint8_t numBytes);

void crc_lut_generate(void);

#endif //CRC_H__
