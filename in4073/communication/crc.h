#ifndef CRC_H__
#define CRC_H__

#include <inttypes.h>
#include "packet.h"

void crc_basic_byte(uint8_t *partial, uint8_t byte);

void crc_lut_byte(uint8_t *partial, uint8_t byte);

uint8_t crc_core(core *pCore);

void crc_lut_generate(void);

#endif //CRC_H__
