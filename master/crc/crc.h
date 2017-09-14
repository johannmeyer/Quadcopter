#include "stdint.h"
#include "packet.h"

#define CODEWORD    0xA7

void crc_basic_byte(uint8_t *partial, uint8_t byte);

void crc_lut_byte(uint8_t *partial, uint8_t byte);

uint8_t crc_core(core *pCore);

void crc_lut_generate(void);