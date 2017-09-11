#include "stdint.h"

#define CODEWORD    0xA7

uint8_t crcBasicCode(uint8_t init, uint8_t byte);

uint8_t crcLutCode(uint8_t init, uint8_t byte);

void crcLutGenerate(void);