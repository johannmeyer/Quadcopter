#include "crc.h"
#include "lut.h"


uint8_t crcLutCode(uint8_t init, uint8_t byte)
{
    return crclut[init ^ byte];
}

uint8_t crcBasicCode(uint8_t init, uint8_t byte)
{
    register uint8_t gen = CODEWORD;
    register uint8_t crc = init ^ byte;
    
    int i=0;
    
    for(i=0; i<8; i++)
    {
    
        if(crc & 0x80)
        {
            crc = (crc<<1) ^ gen;
        }
        else
        {
            crc <<=1;
        }
    }
    
    return crc;
}