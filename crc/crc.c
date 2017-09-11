#include "crc.h"
#include "lut.h"


uint8_t crcLutCode(uint8_t init, uint8_t byte)
{
	/* Codes a word per byte irrespective of the number of bytes it contains.
	   For the first byte the initial value(init) should be 0,
	   while for the following bytes, it should be the result of the previous byte.
	   A pre-compiled look-up table is used for speed since there are only 256 possible values.
	   (by Kostas)*/

    return crclut[init ^ byte];
}

uint8_t crcBasicCode(uint8_t init, uint8_t byte)
{
	/* Codes a word per byte irrespective of the number of bytes it contains.
	   For the first byte the initial value(init) should be 0,
	   while for the following bytes, it should be the result of the previous byte.
	   The algorithm is followed, thus the result for is byte is computed during execution.
	   (by Kostas)*/

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
