/*Written by Konstantinos-P. Metaxas*/

#include "crc.h"
#include "lut.h"

#define CODEWORD  0xA7

void crc_basic_byte(uint8_t *partial, uint8_t byte)
{
    /*
    Codes a byte adding any coded more significant bytes using the 'init' argument.
	  For the most significant byte the initial value(crc) should be 0,
	  while for the following bytes, it should be the result of the previous byte.
	  The CRC8 coding algorithm is used to produce the look-up table before compilation.
    */

    register uint8_t gen = CODEWORD;
    register uint8_t crc = *partial ^ byte;

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

    *partial = crc;
}

void crc_lut_byte(uint8_t *partial, uint8_t byte)
{
    /*
    Codes a byte adding any coded more significant bytes using the 'crc' argument.
	  For the most significant byte the initial value(crc) should be 0,
	  while for the following bytes, it should be the result of the previous byte.
	  A pre-compiled look-up table is used for speed since there are only 256 possible values.
    */

    *partial = crclut[*partial ^ byte];
}

uint8_t crc_message(void *message, uint8_t numBytes)
{
    /*
    The data(core) of the packet are coded per byte using the CRC8 LUT
    for error detection purposes.
    */

    int i;
    uint8_t crc = 0;

    uint8_t *nextByte = (uint8_t*) message;

    for(i=0; i<numBytes; i++)
    {
        crc_lut_byte(&crc, *(nextByte++));
    }

    // To avoid START_BYTE aliasing and has the max number of bit flips. It
    // does not require a flag in header.
    if (crc == START_BYTE)
            crc = 0;

    return crc;
}
