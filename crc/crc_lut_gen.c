#include <stdio.h>
#include "crc.h"

void crc_lut_generate(void)
{
    /*
    Produces the header file, which contains the look-up table
	for CRC8 coded byte from 0x00 to 0xFF using the codeword
    defined in the "crc.h".
    (by Kostas)
    */ 
    
    int i=0;
    uint8_t coded;
    
    FILE *hfil = fopen("lut.h","w");
    
    fprintf(hfil,"const uint8_t crclut[256]={");
    
    for(i=0; i<0xFF; i++)
    {
        coded = 0;
        crc_basic_byte(&coded, i);
        fprintf(hfil,"%d, ", coded);
    }
    coded = 0;
    crc_basic_byte(&coded, i);
    fprintf(hfil,"%d};\n", coded);
    
    fclose(hfil);
}