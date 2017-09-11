#include <stdio.h>
#include "crc.h"

void crcLutGenerate(void)
{
    int i=0;
    
    FILE *hfil = fopen("lut.h","w");
    
    fprintf(hfil,"const uint8_t crclut[256]={");
    
    for(i=0; i<0xFF; i++)
    {
        fprintf(hfil,"%d, ",crcBasicCode(0, i));
    }
    fprintf(hfil,"%d};\n",crcBasicCode(0, ++i));
    
    fclose(hfil);
}