#include <stdlib.h>
#include <stdio.h>
#include "crc.h"
#include "time.h"

void crcLutGenerate(void);

core pack;
char word[5] = {'t', 'e', 's', 't', '1'};

int main(void)
{
    //crc_lut_generate();
    
    pack.header.mode = 't';
    pack.body.roll = 'e';
    pack.body.pitch = 's';
    pack.body.yaw = 't';
    pack.body.lift = '1';
    
    
    int i = 0, j = 0;
    uint8_t ctest = 0;
    
    uint8_t lcrc, bcrc;
    
    clock_t sumBas = 0, sumLut = 0;
    
    clock_t start;
    
    for(j=0; j<1000; j++)
    {
        bcrc = 0;
        start = clock();
        
        for(i=0; i<5; i++)
        {
            crc_basic_byte(&bcrc, word[i]);
        }
        sumBas += clock() - start;
        
        lcrc = 0;
        start = clock();

        lcrc = crc_core(&pack);

        sumLut += clock() - start;
    }
    
    printf("\nCycles using Basic: %d\n", ((int) sumBas)/j);
    printf("CRC8 using Basic: %d\n",bcrc);
    
    printf("\nCycles using LUT: %d\n", ((int) sumLut)/j);
    printf("CRC8 using LUT: %d\n", lcrc);
    
    return 0;
}