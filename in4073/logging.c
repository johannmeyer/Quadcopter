#include <string.h>
#include "in4073.h"
#include "packet.h"

typedef struct actData actData;

struct actData
{
    int16_t act[4];
};

typedef struct sensData sensData;

struct sensData
{
  int16_t phi;
  int16_t theta;
  int16_t psi;
  int16_t sp;
  int16_t sq;
  int16_t sr;
  int16_t sax;
  int16_t say;
  int16_t saz;
  uint16_t bat;
  int32_t temp;
  int32_t press;
};

typedef struct logEntry logEntry;

struct logEntry
{
    uint32_t time;
    uint8_t mode;
    core lastUserInp;
    actData actuators;
    sensData sensors;
};

static uint32_t nextAddress = 0;
static uint16_t logNum = 0;


void write_log_entry(uint32_t logTime, uint8_t logMode, const core *logUser, const int16_t* logAct,
  int16_t logPhi, int16_t logTheta, int16_t logPsi, int16_t logSp, int16_t logSq, int16_t logSr,
  int16_t logSax, int16_t logSay, int16_t logSaz, uint16_t logBat, int32_t logTemp, int32_t logPress)
{
   /*Creates and writes a log entry to the SPI flash memory if empty slots are found.
   (by Kostas)
   */
    if(nextAddress + sizeof(logEntry) > 0x01FFFF)
    {
        printf("Flash full, logging failed\n");
        return;
    }

    logEntry newEntry;
    //A log entry is filled with the data to be written.
    newEntry.time = logTime;
    newEntry.mode = logMode;
    memcpy(&newEntry.lastUserInp, logUser, sizeof(core));
    memcpy(newEntry.actuators.act, logAct, 4 * sizeof(int16_t));
    newEntry.sensors.phi = logPhi;
    newEntry.sensors.theta = logTheta;
    newEntry.sensors.psi = logPsi;
    newEntry.sensors.sp = logSp;
    newEntry.sensors.sq = logSq;
    newEntry.sensors.sr = logSr;
    newEntry.sensors.sax = logSax;
    newEntry.sensors.say = logSay;
    newEntry.sensors.saz = logSaz;
    newEntry.sensors.bat = logBat;
    newEntry.sensors.temp = logTemp;
    newEntry.sensors.press = logPress;

    //Log is written to the SPI flash memory.
    if(!flash_write_bytes(nextAddress, (uint8_t *) &newEntry, sizeof(logEntry)))
    {
        printf("Log write failed for address %lx\n", nextAddress);
        return;
    }

    //The address of the next entry is calculated and the number of saved log entries is updated.
    nextAddress += sizeof(logEntry);
    logNum++;
}

void print_log_entry(uint16_t entryNo)
{
  /*If the requested log entry exists, is printed in the specified format*/

    if(entryNo + 1 > logNum)
    {
        printf("The requested entry is empty\n");
        return;
    }

    logEntry readEntry;

    if(!flash_read_bytes(entryNo * sizeof(logEntry), (uint8_t *) &readEntry, sizeof(logEntry)))
    {
        printf("Failed to read the requested entry\n");
        return;
    }

    // printf("%10ld | m%d | %x %x %x %x %x | ", readEntry.time, readEntry.mode, readEntry.lastUserInp.header.mode, readEntry.lastUserInp.body.roll, readEntry.lastUserInp.body.pitch, readEntry.lastUserInp.body.yaw, readEntry.lastUserInp.body.lift);
    // printf("%3d %3d %3d %3d | ", readEntry.actuators.act[0], readEntry.actuators.act[1], readEntry.actuators.act[2], readEntry.actuators.act[3]);
    // printf("%6d %6d %6d | ", readEntry.sensors.phi, readEntry.sensors.theta, readEntry.sensors.psi);
    // printf("%6d %6d %6d | ", readEntry.sensors.sp, readEntry.sensors.sq, readEntry.sensors.sr);
    // printf("%6d %6d %6d | ", readEntry.sensors.sax, readEntry.sensors.say, readEntry.sensors.saz);
    // printf("%4d | %4ld | %6ld \n", readEntry.sensors.bat, readEntry.sensors.temp, readEntry.sensors.press);

}

void print_last_log(void)
{
  /*If there is at least one entry logged, the last entry is printed*/
    if(logNum < 1)
    {
        printf("No entry logged\n");
        return;
    }

    print_log_entry(logNum-1);

}
