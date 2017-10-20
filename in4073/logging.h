#ifndef LOGGING_H__
#define LOGGING_H__

#define SHORT_LOGGING 0
#define FULL_LOGGING  1
#define SENSOR_LOGGING 2
#include "packet.h"

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
    int16_t actuators[4];
    sensData sensors;
};

typedef struct shortEntry shortEntry;

struct shortEntry
{
  uint32_t time;
  uint8_t mode;
  int16_t actuators[4];
  int16_t phi;
  int16_t theta;
  int16_t psi;
};

typedef struct sensorEntry sensorEntry;

struct sensorEntry
{
  uint32_t time;
  uint8_t mode;
  int16_t sax;
  int16_t say;
  int16_t saz;
  int16_t sp;
  int16_t sq;
  int16_t sr;
};

void init_logging(uint8_t logMode);

void select_log_mode(uint8_t logMode);

void write_short_log(uint32_t logTime, uint8_t logMode, const int16_t* logAct, int16_t logPhi, int16_t logTheta, int16_t logPsi);

void write_log_entry(uint32_t logTime, uint8_t logMode, const core *logUser, const int16_t* logAct,
  int16_t logPhi, int16_t logTheta, int16_t logPsi, int16_t logSp, int16_t logSq, int16_t logSr,
  int16_t logSax, int16_t logSay, int16_t logSaz, uint16_t logBat, int32_t logTemp, int32_t logPress);

void write_sensor_log(uint32_t logTime, uint8_t logMode, int16_t logSp, int16_t logSq, int16_t logSr,
  int16_t logSax, int16_t logSay, int16_t logSaz);

void print_log_entry(uint16_t entryNo);

void print_last_log(void);

#endif // LOGGING_H__
