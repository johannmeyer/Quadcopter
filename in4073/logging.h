#ifndef LOGGING_H__
#define LOGGING_H__

#define SHORT_LOGGING 0
#define FULL_LOGGING  1

void init_logging(uint8_t logMode);

void select_log_mode(uint8_t logMode);

void write_short_log(uint32_t logTime, uint8_t logMode, const int16_t* logAct, int16_t logPhi, int16_t logTheta, int16_t logPsi);

void write_log_entry(uint32_t logTime, uint8_t logMode, const core *logUser, const int16_t* logAct,
  int16_t logPhi, int16_t logTheta, int16_t logPsi, int16_t logSp, int16_t logSq, int16_t logSr,
  int16_t logSax, int16_t logSay, int16_t logSaz, uint16_t logBat, int32_t logTemp, int32_t logPress);

void print_log_entry(uint16_t entryNo);

void print_last_log(void);

#endif // LOGGING_H__
