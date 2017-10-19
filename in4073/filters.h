#ifndef FILTERS_H__

#define FILTERS_H__

#include "fixedp.h"

fp butter(int16_t newSample, uint8_t sensorId);
fp butterAfterKalman(int16_t newSample, uint8_t sensorId);
fp Kalman(fp sOrient, int16_t sRate, uint8_t sensorId);

#endif //FILTERS_H__
