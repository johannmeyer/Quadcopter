#ifndef SENSORS_H__

#define SENSORS_H__

#include <inttypes.h>
#include <stdbool.h>

#define PHI   1
#define THETA 2
#define PSI   3
#define SP    4
#define SQ    5
#define SR    6
#define SAX   7
#define SAY   8
#define SAZ   9

void calibrate_sensors(void);

bool isCalibrated(void);

int16_t get_sensor(uint8_t sensor);

#endif //SENSORS_H__
