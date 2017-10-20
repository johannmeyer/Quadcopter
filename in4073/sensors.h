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
#define BARO  10

void calibrate_sensors(void);

void calibrate_raw_sensors(void);

bool isCalibrated(void);

int32_t get_sensor(uint8_t sensor);

void get_filtered_data(void);

#endif //SENSORS_H__
