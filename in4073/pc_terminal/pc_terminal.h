#ifndef PC_TERMINAL_H__
#define PC_TERMINAL_H__

#define SAFE_MODE 0
#define PANIC_MODE 1
#define MANUAL_MODE 2
#define CALIBRATION_MODE 3
#define YAW_MODE 4
#define FULL_MODE 5
#define RAW_MODE 6
#define HEIGHT_MODE 7
#define WIRELESS_MODE 8
#define EXIT_MODE 9
#define TRIM_VALUE 200

int16_t lift_key, pitch_key, roll_key, yaw_key;
int16_t lift_js, pitch_js, roll_js, yaw_js;
int32_t pitch, roll, yaw;
int32_t lift;
uint8_t mode;
uint8_t P, P1, P2;

#endif // PC_TERMINAL_H__
