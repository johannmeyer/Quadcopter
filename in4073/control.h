#include <stdint.h>

#define MIN_VALUE 250
#define MAX_VALUE 750
#define MAX_MOTOR 800
#define MAX_LIFT 600
#define OUTER_LOOP_FREQ 3

void update_motors();

void fp_yaw_control(int16_t roll, int16_t pitch, int16_t yaw, uint16_t lift, uint16_t yawPpar, int16_t senPsi);

void int_yaw_control(int16_t proll, int16_t ppitch, int16_t pyaw, uint16_t plift, uint16_t yawPpar, int16_t psi_sen);

void run_filters_and_control(uint8_t mode);
void yaw_mode();
void full_mode();
void height_mode();
