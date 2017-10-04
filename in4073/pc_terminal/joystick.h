/*
   Johann Meyer
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>
/*
   Function prototypes
 */

// Joystick file Management
void joystick_open();
void joystick_close();

// Read & Update joystick values
void get_joystick_action(uint8_t *mode, int16_t *roll, int16_t *pitch,
                        int16_t *yaw, int16_t *lift);
#endif // JOYSTICK_H
