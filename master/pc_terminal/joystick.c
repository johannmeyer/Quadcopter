/*
   Johann Meyer
 */
#include "joystick.h"
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
/*
   js_event.type
 */
#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

/*
   js_event.number AKA ID of the axis or button
   TODO Update the values
 */
#define JS_EVENT_ROLl 0   // axis 0 left -ve  right +ve
#define JS_EVENT_PITCH 1  //axis 1  pitch up +ve
#define JS_EVENT_YAW 2    //axis 2 clockwise +ve
#define JS_EVENT_LIFT 3    //axis 3 up side negative
#define JS_EVENT_TRIGGER 0  // button 0

/*
   Struct Definitions
 */
struct js_event {
        uint32_t time; /* event timestamp in milliseconds */
        int16_t value; /* value */
        uint8_t type;  /* event type */
        uint8_t number; /* axis/button number */
};

/*
   Variable declarations
 */

int fd;
struct js_event ev;

/*
   Local Function Prototypes
 */
void joystick_axis_handler(struct js_event ev, int16_t *roll, int16_t *pitch, int16_t *yaw, int16_t *lift);
int joystick_button_handler(struct js_event ev);

/*
   Function Declarations
 */

void joystick_open()
{
        fd = open ("/dev/input/js0", O_RDONLY | O_NONBLOCK );
        assert(fd >= 0);
}

void joystick_close()
{
        int failed = close(fd);
        assert (failed==0);
}

int get_joystick_action(int16_t *roll, int16_t *pitch, int16_t *yaw, int16_t *lift)
{
        // Empty the buffer
        while(read (fd, &ev, sizeof(ev)) > 0)
        {
                switch (ev.type & ~JS_EVENT_INIT)
                {
                case JS_EVENT_AXIS:
                        joystick_axis_handler(ev, roll, pitch, yaw, lift);
                        break;
                case JS_EVENT_BUTTON:
                        return joystick_button_handler(ev);
                        break;
                default:
                        fprintf(stderr, "Error: Unmapped Joystick Event Type.\n");
                        break;
                }
        }
        return 0;
}

void joystick_axis_handler(struct js_event ev, int16_t *roll, int16_t *pitch, int16_t *yaw, int16_t *lift)
{
        switch (ev.number)
        {
        case JS_EVENT_ROLl:
                *roll = ev.value;
                break;
        case JS_EVENT_PITCH:
                *pitch = ev.value;
                break;
        case JS_EVENT_YAW:
                *yaw = ev.value;
                break;
        case JS_EVENT_LIFT:
                *lift = ev.value;
                break;
        default:
                fprintf(stderr, "Error: Unmapped Joystick Axis Event Number.\n");
                break;
        }
}

int joystick_button_handler(struct js_event ev)
{
        switch (ev.number)
        {
        case JS_EVENT_TRIGGER:
                if (ev.value)
                {
                        printf("Panic Mode manually activated\n");
                        return 1;
                }
                break;
        default:
                fprintf(stderr, "Error: Unmapped Joystick Button Event Number.\n");
                break;
        }
        return 0;
}
