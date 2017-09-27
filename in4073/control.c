/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "control.h"
#include "sensors.h"
#include "in4073.h"

/*
Local Variables
 */
int16_t pitch_rate_d, roll_rate_d;
int16_t pitch_act, roll_act, yaw_act;
/*
Local Function Prototypes
 */
void rate_controller();
void angle_controller();

void update_motors(void)
{
        for (int i = 0; i < 4; i++)
        {
                if (ae[i] > 600)
                        ae[i] = 600;
                else if (ae[i] < 0)
                        ae[i] = 0;
        }

        motor[0] = ae[0];
        motor[1] = ae[1];
        motor[2] = ae[2];
        motor[3] = ae[3];
}

void run_filters_and_control()
{
        // fancy stuff here
        // control loops and/or filters

        // Apply actuation
        // TODO is be still necessary?
        ae[0] = (new_lift + pitch_act) / b - yaw_act;
        ae[1] = (new_lift - roll_act) / b + yaw_act;
        ae[2] = (new_lift - pitch_act) / b - yaw_act;
        ae[3] = (new_lift + roll_act) / b + yaw_act;
        update_motors();
}

void rate_controller()
{
        /*
        Johann Meyer
         */
        // Scale sensor values
        // TODO check this works
        int8_t pitch_rate_s = (int32_t)get_sensor(SP) * 127 / 32768;
        int8_t roll_rate_s = (int32_t)get_sensor(SQ) * 127 / 32768;

        // Set actuation inputs
        pitch_act = P2 * (pitch_rate_d - pitch_rate_s);
        roll_act = P2 * (roll_rate_d - roll_rate_s);
}

void angle_controller()
{
        /*
        Johann Meyer
         */
        // Scale sensor values
        // TODO check this works
        int8_t roll_s = (int32_t)get_sensor(PHI) * 127 / 32768;
        int8_t pitch_s = (int32_t)get_sensor(THETA) * 127 / 32768;

        int16_t roll_err = roll - roll_s;
        int16_t pitch_err = pitch - pitch_s;

        // Set new desired values for the rate_controller()
        pitch_rate_d = P1 * pitch_err;
        roll_rate_d = P1 * roll_err;
}
