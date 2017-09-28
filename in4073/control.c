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
#include "fixmath.h"
#include "in4073.h"
#include "sensors.h"
/*
Local Variables
 */
uint8_t outer_counter = 0;
int16_t pitch_rate_d, roll_rate_d;
int16_t pitch_act, roll_act, yaw_act;
/*
Local Function Prototypes
 */
void update_actuator();
void rate_controller();
void angle_controller();
void yaw_controller();

void update_actuator()
{
      // Apply actuation
      // TODO is b still necessary for full control?
      int8_t b = 1;
      ae[0] = (new_lift + pitch_act) / b - yaw_act;
      ae[1] = (new_lift - roll_act) / b + yaw_act;
      ae[2] = (new_lift - pitch_act) / b - yaw_act;
      ae[3] = (new_lift + roll_act) / b + yaw_act;
      for (int i = 0; i < 4; i++)
      {
              if (ae[i] < 180 && new_lift > 180)
                      ae[i] = 180;
      }
      update_motors();
}

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

void fp_yaw_control(int16_t proll, int16_t ppitch, int16_t pyaw, uint16_t plift,
                    uint16_t yawPpar, int16_t senPsi)
{
        // const fix16_t convIndex = F16(127/32768);
        const fix16_t convIndex = F16(0.00387573242188);
        fix16_t       froll = fix16_from_int(proll);
        fix16_t       fpitch = fix16_from_int(ppitch);
        fix16_t       fyaw = fix16_from_int(pyaw >> 2);
        fix16_t       flift = fix16_from_int(plift);
        fix16_t       fyawPpar = fix16_from_int(yawPpar);
        fix16_t       fsenPsi = fix16_from_int(senPsi);

        fix16_t convPsi = fix16_mul(fsenPsi, convIndex);
        fix16_t fyaw_error = fix16_sub(fyaw, convPsi);

        printf("yaw_error: %d,yaw: %d, converted Psi: %d, sensed Psi: %d \n",
               fix16_to_int(fyaw_error), fix16_to_int(fyaw),
               fix16_to_int(convPsi), senPsi);
        ae[0] = fix16_to_int(fix16_sub(fix16_add(flift, fpitch),
                                       fix16_mul(fyawPpar, fyaw_error)));
        ae[1] = fix16_to_int(fix16_add(fix16_sub(flift, froll),
                                       fix16_mul(fyawPpar, fyaw_error)));
        ae[2] = fix16_to_int(fix16_sub(fix16_sub(flift, fpitch),
                                       fix16_mul(fyawPpar, fyaw_error)));
        ae[3] = fix16_to_int(fix16_add(fix16_add(flift, froll),
                                       fix16_mul(fyawPpar, fyaw_error)));
}
/*
TODO Deprecated
 */
 /*
void int_yaw_control(int16_t proll, int16_t ppitch, int16_t pyaw,
                     uint16_t plift, uint16_t yawPpar, int16_t psi_sen)
{
        int8_t b = 1;
        int8_t psi_conv = (int8_t)((psi_sen * 127) / 32768);
        // dcpsi_s = (int8_t)(((float)dcpsi/32768)*127);
        // psi_s = psi_s - dcpsi_s;  // value of yaw from calibrated point
        int8_t yaw_error = (pyaw / 4) - psi_conv;
        printf("yaw_error: %d, yaw: %d, converted Psi: %d, sensed Psi: %d \n",
               yaw_error, pyaw, psi_conv, psi_sen);
        ae[0] = (plift + ppitch) / b - (yawPpar * yaw_error);
        ae[1] = (plift - proll) / b + (yawPpar * yaw_error);
        ae[2] = (plift - ppitch) / b - (yawPpar * yaw_error);
        ae[3] = (plift + proll) / b + (yawPpar * yaw_error);
}
*/
void yaw_controller()
{
        int8_t yaw_s = (int32_t)get_sensor(PSI) * 127 / 32768;
        // TODO why divide by 4 in old code?
        //printf("P in controller:%d\n", P);
        //P=5;
        yaw_act = P * (yaw/4 - yaw_s);
      /*  printf("yaw_error: %d, yaw: %d, converted Psi: %d, sensed Psi: %d P: %d \n",
               yaw_act, yaw/4, yaw_s, get_sensor(PSI),P);*/
}

void run_filters_and_control()
{
        // fancy stuff here
        // control loops and/or filters
}
void yaw_mode()
{
        yaw_controller();
        roll_act = roll;
        pitch_act = pitch;
        update_actuator();
}

void full_mode()
{
        outer_counter++;
        if (outer_counter % 4 == 0)
        {
                angle_controller();
                yaw_controller();
        }
        rate_controller();

        update_actuator();
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
        // Scale sensor va`lues
        // TODO check this works
        int8_t roll_s = (int32_t)get_sensor(PHI) * 127 / 32768;
        int8_t pitch_s = (int32_t)get_sensor(THETA) * 127 / 32768;

        int16_t roll_err = roll - roll_s;
        int16_t pitch_err = pitch - pitch_s;

        // Set new desired values for the rate_controller()
        pitch_rate_d = P1 * pitch_err;
        roll_rate_d = P1 * roll_err;
}
