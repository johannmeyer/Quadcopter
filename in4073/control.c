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
#include "fixedp.h"
#include "in4073.h"
#include "sensors.h"
#include <stdlib.h>
#include "filters.h"


/*
Local Variables
 */
uint32_t control_counter =0 ;
uint8_t outer_counter = 0;
int16_t yaw_prev, yaw_s;
int16_t pitch_rate_d, roll_rate_d, yaw_rate;
int16_t pitch_act, roll_act, yaw_act, height_act;
int16_t yaw_overflow, pitch_overflow, roll_overflow, height_overflow;
int32_t height_s, height_d;
int16_t height_rate_d;
/*
Local Function Prototypes
 */
void update_actuator();
void rate_controller();
void angle_controller();
void yaw_controller();
void height_controller();
void height_rate_controller();
int32_t get_baro();

/*------------------------------------------------------------------
 *  Function Name: update_actuator
 *  Made by: Ishu Goel
 *  Description: This function updates the value of actuators after the
 *  correction from roll, pitch, yaw and height is updated from their
 *  respective functions.
 *------------------------------------------------------------------
 */

void update_actuator()
{
      // Apply actuation
      ae[0] = new_lift + pitch_act - yaw_act + height_act;
      ae[1] = new_lift - roll_act  + yaw_act  + height_act;
      ae[2] = new_lift - pitch_act - yaw_act + height_act;
      ae[3] = new_lift + roll_act  + yaw_act  + height_act;
      for (int i = 0; i < 4; i++)         // Safe check for motor values
      {
              if (ae[i] < 180 && new_lift > 180)      // not allowing motor to stop rotation
                {
                        ae[i] = 180;
                }

              if(new_lift < 160)  // if lift is not provided, do not respond to yaw error
              ae[i] = 0;
      }

      update_motors();
}

void update_motors(void)
{
        for (int i = 0; i < 4; i++)
        {
                if (ae[i] > MAX_MOTOR)
                        ae[i] = MAX_MOTOR;
                else if (ae[i] < 0)
                        ae[i] = 0;
        }

        motor[0] = ae[0];
        motor[1] = ae[1];
        motor[2] = ae[2];
        motor[3] = ae[3];
}


/*------------------------------------------------------------------
 *  Function Name: yaw_controller
 *  Made by: Ishu Goel
 *  Description: This function runs the control loop for yaw control.
 *  It measures the rate of yaw received from gyro sensor and subtracts
 *  it from the yaw rate received from joystick. The yaw correction is
 *  calculated after multiplying the difference with the P gain.
 *------------------------------------------------------------------
 */
void yaw_controller()
{
        yaw_s = (int16_t)((int32_t)(get_sensor(SR) * 255) / 32768);  //can use PSI as well
        //printf("P in yaw controller:%d\n", P);

        yaw_act = P*((yaw/5) + yaw_s);      // yaw/5 to scale down yaw coming from joystick

        if((new_lift - yaw_act) < MIN_VALUE  && new_lift > MIN_VALUE )  // to check overflow caused by yaw correction
        {
          yaw_overflow = MIN_VALUE - (new_lift - yaw_act);
          yaw_act -= yaw_overflow;
        }
        else if((new_lift + yaw_act) < MIN_VALUE && new_lift > MIN_VALUE)
        {
          yaw_overflow = MIN_VALUE - (new_lift + yaw_act);
          yaw_act += yaw_overflow;
        }


        if((new_lift + yaw_act) > MAX_VALUE && new_lift > MIN_VALUE)
        {
          yaw_overflow = (new_lift + yaw_act) - MAX_VALUE;
          yaw_act -= yaw_overflow;
        }
        else if((new_lift - yaw_act) > MAX_VALUE && new_lift > MIN_VALUE)
        {
          yaw_overflow = (new_lift - yaw_act) - MAX_VALUE;
          yaw_act += yaw_overflow;
        }
        //yaw_act *= -1;
        /*if (yaw_act !=0)
        {
          printf("yaw_error: %d, converted SR: %d, SR: %d, P: %d \n",
         yaw_act,  yaw_s, (int16_t) get_sensor(SR), P);
       }*/
}

/*void run_filters_and_control(uint8_t mode)
{
  switch (mode)
  {
    case FULL_MODE:
      //rate_controller();
      //update_actuator();
      full_mode();
      break;
    case RAW_MODE:
      full_mode();
    default:
    //printf("not full mode\n");
    break;
  }
        // fancy stuff here
        // control loops and/or filters

}*/

/*------------------------------------------------------------------
 *  Function Name: yaw_mode
 *  Made by: Ishu Goel
 *  Description: This function runs when yaw mode is received from the
 *  PC. It calls the yaw_controller function keeping roll and pitch values
 *  same as received from the PC.
 *------------------------------------------------------------------
 */

void yaw_mode()
{
        yaw_controller();
        roll_act = roll;
        pitch_act = pitch;
        update_actuator();
}

/*------------------------------------------------------------------
 *  Function Name: full_mode()
 *  Made by: Johann Meyer
 *  Description: Runs the control loops required by FULL_MODE at the
 *  required frequencies
 *------------------------------------------------------------------
 */
void full_mode()
{
        // Outer loop must be updated slower than inner loop
        if (outer_counter++ % OUTER_LOOP_FREQ == 0)
        {
                angle_controller();
                yaw_controller();
        }

        rate_controller();
        update_actuator();
}


/*------------------------------------------------------------------
 *  Function Name: height_mode
 *  Made by: Ishu Goel
 *  Description: This function runs the height control loop. It first
 *  stores the desired height and run height_rate_controller(inner loop)
 *  function at higher frequency than height_controller(outer_loop) function.
 *  In height mode, the functionalities of full mode are run as well.
 *------------------------------------------------------------------
 */
void height_mode()
{
        static uint8_t height_counter;

        for (int i =0; i<5; i++)      // to get proper average of barometer values in first time
        {
          height_d = get_baro();        // desired height of drone
        }
        if (height_counter++ % 3 == 0)
        {
            height_controller();      // outer control loop is run at low frequency
        }
        height_rate_controller();
        full_mode();                // excpet height evrything works as full mode

}

/*------------------------------------------------------------------
 *  Function Name: height_controller
 *  Made by: Ishu Goel
 *  Description: This function runs the outer control loop of height control.
 *  It takes the value of barometer and subtracts it from the desired value
 *  of height. The desired height rate is calculated after multiplying
 *  the difference with the P gain.
 *------------------------------------------------------------------
 */
void height_controller()
{

        height_s= get_baro();
        height_rate_d = P3 *(height_d - height_s);      // pressure is less at more height
}

/*------------------------------------------------------------------
 *  Function Name: height_rate_controller
 *  Made by: Ishu Goel
 *  Description: This function runs the inner control loop of height control.
 *  It takes the filtered values of accelerometer and integrate it to get the
 *  rate of height. The height correction is calculated by subtracting the
 *  desired height rate from the scaled value of height rate obtained from
 *  accelerometer.
 *------------------------------------------------------------------
 */
void height_rate_controller()
{
        height_s = get_baro();             // just to keep values of barometer updated
        static int16_t prev_height_rate=0;
        int16_t height_rate_s = (int16_t)(butter(get_sensor(SAX),THETA)>>18);// 10 initially
        prev_height_rate = prev_height_rate + height_rate_s;

        height_act = height_rate_d - P4*(prev_height_rate/100);

        if((new_lift + height_act) > MAX_VALUE && new_lift > MIN_VALUE )        //keeping a check on max height
        {
          height_overflow = (new_lift + height_act) - MAX_VALUE;
          height_act -= height_overflow;
        }
        if((new_lift + height_act) < MIN_VALUE && new_lift > MIN_VALUE)        //keeping a check on min height
        {
          height_overflow = MIN_VALUE - (new_lift + height_act);
          height_act += height_overflow;
        }
      //  if(height_act!=0)
      static int count = 0;
      if (count++%10)
        printf("height_act : %d, acc_sensor: %d, height_s: %ld, P3: %d, P4: %d \n",height_act, height_rate_s,height_s, P3, P4);

}

/*------------------------------------------------------------------
 *  Function Name: get_baro
 *  Made by: Ishu Goel
 *  Description: This function calculates the moving average of barometer
 *  values.
 *------------------------------------------------------------------
 */

int32_t get_baro()
{
        int32_t baro_value;
        static int32_t avg_pressure = 0;

        baro_value = get_sensor(BARO);
        printf("Baro : %ld  ",baro_value);

        avg_pressure = (4*avg_pressure)/5 + (baro_value/5);   // Moving average of 5 Barometer values
        printf("avg:%ld \n",avg_pressure);
        return avg_pressure;
}


/*------------------------------------------------------------------
 *  Function Name: rate_controller()
 *  Made by: Johann Meyer
 *  Description: Runs the inner loop controller for FULL_MODE. The
 *  controller is identical for both pitch and roll axes.
 *------------------------------------------------------------------
 */
void rate_controller()
{
        // Get required precision from noisy sensors
        int16_t pitch_rate_s = (get_sensor(SQ)>>8);
        int16_t roll_rate_s = (get_sensor(SP)>>8);

        // Set actuation inputs
        pitch_act = pitch_rate_d +(P2 * pitch_rate_s);
        roll_act = roll_rate_d - (P2 * roll_rate_s);

        // Prevent actuators from switching off
        if((new_lift - roll_act) < MIN_VALUE && new_lift > MIN_VALUE)
        {
          roll_overflow = MIN_VALUE - (new_lift - roll_act);
          roll_act -= roll_overflow;
        }
        else if((new_lift + roll_act) < MIN_VALUE && new_lift > MIN_VALUE)
        {
          roll_overflow = MIN_VALUE - (new_lift + roll_act);
          roll_act += roll_overflow;
        }

        if((new_lift + roll_act) > MAX_VALUE && new_lift > MIN_VALUE)
        {
          roll_overflow = (new_lift + roll_act) - MAX_VALUE;
          roll_act -= roll_overflow;
        }
        else if((new_lift - roll_act) > MAX_VALUE && new_lift > MIN_VALUE )
        {
          roll_overflow = (new_lift - roll_act) - MAX_VALUE;
          roll_act += roll_overflow;
        }

        if((new_lift - pitch_act) < MIN_VALUE && new_lift > MIN_VALUE )
        {
          pitch_overflow = MIN_VALUE - (new_lift - pitch_act);
          pitch_act -= pitch_overflow;
        }
        else if((new_lift + pitch_act) < MIN_VALUE && new_lift > MIN_VALUE )
        {
          pitch_overflow = MIN_VALUE - (new_lift + pitch_act);
          pitch_act += pitch_overflow;
        }

        if((new_lift + pitch_act) > MAX_VALUE && new_lift > MIN_VALUE)
        {
          pitch_overflow = (new_lift + pitch_act) - MAX_VALUE;
          pitch_act -= pitch_overflow;
        }
        else if((new_lift - pitch_act) > MAX_VALUE && new_lift > MIN_VALUE)
        {
          pitch_overflow = (new_lift - pitch_act) - MAX_VALUE;
          pitch_act += pitch_overflow;
        }

}

/*------------------------------------------------------------------
 *  Function Name: angle_controller()
 *  Made by: Johann Meyer
 *  Description: Runs the outer control loop for FULL_MODE. It sets
 *  the desired angle of the UAV for both pitch and roll axes.
 *------------------------------------------------------------------
 */
void angle_controller()
{
        // Scale sensor values
        int16_t roll_s = (get_sensor(PHI)>>9);
        int16_t pitch_s = (get_sensor(THETA)>>9);

        // Scale user commanded values
        int16_t roll_err = (roll>>4) - roll_s;
        int16_t pitch_err = (pitch>>4) - pitch_s;

        // Set new desired values for the rate_controller()
        pitch_rate_d = P1 * pitch_err;
        roll_rate_d = P1 * roll_err;
}
