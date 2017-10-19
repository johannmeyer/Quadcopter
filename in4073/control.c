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


void update_actuator()
{
      // Apply actuation
      // TODO is b still necessary for full control?
      int8_t b = 1;
      //int8_t overflow =0;
      ae[0] = (new_lift + pitch_act) / b - yaw_act + height_act;
      ae[1] = (new_lift - roll_act) / b + yaw_act  + height_act;
      ae[2] = (new_lift - pitch_act) / b - yaw_act + height_act;
      ae[3] = (new_lift + roll_act) / b + yaw_act  + height_act;
      for (int i = 0; i < 4; i++)         // Safe check for motor values
      {
              if (ae[i] < 180 && new_lift > 180)      // not allowing motor to stop rotation
                {
                        //overflow = 180 - ae[i];
                        ae[i] = 180;
                }
              /*  else if(overflow > 0 && ae[i] > 180 && new_lift > 180)
                {
                        ae[i] -= overflow;
                }*/
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

void yaw_controller()
{
        yaw_s = (int16_t)((int32_t)(get_sensor(SR) * 255) / 32768);  //can use PSI as well
        //printf("P in controller:%d\n", P);
        //P=5;

        yaw_act = P*((yaw/5) + yaw_s);      // yaw/4 to scale down yaw coming from joystick
        //yaw_prev = yaw_s;
        /*if (check_timer_flag())
        {
              if (yaw_rate !=0)
              {
              printf("yaw_error: %d, yaw: %d, converted Psi: %d, yaw_rate: %d yaw: %d \n",
             yaw_act, yaw>>1, yaw_s,yaw_rate,P);
              }
             clear_timer_flag();
        }*/
        if((new_lift - yaw_act) < MIN_VALUE )
        {
          yaw_overflow = MIN_VALUE - (new_lift - yaw_act);
          yaw_act -= yaw_overflow;
        }
        else if((new_lift + yaw_act) < MIN_VALUE )
        {
          yaw_overflow = MIN_VALUE - (new_lift + yaw_act);
          yaw_act += yaw_overflow;
        }


        if((new_lift + yaw_act) > MAX_VALUE )
        {
          yaw_overflow = (new_lift + yaw_act) - MAX_VALUE;
          yaw_act -= yaw_overflow;
        }
        else if((new_lift - yaw_act) > MAX_VALUE )
        {
          yaw_overflow = (new_lift - yaw_act) - MAX_VALUE;
          yaw_act += yaw_overflow;
        }
        //yaw_act *= -1;
        if (yaw_act !=0)
        {
          printf("yaw_error: %d, converted SR: %d, SR: %d, P: %d \n",
         yaw_act,  yaw_s, (int16_t) get_sensor(SR), P);
       }
}

void run_filters_and_control(uint8_t mode)
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
        //int32_t start_time = get_time_us();
        if (outer_counter++ % 3 == 0)
        {
                angle_controller();
                yaw_controller();
                //yaw_act = 0;//TODO  remove after testing
        }

        rate_controller();
        update_actuator();
        //printf("%ld\n", get_time_us()-start_time);
      //  outer_counter++;
}

void height_mode()
{
        static uint8_t height_counter;

        for (int i =0; i<5; i++)
        {
          height_d = get_baro();
        }
        if (height_counter++ % 5 == 0)
        {
            height_controller();
        }
        height_rate_controller();
        full_mode();

}

void height_controller()
{

        height_s= get_baro();
        height_rate_d = P3 *(height_d - height_s);      // pressure is less at more height
}

void height_rate_controller()
{
        height_s= get_baro();             // just to keep values of barometer updated
        static int16_t prev_height_rate=0;
        int16_t height_rate_s = (int16_t)(butter(get_sensor(SAX),THETA)>>18);
        prev_height_rate = prev_height_rate + height_rate_s;

        height_act = height_rate_d - P4*(prev_height_rate/100);

        if((new_lift + height_act) > MAX_VALUE )        //keeping a check on max height
        {
          height_overflow = (new_lift + height_act) - MAX_VALUE;
          height_act -= height_overflow;
        }
        if(height_act!=0)
        printf("height_act : %d, acc_sensor: %d, height_s: %ld, P3: %d, P4: %d",height_act, height_rate_s,height_s, P3, P4);

}

int32_t get_baro()
{
        //int8_t n=5;
        //static int i=0;
      //  static int32_t baro_values[5];
        int32_t baro_value;
        static int32_t avg_pressure = 0;

            //baro_values[i]= get_sensor(BARO);
            baro_value = get_sensor(BARO);
            printf("Baro : %ld  ",baro_value);
          /*  if(i<n)
            i++;
            else
            i=0;

            for (int j =0; j<n;j++)
            {
              avg_pressure += baro_values[j];
            }
            */
        avg_pressure = (4*avg_pressure)/5 + (baro_value/5);
        printf("avg:%ld \n",avg_pressure);
        return avg_pressure;
}



void rate_controller()
{
        /*
        Johann Meyer
         */
        // Scale sensor values
        // TODO check this works
        int16_t pitch_rate_s = (get_sensor(SQ)>>8);
        int16_t roll_rate_s = (get_sensor(SP)>>8);

        // Set actuation inputs
        pitch_act = pitch_rate_d +(P2 * pitch_rate_s);
        roll_act = roll_rate_d - (P2 * roll_rate_s);

        if((new_lift - roll_act) < MIN_VALUE )
        {
          roll_overflow = MIN_VALUE - (new_lift - roll_act);
          roll_act -= roll_overflow;
        }
        else if((new_lift + roll_act) < MIN_VALUE )
        {
          roll_overflow = MIN_VALUE - (new_lift + roll_act);
          roll_act += roll_overflow;
        }

        if((new_lift + roll_act) > MAX_VALUE )
        {
          roll_overflow = (new_lift + roll_act) - MAX_VALUE;
          roll_act -= roll_overflow;
        }
        else if((new_lift - roll_act) > MAX_VALUE )
        {
          roll_overflow = (new_lift - roll_act) - MAX_VALUE;
          roll_act += roll_overflow;
        }

        if((new_lift - pitch_act) < MIN_VALUE )
        {
          pitch_overflow = MIN_VALUE - (new_lift - pitch_act);
          pitch_act -= pitch_overflow;
        }
        else if((new_lift + pitch_act) < MIN_VALUE )
        {
          pitch_overflow = MIN_VALUE - (new_lift + pitch_act);
          pitch_act += pitch_overflow;
        }

        if((new_lift + pitch_act) > MAX_VALUE )
        {
          pitch_overflow = (new_lift + pitch_act) - MAX_VALUE;
          pitch_act -= pitch_overflow;
        }
        else if((new_lift - pitch_act) > MAX_VALUE )
        {
          pitch_overflow = (new_lift - pitch_act) - MAX_VALUE;
          pitch_act += pitch_overflow;
        }

        if(roll_act !=0)
        printf("pitch_rate_s: %d | roll_rate_s: %d | P1: %d | P2: %d \n", pitch_rate_s,roll_rate_s, P1, P2);

        //roll_act = 0; //TODO  remove after testing roll
}

void angle_controller()
{
        /*
        Johann Meyer
         */
        // Scale sensor values
        // TODO check this works
        int16_t roll_s = (get_sensor(PHI)>>9);
        int16_t pitch_s = (get_sensor(THETA)>>9);

        int16_t roll_err = (roll>>4) - roll_s;
        //printf("Roll error: %d\n", roll_err);
        int16_t pitch_err = (pitch>>4) - pitch_s;

        // Set new desired values for the rate_controller()
        pitch_rate_d = P1 * pitch_err;
        roll_rate_d = P1 * roll_err;
        printf("roll_s: %d | pitch_s: %d\n",roll_s, pitch_s);
}
