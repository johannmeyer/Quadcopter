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

#include "in4073.h"
#include "control.h"
#include "sensors.h"

void update_motors(void)
{
 if(ae[0]> 600)
   { ae[0] = 600; }
 if(ae[0] < 0)
   { ae[0] = 0;}
 if(ae[1]> 600)
   { ae[1] = 600; }
 if(ae[1] < 0)
   { ae[1] = 0;}
 if(ae[2]> 600)
   { ae[2] = 600; }
 if(ae[2] < 0)
   { ae[2] = 0;}
 if(ae[3]> 600)
   { ae[3] = 600; }
 if(ae[3] < 0)
   { ae[3] = 0;}
 motor[0] = ae[0];
 motor[1] = ae[1];
 motor[2] = ae[2];
 motor[3] = ae[3];
}

void process_mode(uint8_t current_mode)
{
	switch (current_mode)
	{
		case SAFE_MODE:									// Safe mode
				ae[0] = 0;
				ae[1] = 0;
				ae[2] = 0;
				ae[3] = 0;
				update_motors();
				if (lift == 0 && roll ==0 && pitch ==0 && yaw ==0)
				{
					safe_flag = true;
				}
				else
				{
					safe_flag = false;
				}
				break;

		case PANIC_MODE:			            // Panic mode
    		printf("Panic mode reached\n");
        while(ae[0] > 0 || ae[1] > 0 || ae[2] > 0 || ae[3]  > 0)
          {
					ae[0] -= PANIC_SPEED;
  				ae[1] -= PANIC_SPEED;
  				ae[2] -= PANIC_SPEED;
  				ae[3] -= PANIC_SPEED;
				  update_motors();
					}
					prev_mode = SAFE_MODE;
					break;

    case MANUAL_MODE:									// Manual mode
				// Lift, pitch, roll and yaw
        b=1;
        d=1;
				/*ae[0] = ae[0] + (lift_delta + pitch_delta)/b - yaw_delta/d;
				ae[1] = ae[1] + (lift_delta - roll_delta)/b  + yaw_delta/d;
				ae[2] = ae[2] + (lift_delta - pitch_delta)/b - yaw_delta/d;
				ae[3] = ae[3] + (lift_delta + roll_delta)/b  + yaw_delta/d;
				*/
				ae[0] = (new_lift + pitch)/b - yaw/d;
				ae[1] = (new_lift - roll)/b  + yaw/d;
				ae[2] = (new_lift - pitch)/b - yaw/d;
				ae[3] = (new_lift + roll)/b  + yaw/d;

				printf("data:%d %d %d",yaw,ae[0],ae[1]);
				update_motors();
				break;

		case CALIBRATION_MODE:									// Calibration mode
				//bool isCalibrated() function is used to check calibration status
				prev_mode = CALIBRATION_MODE;
				break;

		case YAW_MODE:									// Yaw control mode
			  b = 1;
        yaw_parameter = 5;
        psi_s = (int8_t)((get_sensor(PSI)*127)/32768);
        //dcpsi_s = (int8_t)(((float)dcpsi/32768)*127);
        //psi_s = psi_s - dcpsi_s;  // value of yaw from calibrated point
        yaw_error = (yaw/4) - psi_s;
				printf("yaw_error: %d,yaw: %d, psi_s: %d, psi: %d \n", yaw_error,yaw,psi_s,get_sensor(PSI) );
        ae[0] = (new_lift + pitch)/b - (yaw_parameter*yaw_error);
				ae[1] = (new_lift - roll)/b  + (yaw_parameter*yaw_error);
				ae[2] = (new_lift - pitch)/b - (yaw_parameter*yaw_error);
				ae[3] = (new_lift + roll)/b  + (yaw_parameter*yaw_error);
        update_motors();
        break;

		case FULL_MODE:									// Full control mode
      break;

		case RAW_MODE:                 // Raw control mode
			break;

		case HEIGHT_MODE:                 // Height control mode
			break;

		case WIRELESS_MODE:                 // Wireless mode
			break;

    case EXIT_MODE:
			demo_done = true;
      break;

		default:
			nrf_gpio_pin_toggle(RED);
	}
}


void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	//update_motors();
}
