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
#include "fixmath.h"

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

void fp_yaw_control(int16_t proll, int16_t ppitch, int16_t pyaw, uint16_t plift, uint16_t yawPpar, int16_t senPsi)
{
  //const fix16_t convIndex = F16(127/32768);
  const fix16_t convIndex = F16(0.00387573242188);
  register fix16_t froll = fix16_from_int(proll);
  register fix16_t fpitch = fix16_from_int(ppitch);
  register fix16_t flift = fix16_from_int(plift);
  fix16_t fyaw = fix16_from_int(pyaw>>2);
  fix16_t fyawPpar = fix16_from_int(yawPpar);
  fix16_t fsenPsi = fix16_from_int(senPsi);

  fix16_t convPsi = fix16_mul(fsenPsi, convIndex);
  register fix16_t fscaled_error = fix16_mul(fyawPpar,fix16_sub(fyaw, convPsi));

  //printf("yaw_error: %d,yaw: %d, converted Psi: %d, sensed Psi: %d \n", fix16_to_int(fix16_sub(fyaw, convPsi)),fix16_to_int(fyaw), fix16_to_int(convPsi), senPsi);
  ae[0] = fix16_to_int(fix16_sub(fix16_add(flift, fpitch), fscaled_error));
  ae[1] = fix16_to_int(fix16_add(fix16_sub(flift, froll), fscaled_error));
  ae[2] = fix16_to_int(fix16_sub(fix16_sub(flift, fpitch), fscaled_error));
  ae[3] = fix16_to_int(fix16_add(fix16_add(flift, froll), fscaled_error));
}

void int_yaw_control(int16_t proll, int16_t ppitch, int16_t pyaw, uint16_t plift, uint16_t yawPpar, int16_t psi_sen)
{
  int8_t b = 1;
  int8_t psi_conv = (int8_t)((psi_sen*127)/32768);
  //dcpsi_s = (int8_t)(((float)dcpsi/32768)*127);
  //psi_s = psi_s - dcpsi_s;  // value of yaw from calibrated point
  register int8_t yaw_error = (pyaw/4) - psi_conv;
  //printf("yaw_error: %d, yaw: %d, converted Psi: %d, sensed Psi: %d \n", yaw_error, pyaw, psi_conv, psi_sen);
  ae[0] = (plift + ppitch)/b - (yawPpar*yaw_error);
  ae[1] = (plift - proll)/b  + (yawPpar*yaw_error);
  ae[2] = (plift - ppitch)/b - (yawPpar*yaw_error);
  ae[3] = (plift + proll)/b  + (yawPpar*yaw_error);
}

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	//update_motors();
}
