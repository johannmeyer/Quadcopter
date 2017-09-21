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

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	//update_motors();
}
