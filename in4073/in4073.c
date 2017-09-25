/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include <stdint.h>
#include "packet_uav.h"
#include "logging.h"
#include "sensors.h"
#include "control.h"
#include <stdlib.h>



core *logCore;
/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void determine_mode(uint8_t mode)
{

	switch (mode)
	{
		case SAFE_MODE:									// Safe mode
			if (prev_mode == PANIC_MODE || prev_mode == CALIBRATION_MODE || prev_mode == SAFE_MODE)
			{
				prev_mode = SAFE_MODE;
				printf("Safe mode1 \n");
			}
			else
			{
				prev_mode = PANIC_MODE;
			}
			break;

		case PANIC_MODE:			            // Panic mode
		 if (prev_mode == SAFE_MODE || prev_mode == CALIBRATION_MODE)
		 {
			prev_mode = SAFE_MODE;
			printf("Safe mode \n");
		 }
		else
			{
				prev_mode = PANIC_MODE;
			}

		break;

		case MANUAL_MODE:									// Manual mode
			if ((prev_mode == SAFE_MODE && safe_flag)  || prev_mode == MANUAL_MODE)
			{
				prev_mode = MANUAL_MODE;
			}
			break;

		case CALIBRATION_MODE:									// Calibration mode
			if (prev_mode == SAFE_MODE )
			{
				printf("Calibrating sensors \n");
				calibrate_sensors();
				printf("Calibration done \n");
				prev_mode = CALIBRATION_MODE;
			}
			break;

		case YAW_MODE:									// Yaw control mode
			if(prev_mode == YAW_MODE ||(prev_mode == SAFE_MODE && isCalibrated() && safe_flag))
			{
				prev_mode = YAW_MODE;
			}
			break;

		case FULL_MODE:									// Full control mode
			if(prev_mode == FULL_MODE || (prev_mode == SAFE_MODE && isCalibrated() && safe_flag) || prev_mode == YAW_MODE )
			{
				prev_mode = FULL_MODE;
			}
			break;

		case RAW_MODE:                 // Raw control mode
			if(prev_mode == RAW_MODE || (prev_mode == SAFE_MODE && isCalibrated()&& safe_flag))
			{
				prev_mode = RAW_MODE;
			}
			break;

		case HEIGHT_MODE:                 // Height control mode
			if(prev_mode == HEIGHT_MODE || (prev_mode == SAFE_MODE && isCalibrated() && safe_flag) || prev_mode == FULL_MODE)
			{
				prev_mode = HEIGHT_MODE;
			}
			break;

		case WIRELESS_MODE:                 // Wireless mode
			if(prev_mode == WIRELESS_MODE || (prev_mode == SAFE_MODE && isCalibrated() && safe_flag))
			{
				prev_mode = WIRELESS_MODE;
			}
			break;

		case EXIT_MODE:
		process_mode(PANIC_MODE);
		//demo_done = true;
		prev_mode = EXIT_MODE;
		break;

		default:
			nrf_gpio_pin_toggle(RED);
	}

}


void calculate_values()
{
	new_lift = 3*lift;
	if(new_lift > 600) new_lift = 600;
  lift_delta = new_lift - prev_lift;
  roll_delta = roll - prev_roll;
  pitch_delta = pitch - prev_pitch;
  yaw_delta = yaw - prev_yaw;
  prev_lift = new_lift;
  prev_roll = roll;
  prev_pitch = pitch;
  prev_yaw = yaw;
}

void battery_monitoring(uint8_t mode)
{
  adc_request_sample();

  if((bat_volt < 1100) && (mode>0))
  {
    //TODO Warnings to be sent as messages instead of prints.
    printf("Battery level is low(%d mV). Land the drone!\n",bat_volt);
    if (bat_volt <= 1050)
		{
			mode = PANIC_MODE;
			process_mode(PANIC_MODE);
			}
  }
}

bool isCharged(void)
{
  adc_request_sample();

  if (bat_volt < 11000) return false;
  else return true;
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */
int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);
	baro_init();
	spi_flash_init();
	ble_init();
  select_log_mode(SHORT_LOGGING);

	uint32_t counter = 0;
  logCore = (core *) malloc(sizeof(core));
	prev_mode = SAFE_MODE;
	demo_done = false;
	exit_mode_flag = false;
	safe_flag = false;

	while (!demo_done)
	{
		if (rx_queue.count >= sizeof(packet))
    {
      int failed = decode(&logCore);
      calculate_values();
			if (mode != prev_mode)
			{
				//printf("Determine mode \n");
			determine_mode(mode);
			}
			process_mode(prev_mode);
			//printf("new mode : %d, prev_mode : %d\n",mode , prev_mode);
      if(!failed)
      {
        // TODO Process the data e.g. change states
        printf("Message:\t%x | %d | %d | %d | %x ||\t %d | %d | %d | %d\n", prev_mode, roll, pitch, yaw, lift,ae[0],ae[1],ae[2],ae[3]);
      }
    }

		if (check_timer_flag())
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

      //TODO Separate flight mode in the Makefile
      #ifdef FLIGHT
			battery_monitoring(prev_mode);
      #endif
			read_baro();

    /*  if(isCalibrated())
      {
        write_log_entry(get_time_us(), prev_mode, logCore, ae, get_sensor(PHI), get_sensor(THETA), get_sensor(PSI),
        get_sensor(SP), get_sensor(SQ), get_sensor(SR), get_sensor(SAX), get_sensor(SAY), get_sensor(SAZ),
        bat_volt, temperature, pressure);
        print_last_log();
      }
      else
      {
        write_log_entry(get_time_us(), prev_mode, logCore, ae, phi, theta, psi, sp, sq, sr, sax, say, saz,
        bat_volt, temperature, pressure);
        print_last_log();
      }
*/
			clear_timer_flag();
		}

		if (check_sensor_int_flag())
		{
			get_dmp_data();
			run_filters_and_control();
		}
	}

  free(logCore);

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
