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
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include "control.c"

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c)
{
	switch (c)
	{
		case '0':									// Safe mode
			if (prev_mode!= 4 || prev_mode!= 5)
			{
				ae[0] = 0;
				ae[1] = 0;
				ae[2] = 0;
				ae[3] = 0;
				update_motors();
				prev_mode = 0;
			}
			break;

		case '1':									// Panic mode
			prev_mode = 1;
			break;

		case '2':									// Manual mode
			if (prev_mode = 0)
			{
				// Lift, pitch, roll and yaw
				ae[0] = ae[0] + b*(lift_delta + pitch_delta) - d*yaw_delta;
				ae[1] = ae[1] + b*(lift_delta - roll_delta)  + d*yaw_delta;
				ae[2] = ae[2] + b*(lift_delta - pitch_delta) - d*yaw_delta;
				ae[3] = ae[3] + b*(lift_delta + roll_delta)  + d*yaw_delta;

				update_motors();
				prev_mode = 2;
			}
			break;

		case '3':									// Calibration mode
			if (prev_mode = 0)
			{
				Calibration_flag = true;
				prev_mode = 3;
			}
			break;

		case '4':									// Yaw control mode
			prev_mode = 4;
			break;

		case '5':									// Full control mode
			prev_mode = 5;
			break;

		case '6':

			break;
		case '7':

			break;
		case '8':

			break;

		case 'q':
			ae[0] += 10;
			break;
		case 'a':
			ae[0] -= 10;
			if (ae[0] < 0) ae[0] = 0;
			break;
		case 'w':
			ae[1] += 10;
			break;
		case 's':
			ae[1] -= 10;
			if (ae[1] < 0) ae[1] = 0;
			break;
		case 'e':
			ae[2] += 10;
			break;
		case 'd':
			ae[2] -= 10;
			if (ae[2] < 0) ae[2] = 0;
			break;
		case 'r':
			ae[3] += 10;
			break;
		case 'f':
			ae[3] -= 10;
			if (ae[3] < 0) ae[3] = 0;
			break;
		case 27:
			demo_done = true;
			break;
		default:
			nrf_gpio_pin_toggle(RED);
	}
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

	uint32_t counter = 0;
	prev_mode = 0;
	demo_done = false;
	Calibration_flag = false;

	while (!demo_done)
	{
		if (rx_queue.count) process_key( dequeue(&rx_queue) );

		if (check_timer_flag())
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

			adc_request_sample();
			read_baro();

			printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			printf("%6d %6d %6d | ", phi, theta, psi);
			printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);

			clear_timer_flag();
		}

		if (check_sensor_int_flag())
		{
			get_dmp_data();
			run_filters_and_control();
		}
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
