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
#include "logging.h"
#include "packet_uav.h"
#include "sensors.h"
#include <stdint.h>
#include <stdlib.h>
//#include "control.c"
#define SAFE_MODE 0
#define PANIC_MODE 1
#define MANUAL_MODE 2
#define CALIBRATION_MODE 3
#define YAW_MODE 4
#define FULL_MODE 5
#define RAW_MODE 6
#define HEIGHT_MODE 7
#define WIRELESS_MODE 8
#define EXIT_MODE 9

core *logCore;
/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */

void update_motors(void)
{
        if (ae[0] > 600)
        {
                ae[0] = 600;
        }
        if (ae[0] < 0)
        {
                ae[0] = 0;
        }
        if (ae[1] > 600)
        {
                ae[1] = 600;
        }
        if (ae[1] < 0)
        {
                ae[1] = 0;
        }
        if (ae[2] > 600)
        {
                ae[2] = 600;
        }
        if (ae[2] < 0)
        {
                ae[2] = 0;
        }
        if (ae[3] > 600)
        {
                ae[3] = 600;
        }
        if (ae[3] < 0)
        {
                ae[3] = 0;
        }
        motor[0] = ae[0];
        motor[1] = ae[1];
        motor[2] = ae[2];
        motor[3] = ae[3];
}

void process_mode(uint8_t mode)
{
        switch (mode)
        {
        case 0: // Safe mode
                if (prev_mode == PANIC_MODE || prev_mode == CALIBRATION_MODE ||
                    prev_mode == MANUAL_MODE || prev_mode == SAFE_MODE)
                {
                        ae[0] = 0;
                        ae[1] = 0;
                        ae[2] = 0;
                        ae[3] = 0;
                        update_motors();
                        prev_mode = SAFE_MODE;
                }
                break;

        case 1: // Panic mode
                if (prev_mode != SAFE_MODE || prev_mode != MANUAL_MODE ||
                    prev_mode != CALIBRATION_MODE) // Panic mode
                {
                        prev_mode = PANIC_MODE;
                }
                break;

        case 2: // Manual mode
                if (prev_mode == SAFE_MODE || prev_mode == MANUAL_MODE)
                {
                        // Lift, pitch, roll and yaw
                        b = 1;
                        d = 1;
                        ae[0] = ae[0] + (lift_delta + pitch_delta) / b -
                                yaw_delta / d;
                        ae[1] = ae[1] + (lift_delta - roll_delta) / b +
                                yaw_delta / d;
                        ae[2] = ae[2] + (lift_delta - pitch_delta) / b -
                                yaw_delta / d;
                        ae[3] = ae[3] + (lift_delta + roll_delta) / b +
                                yaw_delta / d;
                        update_motors();
                        prev_mode = MANUAL_MODE;
                }
                break;

        case 3: // Calibration mode
                if (prev_mode == SAFE_MODE || prev_mode == CALIBRATION_MODE)
                {
                        // bool isCalibrated() function is used to check
                        // calibration status
                        calibrate_sensors();
                        prev_mode = CALIBRATION_MODE;
                }
                break;

        case 4: // Yaw control mode
                if (prev_mode == YAW_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated()) ||
                    prev_mode == CALIBRATION_MODE)
                {
                        b = 1;
                        psi_s = ((float)psi / 32768) * 127;
                        dcpsi_s = ((float)dcpsi / 32768) * 127;
                        psi_s = psi_s -
                                dcpsi_s; // value of yaw from calibrated point
                        yaw_error = yaw - psi_s;

                        ae[0] = ae[0] + (lift_delta + pitch_delta) / b -
                                (yaw_parameter * yaw_error);
                        ae[1] = ae[1] + (lift_delta - roll_delta) / b +
                                (yaw_parameter * yaw_error);
                        ae[2] = ae[2] + (lift_delta - pitch_delta) / b -
                                (yaw_parameter * yaw_error);
                        ae[3] = ae[3] + (lift_delta + roll_delta) / b +
                                (yaw_parameter * yaw_error);

                        update_motors();

                        prev_mode = YAW_MODE;
                }
                break;

        case 5: // Full control mode
                if (prev_mode == FULL_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated()) ||
                    prev_mode == CALIBRATION_MODE || prev_mode == YAW_MODE)
                {
                        prev_mode = FULL_MODE;
                }
                break;

        case 6: // Raw control mode
                if (prev_mode == RAW_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated()) ||
                    prev_mode == CALIBRATION_MODE)
                {
                        prev_mode = RAW_MODE;
                }
                break;

        case 7: // Height control mode
                if (prev_mode == HEIGHT_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated()) ||
                    prev_mode == CALIBRATION_MODE || prev_mode == FULL_MODE)
                {
                        prev_mode = HEIGHT_MODE;
                }
                break;

        case 8: // Wireless mode
                if (prev_mode == WIRELESS_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated()) ||
                    prev_mode == CALIBRATION_MODE)
                {
                        prev_mode = WIRELESS_MODE;
                }
                break;

        case 9:
                demo_done = true;
                break;

        default:
                nrf_gpio_pin_toggle(RED);
        }
}

void calculate_values()
{
        lift_delta = lift - prev_lift;
        roll_delta = roll - prev_roll;
        pitch_delta = pitch - prev_pitch;
        yaw_delta = yaw - prev_yaw;
        prev_lift = lift;
        prev_roll = roll;
        prev_pitch = pitch;
        prev_yaw = yaw;
}

void battery_monitoring(uint8_t mode)
{
        adc_request_sample();

        if ((bat_volt < 11000) && (mode > 0))
        {
                // TODO Warnings to be sent as messages instead of prints.
                printf("Battery level is low(%d mV). Land the drone!\n",
                       bat_volt);
                if (bat_volt <= 10500)
                        process_mode(PANIC_MODE);
        }
}

bool isCharged(void)
{
        adc_request_sample();

        if (bat_volt < 11000)
                return false;
        else
                return true;
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
        logCore = (core *)malloc(sizeof(core));
        prev_mode = SAFE_MODE;
        demo_done = false;

        while (!demo_done)
        {

                int failed = decode(&logCore);
                calculate_values();
                process_mode(mode);
                if (!failed)
                {
                        // TODO Process the data e.g. change states
                        printf(
                            "Message:\t%x | %d | %d | %d | %x ||\t %x | %x | %x | %x\n",
                            mode, roll, pitch, yaw, lift, ae[0], ae[1], ae[2],
                            ae[3]);
                }

                if (check_timer_flag())
                {
                        if (counter++ % 20 == 0)
                                nrf_gpio_pin_toggle(BLUE);

// TODO Separate flight mode in the Makefile
#ifdef FLIGHT
                        battery_monitoring(prev_mode);
#endif
                        read_baro();

                        /*  if(isCalibrated())
                          {
                            write_log_entry(get_time_us(), prev_mode, logCore,
                          ae, get_sensor(PHI), get_sensor(THETA),
                          get_sensor(PSI),
                            get_sensor(SP), get_sensor(SQ), get_sensor(SR),
                          get_sensor(SAX), get_sensor(SAY), get_sensor(SAZ),
                            bat_volt, temperature, pressure);
                            print_last_log();
                          }
                          else
                          {
                            write_log_entry(get_time_us(), prev_mode, logCore,
                          ae, phi, theta, psi, sp, sq, sr, sax, say, saz,
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
