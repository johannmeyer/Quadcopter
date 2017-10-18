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

#include "control.h"
#include "filters.h"
#include "in4073.h"
#include "logging.h"
#include "packet_uav.h"
#include "sensors.h"
#include <stdint.h>
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
        case SAFE_MODE: // Safe mode
                if (prev_mode == PANIC_MODE || prev_mode == CALIBRATION_MODE ||
                    prev_mode == SAFE_MODE)
                {
                        prev_mode = SAFE_MODE;
                        printf("Safe mode \n");
                }
                else
                {
                        prev_mode = PANIC_MODE;
                }
                break;

        case PANIC_MODE: // Panic mode
                if (prev_mode == SAFE_MODE || prev_mode == CALIBRATION_MODE)
                {
                        prev_mode = SAFE_MODE;
                        // printf("Safe mode \n");
                }
                else
                {
                        prev_mode = PANIC_MODE;
                }

                break;

        case MANUAL_MODE: // Manual mode
                if ((prev_mode == SAFE_MODE && safe_flag) ||
                    prev_mode == MANUAL_MODE)
                {
                        prev_mode = MANUAL_MODE;
                }
                break;

        case CALIBRATION_MODE: // Calibration mode
                if (prev_mode == SAFE_MODE)
                {
                        uint32_t time = get_time_us();
                        printf("Calibrating sensors \n");
                        calibrate_sensors();
                        printf("Calibration done. Time elapsed: %ld ms\n",
                               (get_time_us() - time) / 1000);
                        init_logging(FULL_LOGGING);
                        prev_mode = CALIBRATION_MODE;
                }
                break;

        case YAW_MODE: // Yaw control mode
                if (prev_mode == YAW_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated() && safe_flag))
                {
                        prev_mode = YAW_MODE;
                }
                break;

        case FULL_MODE: // Full control mode
                if (prev_mode == FULL_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated() && safe_flag) ||
                    prev_mode == YAW_MODE)
                {
                        prev_mode = FULL_MODE;
                }
                else if (prev_mode == HEIGHT_MODE && height_lift_flag)
                {
                        prev_mode = HEIGHT_MODE;
                }
                else if (prev_mode == HEIGHT_MODE && !height_lift_flag)
                {
                        prev_mode = FULL_MODE;
                        printf("Full mode entered from lift\n");
                }
                break;

        case RAW_MODE: // Raw control mode
                if (prev_mode == RAW_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated() && safe_flag))
                {
                        prev_mode = RAW_MODE;
                }
                break;

        case HEIGHT_MODE: // Height control mode
                if (prev_mode == FULL_MODE)
                {
                        prev_lift = new_lift;
                        prev_mode = HEIGHT_MODE;
                        height_mode_flag = true;
                        printf("Height mode entered \n");
                }
                break;

        case WIRELESS_MODE: // Wireless mode
                if (prev_mode == WIRELESS_MODE ||
                    (prev_mode == SAFE_MODE && isCalibrated() && safe_flag))
                {
                        prev_mode = WIRELESS_MODE;
                }
                break;

        case EXIT_MODE:
                process_mode(PANIC_MODE);
                // demo_done = true;
                prev_mode = EXIT_MODE;
                break;

        default:
                nrf_gpio_pin_toggle(RED);
        }
}

void process_mode(uint8_t current_mode)
{
        switch (current_mode)
        {
        case SAFE_MODE: // Safe mode
                ae[0] = 0;
                ae[1] = 0;
                ae[2] = 0;
                ae[3] = 0;
                update_motors();
                if (lift == 0 && roll == 0 && pitch == 0 && yaw == 0)
                {
                        safe_flag = true;
                }
                else
                {
                        safe_flag = false;
                }
                break;

        case PANIC_MODE: // Panic mode
                printf("Panic mode reached\n");
                while (ae[0] > 0 || ae[1] > 0 || ae[2] > 0 || ae[3] > 0)
                {
                        if (check_timer_flag()) // timer of 50ms
                        {
                                ae[0] -= PANIC_SPEED;
                                ae[1] -= PANIC_SPEED;
                                ae[2] -= PANIC_SPEED;
                                ae[3] -= PANIC_SPEED;
                                update_motors();
                                clear_timer_flag();
                        }
                }
                safe_flag = false;
                prev_mode = SAFE_MODE;
                break;

        case MANUAL_MODE: // Manual mode
                          // Lift, pitch, roll and yaw
                d = 2;
                ae[0] = new_lift + pitch - yaw * d;
                ae[1] = new_lift - roll + yaw * d;
                ae[2] = new_lift - pitch - yaw * d;
                ae[3] = new_lift + roll + yaw * d;
                for (int i = 0; i < 4; i++)
                {
                        if (ae[i] < 180 && new_lift > 180)
                                ae[i] = 180;
                }
                update_motors();
                break;

        case CALIBRATION_MODE: // Calibration mode
                // bool isCalibrated() function is used to check calibration
                // status
                prev_mode = CALIBRATION_MODE;
                break;

        case YAW_MODE: // Yaw control mode

                // int_yaw_control(roll, pitch, yaw, new_lift, 5,
                // get_sensor(PSI));
                // fp_yaw_control(roll, pitch, yaw, new_lift, 5,
                // get_sensor(PSI));
                yaw_mode();
                // update_motors();

                break;

        case FULL_MODE: // Full control mode

                //full_mode();

                break;

        case RAW_MODE: // Raw control mode
                break;

        case HEIGHT_MODE: // Height control mode
                if (new_lift != prev_lift)
                {
                        height_lift_flag = false;
                        printf("lift changed \n");
                }
                else
                {
                        height_lift_flag = true;
                        height_mode();
                }

                break;

        case WIRELESS_MODE: // Wireless mode
                break;

        case EXIT_MODE:
                demo_done = true;
                break;

        default:
                nrf_gpio_pin_toggle(RED);
        }
}

void calculate_values()
{

        new_lift = 3 * lift;
        if (lift > 0) // make lift non linear
                new_lift += 100;
        if (lift > 100)
                new_lift = 400 + 2 * (lift - 100);
        if (new_lift > MAX_LIFT)
                new_lift = MAX_LIFT;
}

void battery_monitoring(uint8_t mode)
{
        adc_request_sample();

        if ((bat_volt < 1100) && (mode > 0))
        {
                // TODO Warnings to be sent as messages instead of prints.
                printf("Battery level is low(%d mV). Land the drone!\n",
                       bat_volt);
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
        // TODO duplicate of battery_monitoring()?
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
        select_log_mode(SHORT_LOGGING);

        uint32_t counter = 0;
        // int32_t height_value;
        // int16_t acc_x;

        logCore = (core *)malloc(sizeof(core));
        prev_mode = SAFE_MODE;
        demo_done = false;
        exit_mode_flag = false;
        safe_flag = false;
        height_lift_flag = false;
        height_mode_flag = false;

        P = 19; // for yaw control
        P1 = 14;
        P2 = 21;
        P3 = 1; // for height control
        P4 = 1;
        C1 = 1; // for Kalman Filter
        C2 = 1;

        while (!demo_done)
        {
                // printf("new mode : %d, prev_mode : %d\n",mode , prev_mode);
                // TODO Process the data e.g. change states
                decode(&logCore);
                calculate_values();
                if (check_timer_flag())
                {
                        //int32_t start_time = get_time_us();

                        if (mode != prev_mode)
                        {
                                // printf("Determine mode \n");
                                determine_mode(mode);
                        }
                        else if (height_mode_flag && mode == HEIGHT_MODE)
                        {
                                height_mode_flag = false;
                                height_lift_flag = false;
                                prev_mode = FULL_MODE;
                                printf("Full mode entered from main \n");
                        }
                        process_mode(prev_mode);
                        // printf("Message:\t%x | %d | %d | %d | %x ||\t %d | %d
                        // | %d | %d\n", prev_mode, roll,pitch, yaw,
                        // lift,ae[0],ae[1],ae[2],ae[3]);

                        if (counter++ % 20 == 0)
                        {
                                nrf_gpio_pin_toggle(BLUE);
                                printf(
                                    "Message:\t%x | %d | %d | %d | %x ||\t %d | %d | %d | %d\n",
                                    prev_mode, roll, pitch, yaw, lift, ae[0],
                                    ae[1], ae[2], ae[3]);
                                // printf("P1 : %d, P2: %d \n", P1, P2);
                                /*if(isCalibrated())
                      {
                                                read_baro();
                                                acc_x = get_sensor(SAX);
                                                height_value =
                      (0.3*(abs(butter(acc_x,THETA)>>10)/10)) +
                      (0.7*(abs(get_sensor(BARO))/4));
                                                //height_value =
                      (butter(acc_x,THETA)>>10);
                                                //printf("%6d %6d %6d | ",
                      get_sensor(PHI), get_sensor(THETA), get_sensor(PSI));
                              //printf("Gyro: %6d %6d %6d %ld \n ",
                      get_sensor(SP), get_sensor(SQ), get_sensor(SR),pressure);
                              printf("Acc:%6d %ld | %ld
                      \n",acc_x,height_value,pressure);
                                        }*/
                                /*else
                                {
                                                          printf("%6d %6d %6d |
                                ", phi, theta, psi);
                                        printf("%6d %6d %6d | ", sp, sq, sr);
                                        printf("%6d %6d %6d | ", sax, say, saz);
                                                          printf("\n");
                                }*/
                        }

// TODO Separate flight mode in the Makefile
#ifdef FLIGHT
                        battery_monitoring(prev_mode);
#endif
                        read_baro();

                        /*if(isCalibrated())
                          {
                                                    printf("%6d %6d %6d | ",
                          get_sensor(PHI), get_sensor(THETA), get_sensor(PSI));
                                  printf("%6d %6d %6d | ", get_sensor(SP),
                          get_sensor(SQ), get_sensor(SR));
                                  printf("%6d %6d %6d | ", get_sensor(SAX),
                          get_sensor(SAY), get_sensor(SAZ));
                                                    printf("\n");
                          }
                          else
                          {
                                                    printf("%6d %6d %6d | ",
                          phi, theta, psi);
                                  printf("%6d %6d %6d | ", sp, sq, sr);
                                  printf("%6d %6d %6d | ", sax, say, saz);
                                                    printf("\n");
                          }*/
                        clear_timer_flag();
                }
                if (check_sensor_int_flag())
                {
                        get_dmp_data();
                        read_baro();
                        run_filters_and_control(prev_mode);
                }
        }

        free(logCore);

        printf("\n\t Goodbye \n\n");
        nrf_delay_ms(100);

        NVIC_SystemReset();
}
