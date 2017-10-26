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
#include "in4073.h"
#include "logging.h"
#include "packet_uav.h"
#include "sensors.h"
#include <stdint.h>
#include <stdlib.h>

#define BATTERY_LOW      1100 //11V in cV
#define BATTERY_CRITICAL 1050 //10.5V in cV

core *logCore;

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */

 /*------------------------------------------------------------------
  *  Function Name: determine_mode
  *  Made by: Ishu Goel
  *  Description: This function acts as Finite State Machine and
  *  update the current mode of the drone. It takes value of mode
  *  received from PC as input and allows change in mode only when
  *  the required conditions are met.
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
                        imu_init(true, 100);
                        uint32_t time = get_time_us();
                        printf("Calibrating sensors \n");
                        calibrate_sensors();
                        printf("Calibration done. Time elapsed: %ld ms\n", (get_time_us() - time) / 1000);
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
                    (prev_mode == SAFE_MODE && isCalibrated() && safe_flag))
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
                if ((prev_mode == RAW_MODE || prev_mode == SAFE_MODE) && safe_flag)
                {
                  imu_init(false, 100);
                  uint32_t time = get_time_us();
                  printf("Calibrating sensors \n");
                  calibrate_raw_sensors();
                  printf("Calibration done. Time elapsed: %ld ms\n", (get_time_us() - time) / 1000);
                  prev_mode = RAW_MODE;
                }
                break;

        case HEIGHT_MODE: // Height control mode
                if (prev_mode == FULL_MODE)
                {
                        prev_lift = new_lift;
                        prev_mode = HEIGHT_MODE;
                        height_mode_flag = true;      //to check the toggle of 7 key
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

        case EXIT_MODE:   //Exit the program
                process_mode(PANIC_MODE);
                prev_mode = EXIT_MODE;
                break;

        default:
                nrf_gpio_pin_toggle(RED);
        }
}


/*------------------------------------------------------------------
 *  Function Name: process_mode
 *  Made by: Ishu Goel
 *  Description: This function runs the mode provided as argument.
 *  Working of all the modes is described in this function.
 *------------------------------------------------------------------
 */

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
                        safe_flag = true;         //Safety condition to prevent change of mode if any user input of motor rotation is provided
                }
                else
                {
                        safe_flag = false;
                }
                break;

        case PANIC_MODE: // Panic mode
                printf("Panic mode reached\n");
                while (ae[0] > 0 || ae[1] > 0 || ae[2] > 0 || ae[3] > 0)    //reduce motor speed untill they are at standstill
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
                prev_mode = SAFE_MODE;      // switch to safe mode after motors have stopped
                break;

        case MANUAL_MODE: // Manual mode
                          // Lift, pitch, roll and yaw
                d = 2;
                ae[0] = new_lift + pitch - yaw * d;   // take inputs from user and apply directly
                ae[1] = new_lift - roll + yaw * d;
                ae[2] = new_lift - pitch - yaw * d;
                ae[3] = new_lift + roll + yaw * d;
                for (int i = 0; i < 4; i++)
                {
                        if (ae[i] < 180 && new_lift > 180)    // safety check to keep motors running
                                ae[i] = 180;
                }
                update_motors();
                break;

        case CALIBRATION_MODE: // Calibration mode

                prev_mode = CALIBRATION_MODE;
                break;

        case YAW_MODE: // Yaw control mode

                yaw_mode();

                break;

        case FULL_MODE: // Full control mode

                full_mode();

                break;

        case RAW_MODE: // Raw control mode
              full_mode();
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

/*------------------------------------------------------------------
 *  Function Name: calculate_values
 *  Made by: Ishu Goel
 *  Description: This function takes value of lift provided from PC
 *  as input and converts it into non-linear curve for better control
 *  of drone from joystick.
 *------------------------------------------------------------------
 */

void calculate_values()
{

        new_lift = 0;
        if (lift > 100)
                new_lift = 400 + 2 * ((uint16_t)lift - 100);
        else if (lift > 0) // make lift non-linear
                new_lift = 3 * (uint16_t)lift + 100;

        if (new_lift > MAX_LIFT)    // limit max value of lift
                new_lift = MAX_LIFT;
}

/*------------------------------------------------------------------
 *  Function Name: battery_monitoring
 *  Made by: Konstantinos-P. Metaxas
 *  Description: This function is called periodically to monitor the
 *  remaining capacity of the battery, inform the user when it is low
 *  and immediately enter PANIC_MODE when it is critical.
 *------------------------------------------------------------------
 */

void battery_monitoring(uint8_t mode)
{
        adc_request_sample();

        if ((bat_volt < BATTERY_LOW) && (mode > 0))
        {
                // TODO Warnings to be sent as messages instead of prints.
                printf("Battery level is low(%d mV). Land the drone!\n",
                       bat_volt);
                if (bat_volt <= BATTERY_CRITICAL)
                {
                        process_mode(PANIC_MODE);
                }
        }
}

/*------------------------------------------------------------------
 *  Function Name: isCharged
 *  Made by: Konstantinos-P. Metaxas
 *  Description: This function is called when a new mode is requested
 *  to prevent the drone entering an energy consuming mode when the
 *  battery is low.
 *------------------------------------------------------------------
 */

bool isCharged(void)
{
  #ifdef FLIGHT
        adc_request_sample();
        // TODO duplicate of battery_monitoring()?
        if (bat_volt < BATTERY_LOW)
                return false;
        else
  #endif
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
        select_log_mode(SENSOR_LOGGING);

        uint32_t counter = 0, maxCounter=0;

        logCore = (core *)malloc(sizeof(core));
        prev_mode = SAFE_MODE;
        demo_done = false;
        exit_mode_flag = false;
        safe_flag = false;
        height_lift_flag = false;
        height_mode_flag = false;

        P = 19; // for yaw control
        P1 = 13;
        P2 = 15;
        P3 = 20; // for height control
        P4 = 0;
        C1 = 1; // for Kalman Filter
        C2 = 1;

        while (!demo_done)
        {
                // TODO Process the data e.g. change states
                decode(&logCore);
                calculate_values();
                if (check_timer_flag())
                {
                        //int32_t start_time = get_time_us();

                        if (mode != prev_mode)
                        {
                                determine_mode(mode);
                        }
                        else if (height_mode_flag && mode == HEIGHT_MODE)
                        {
                                height_mode_flag = false;
                                height_lift_flag = false;
                                prev_mode = FULL_MODE;
                                printf("Full mode entered from main \n");
                        }

                        // printf("Message:\t%x | %d | %d | %d | %x ||\t %d | %d
                        // | %d | %d\n", prev_mode, roll,pitch, yaw,
                        // lift,ae[0],ae[1],ae[2],ae[3]);

                        if (counter++ % 5 == 0)
                        {
                          nrf_gpio_pin_toggle(BLUE);
                          printf("Message:\t%x | %d | %d | %d | %x ||\t %d | %d | %d | %d\n",
                                    prev_mode, roll, pitch, yaw, lift, ae[0],
                                    ae[1], ae[2], ae[3]);
                          printf("P1 : %d, P2: %d\n", P1, P2);
                        }
#ifdef FLIGHT
                        battery_monitoring(prev_mode);
#endif
                        clear_timer_flag();
                }
                if (check_sensor_int_flag())
                {
                  if(prev_mode != RAW_MODE) get_dmp_data();
                  else
                  {
                    counter = get_time_us();
                    get_filtered_data();
                    counter = get_time_us() - counter;
                    if(counter>maxCounter && counter < 5000000) maxCounter = counter;
                  }
                  read_baro();
                  /*if(isCalibrated())
                  {
                    printf("%ld\t", get_time_us());
                    printf("%6d %6d %6d | ", get_sensor(PHI), get_sensor(THETA), get_sensor(PSI));
                    printf("%6d %6d %6d\t", get_sensor(SAX), get_sensor(SAY), get_sensor(SAZ));
                    printf("%6d %6d %6d\n",get_sensor(SP), get_sensor(SQ), get_sensor(SR));
                  }
                  else
                  {
                    printf("%ld\t", get_time_us());
                    printf("%6d %6d %6d | ", phi, theta, psi);
                    printf("%6d %6d %6d\t", sax, say, saz);
                    printf("%6d %6d %6d\n", sp, sq, sr);
                  }*/
                  process_mode(prev_mode);
                }
        }

        free(logCore);

        printf("MaxFilter time:%ld\n", maxCounter);
        printf("\n\t Goodbye \n\n");
        nrf_delay_ms(100);

        NVIC_SystemReset();
}
