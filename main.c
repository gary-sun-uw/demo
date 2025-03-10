/*
Simple program to demonstrate communicating with a BMI270 over SPI on an MSP430.
This example uses the MSP-EXP430FR6989 Launchpad and its EUSCI_B0 SPI interface,
which has the following pinout:

P1.4: UCB0CLK (serial clock) -> BMI270 pin 13
P1.5: CSB (chip select bar) -> BMI270 pin 12
P1.6: UCB0SIMO (peripheral in, controller out) -> BMI270 pin 14
P1.7: UCB0SOMI (peripheral out, controller in) -> BMI270 pin 1
*/

#include <stdio.h>
#include <driverlib.h>
#include "BMI270_SensorAPI/bmi270.h"
#include "BMI270_SensorAPI/bmi2_defs.h"
#include "bmi270_spi.h"
#include "gpio.h"
#include "hal_LCD.h"
#include "intrinsics.h"
#include "msp430fr5xx_6xxgeneric.h"
#include "util.h"
#include <msp430.h>
#include "interrupt.h"
#include "uart.h"
#include "classification/analyze.h"
#include "classification/classify.h"
#include "commons/data_formats.h"
#include "commons/circular_buffer.h"
#include "commons/constants.h"
#include "StopWatchMode.h"

//#define CAPTURE
//#define PRINT_CAPTURE
//#define CLASSIFY

unsigned int int1_status = 0;
unsigned int int2_status = 0;
unsigned int int1_status_result = 0;
unsigned int int2_status_result = 0;
unsigned int interval_setting_mode = 0;

volatile unsigned int counter = 0;
volatile int centisecond = 0;
Calendar currentTime;
unsigned int x = 0;
unsigned int y = 0;

#define INT_PORT GPIO_PORT_P2
#define INT1_PIN GPIO_PIN4
#define INT2_PIN GPIO_PIN5

#define DATA_LEN 190
#pragma PERSISTENT(sensor_data)
static struct BMI2SensData sensor_data[DATA_LEN] = { { { 0 } } };
static struct BMI2SensData sensor_data_snapshot =  { { 0 } } ;

#define NO_MOTION_SNAPSHOT_LEN 5
static struct BMI2SensData no_motion_snapshot[NO_MOTION_SNAPSHOT_LEN] = { { { 0 } } };



static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

void init_spi() {
    // Set pins P1.6 and P1.4 as UCB0SIMO and UCB0CLK respectively
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 + GPIO_PIN4,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    // Set pin P1.7 as UCB0SOMI
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN7,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    // While it is possible to set this as an SPI chip select pin (UCB0STE), it should instead
    // be set just as a normal GPIO output, so that it doesn't get driven low after every write.
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
    __delay_cycles(100);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    EUSCI_B_SPI_initMasterParam param = {
        .selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
        .clockSourceFrequency = CS_getSMCLK(),
        .desiredSpiClock = 1000000,
        // Per the datasheet, the BMI270 supports either 00 (the current setting) or 11 for clockPhase and clockPolarity.
        // This is automatically detected by the BMI270.
        .clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
        .clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
        .msbFirst = EUSCI_B_SPI_MSB_FIRST,
        .spiMode = EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW
    };
    EUSCI_B_SPI_initMaster(SPI_BASE, &param);
    // Honestly I have no idea what this next line does, it might do nothing
    EUSCI_B_SPI_select4PinFunctionality(SPI_BASE, EUSCI_B_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE);
    EUSCI_B_SPI_enable(SPI_BASE);
}

void init_clk() {
    // Set DCO Frequency to 4 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_3);

    // Configure MCLK, SMCLK to be sourced by DCOCLK
    CS_initClockSignal(CS_MCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1); // 4 MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1); // 4 MHz
}

void init_uart() {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configure UART
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 2;
    param.firstModReg = 2;
    param.secondModReg = 0xDD;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A1_BASE, &param)) {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A1_BASE);
}

void init_GPIO() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1|GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
    
    // LED
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN7);

    //  // Configure button S1 (P1.1) interrupt
    // GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    // GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    // GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    // GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // // Configure button S2 (P1.2) interrupt
    // GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    // GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    // GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    // GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // P2.4 interrupt
    // GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    // GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN4);
    // GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);
    //GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN4);

    // P2.5 interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);
}



int main(void) {
    static struct bmi2_dev bmi;

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    init_clk();
    init_spi();
    Init_LCD();
    init_GPIO();
    init_bmi_device(&bmi);
    init_uart();

    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);

    // #define ACTIONS_BUFFER_ON

    #ifdef ACTIONS_BUFFER_ON
    uint8_t actions_buffer_size = 3;
    int i;
    ActionTypeCircularBuffer recorded_actions;
    init_actiontype_circular_buffer(&recorded_actions, actions_buffer_size);
    #endif

    //__enable_interrupt();

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Accel sensor and no-motion feature are listed in array. */
    #define SENS_NUM 4
    uint8_t sens_list[SENS_NUM] = { BMI2_ACCEL, BMI2_GYRO, BMI2_WRIST_GESTURE, BMI2_NO_MOTION };

    struct bmi2_feat_sensor_data sensor_activity_data = { 0 };
    sensor_activity_data.type = BMI2_STEP_ACTIVITY;
    const char *activity_output[4] = { "STILL ", "WALK  ", "RUN   ", "UNKNOW" };

    char* current_state = "UNKNOW";
    uint32_t indx = 0;

    const char *gesture_output[6] =
    { "unknown_gesture", "push_arm_down", "pivot_up", "wrist_shake_jiggle", "flick_in", "flick_out" };
    struct bmi2_feat_sensor_data sens_data = { 0 };
    sens_data.type = BMI2_WRIST_GESTURE;

    char output[128];
    int len;

    struct bmi2_sens_data sensor_data_temp = {0};

    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi);
    //bmi2_error_codes_print_result(rslt);

    IntervalSetting intSet = default_interval_setting;

    if (rslt == BMI2_OK)
    {
        /* Enable the selected sensors. */
        //bmi.aps_status = BMI2_ENABLE;
        set_accel_gyro_config(&bmi);
        setup_interrupt_pin(&bmi);
        setup_features(&bmi);
        bmi270_sensor_enable(sens_list, SENS_NUM, &bmi);
        bmi270_map_feat_int(feat_int_map, NUM_FEAT, &bmi);
        stopWatchModeInit();

        //interval_setting_mode = 1;
        do {
            //printf("LPM\n");
            //Renable interrupts
            GPIO_clearInterrupt(INT_PORT, INT2_PIN);
            GPIO_enableInterrupt(INT_PORT, INT2_PIN);

            GPIO_clearInterrupt(INT_PORT, INT1_PIN);
            GPIO_enableInterrupt(INT_PORT, INT1_PIN);

            //displayText(current_state);
            //printf(current_state);
            displayTime();

            if(interval_setting_mode == 1){
                RTC_C_startClock(RTC_C_BASE);
            }

            // LPM Sleep
            __bis_SR_register(LPM0_bits + GIE);
            
            if(interval_setting_mode == 1){
                if(centisecond % (5048) == 0){
                    x++;
                }
                if(x == 5){
                    x = 0;
                    indx = 0;
                    #define INT_SET_FRAMES 5
                    Frame f[INT_SET_FRAMES];
                    while (indx < NO_MOTION_SNAPSHOT_LEN)
                    {
                        rslt = bmi2_get_sensor_data(&sensor_data_temp, &bmi);               
                        if ((rslt == BMI2_OK) && (sensor_data_temp.status & BMI2_DRDY_ACC) &&
                            (sensor_data_temp.status & BMI2_DRDY_GYR))
                        {   
                            f[indx] = (struct Frame){
                                .acc = (struct Vec3d){
                                    .x = (double)sensor_data_temp.acc.x,
                                    .y = (double)sensor_data_temp.acc.y,
                                    .z = (double)sensor_data_temp.acc.z
                                },
                                .gyr = (struct Vec3d){
                                    .x = (double)sensor_data_temp.gyr.x,
                                    .y = (double)sensor_data_temp.gyr.y,
                                    .z = (double)sensor_data_temp.gyr.z
                                }
                            };

                            indx++;
                        }
                    }

                    IntervalSetting result = classify_interval_setting(f, INT_SET_FRAMES, interval_vector_min_similarity);
                    if(result.interval_mins != -1){
                        interval_setting_mode = 0;
                        intSet = result;
                        stopWatchModeInit();
                        printf("switched to interval: %d, exiting interval setting mode\n", result.interval_mins);
                    }

                    #ifdef CLASSIFY
                    ActionType at = tree_classify(f);
                    switch (at) {
                        case STAND: 
                            printf("STAND");
                            //RTC_C_holdClock(RTC_C_BASE);
                            break;
                        case SIT: 
                            printf("SIT");
                            //RTC_C_startClock(RTC_C_BASE);
                            break;
                        case REST:
                            printf("REST");
                            break;
                        case OTHER: 
                            printf("OTHER");
                            break;
                    }
                    printf("\n");
                    #endif
                
                    #ifdef PRINT_CAPTURE
                    indx = 0;
                    printf("%d,", y);
                    while (indx < INT_SET_FRAMES)
                    {
                        if(indx == INT_SET_FRAMES - 1){
                            
                        printf("%d,%d,%d,%d,%d,%d",
                        f[indx].acc.x,f[indx].acc.y,f[indx].acc.z,
                        f[indx].gyr.x,f[indx].gyr.y,f[indx].gyr.z);
                        } else {
                            
                        printf("%d,%d,%d,%d,%d,%d,",
                        f[indx].acc.x,f[indx].acc.y,f[indx].acc.z,
                        f[indx].gyr.x,f[indx].gyr.y,f[indx].gyr.z);
                        }
                        indx++;
                    }
                    printf("\n");
                    y++;
                    #endif
                }
            } else {
                //notify user
            
                if(int2_status == 1){
                    int2_status = 0;
                    bmi2_get_int_status(&int2_status_result, &bmi);
                    //printf("int2_status_result: %x\n",int2_status_result);

                    if(int2_status_result & BMI270_WRIST_GEST_STATUS_MASK){
                        bmi270_get_feature_data(&sens_data, 1, &bmi);
                        if (sens_data.sens_data.wrist_gesture_output == 3){
                            //shake -> enter interval setting mode
                            interval_setting_mode = 1;
                            printf("interval setting mode on \n");
                        }
                        //printf("Wrist gesture = %d\r\n", sens_data.sens_data.wrist_gesture_output);
                        //printf("Gesture output = %s\n", gesture_output[sens_data.sens_data.wrist_gesture_output]);
                    }

                    // /* To check the interrupt status of any-motion. */
                    if (int2_status_result & BMI270_ANY_MOT_STATUS_MASK){
                    }

                    int2_status_result = 0;
                }

                if(int1_status == 1){
                    int1_status = 0;
                    bmi2_get_int_status(&int1_status_result, &bmi);
                    /* To check the interrupt status of no-motion. */
                    if (int1_status_result & BMI270_NO_MOT_STATUS_MASK){
                        //printf("no-motion interrupt is generated\n");
                        indx = 0;
                        Frame f[5];
                        while (indx < NO_MOTION_SNAPSHOT_LEN)
                        {
                            rslt = bmi2_get_sensor_data(&sensor_data_temp, &bmi);               
                            if ((rslt == BMI2_OK) && (sensor_data_temp.status & BMI2_DRDY_ACC) &&
                                (sensor_data_temp.status & BMI2_DRDY_GYR))
                            {   
                                f[indx] = (struct Frame){
                                    .acc = (struct Vec3d){
                                        .x = sensor_data_temp.acc.x,
                                        .y = sensor_data_temp.acc.y,
                                        .z = sensor_data_temp.acc.z
                                    },
                                    .gyr = (struct Vec3d){
                                        .x = sensor_data_temp.gyr.x,
                                        .y = sensor_data_temp.gyr.y,
                                        .z = sensor_data_temp.gyr.z
                                    }
                                };
    
                                indx++;
                            }
                        }

                        #ifdef ACTIONS_BUFFER_ON
                        add_to_actiontype_circular_buffer(&recorded_actions, sample);

                        printf(sample == STAND ? "STAND " : sample == SIT ? "SIT " : "OTHER ");

                        for (i = 0; i < actions_buffer_size; i++) {
                            ActionType a = recorded_actions.data[(recorded_actions.tail + i) % recorded_actions.size];
                            printf(a == STAND ? "STAND " : a == SIT ? "SIT " : "OTHER ");
                        }
                        printf("\n");

                        uint8_t n_stands = 0;
                        uint8_t n_sits = 0;
                        uint8_t n_others = 0;
                        for (i = 0; i < actions_buffer_size; i++) {
                            ActionType a = recorded_actions.data[(recorded_actions.tail + i) % recorded_actions.size];
                            switch (a) {
                                case SIT: n_sits++; break;
                                case STAND: n_stands++; break;
                                case OTHER: n_others++; break;
                                default: break;
                            }
                        }

                        if (n_stands >= 1) {
                            sample = STAND;
                        } else {
                            sample = n_stands > n_sits ? (n_stands > n_others ? STAND : OTHER) : (n_sits > n_others ? SIT : OTHER);
                        }
                        #endif
                        
                        indx = 0;
                        switch (tree_classify(f)) {
                            case STAND:
                                stopWatchModeInit();
                                break;
                            case SIT: 
                                RTC_C_startClock(RTC_C_BASE);
                                break;
                            case OTHER: 
                                break;
                        }
                    }



                    // if (int1_status_result & BMI270_STEP_ACT_STATUS_MASK){
                    //     /* Get step activity output. */
                    //     rslt = bmi270_get_feature_data(&sensor_activity_data, 1, &bmi);

                    //     /* Print the step activity output. */
                    //     printf("Step activity = %s\n", activity_output[sensor_activity_data.sens_data.activity_output]);

                    //     if(sensor_activity_data.sens_data.activity_output == 0) {
                    //         indx = 0;
                    //         while (indx < 1)
                    //         {
                    //             rslt = bmi2_get_sensor_data(&sensor_data_temp, &bmi);               
                    //                 if ((rslt == BMI2_OK) && (sensor_data_temp.status & BMI2_DRDY_ACC) &&
                    //                     (sensor_data_temp.status & BMI2_DRDY_GYR))
                    //                 {   
                    //                     sensor_data_snapshot.acc = (struct BMI2SensAxisData){
                    //                         .x = sensor_data_temp.acc.x,
                    //                         .y = sensor_data_temp.acc.y,
                    //                         .z = sensor_data_temp.acc.z};
                    //                     sensor_data_snapshot.gyr = (struct BMI2SensAxisData){
                    //                         .x = sensor_data_temp.gyr.x,
                    //                         .y = sensor_data_temp.gyr.y,
                    //                         .z = sensor_data_temp.gyr.z};
                    //                     indx++;
                    //                 }
                    //         }
                    //     }

                    //     // if(sensor_activity_data.sens_data.activity_output == 1 || sensor_activity_data.sens_data.activity_output == 2) {
                    //     //     current_state = "STAND ";
                    //     // }

                    // }
                    int1_status_result = 0;
                }
            }

        } while (rslt == BMI2_OK);
    }
}


/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];
    #define ACCEL 0
    #define GYRO 1
    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

/*
 * PORT1 Interrupt Service Routine
 * Handles S1 and S2 button press interrupts
 */
// #pragma vector = PORT1_VECTOR
// __interrupt void PORT1_ISR(void) {
//     switch(__even_in_range(P1IV, P1IV_P1IFG7)){
        
//         case P1IV_NONE : break;
//         case P1IV_P1IFG0 : break;
//         case P1IV_P1IFG1 :    // Button S1 pressed
//             P9OUT &= ~BIT7;
//             break;
//         case P1IV_P1IFG2 :    // Button S2 pressed
//             P9OUT |= BIT7;
//             break;
//         case P1IV_P1IFG3 : break;
//         case P1IV_P1IFG4 : break;
//         case P1IV_P1IFG5 : break;
//         case P1IV_P1IFG6 : break;
//         case P1IV_P1IFG7 : break;
//     }
// }

/*
 * RTC Interrupt Service Routine
 * Wakes up every ~10 milliseconds to update stowatch
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_ISR(void)
{
    switch(__even_in_range(RTCIV, 16))
    {
    case RTCIV_NONE: break;      //No interrupts
    case RTCIV_RTCOFIFG: break;      //RTCOFIFG
    case RTCIV_RTCRDYIFG:             //RTCRDYIFG
        counter = RTCPS;
        centisecond = 0;
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case RTCIV_RTCTEVIFG:             //RTCEVIFG
        //Interrupts every minute
        __no_operation();
        break;
    case RTCIV_RTCAIFG:             //RTCAIFG
         __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
        //__no_operation();
        break;
    case RTCIV_RT0PSIFG:
        centisecond = RTCPS - counter;
        __bic_SR_register_on_exit(LPM3_bits);
        break;     //RT0PSIFG
    case RTCIV_RT1PSIFG:
        __bic_SR_register_on_exit(LPM3_bits);
        break;     //RT1PSIFG

    default: break;
    }
}

/*
 * PORT2 Interrupt Service Routine
 */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void) {
    switch(__even_in_range(P2IV, P2IV_P2IFG7)){
        
        case P2IV_NONE : break;
        case P2IV_P2IFG0 : break;
        case P2IV_P2IFG1 : break;
        case P2IV_P2IFG2 : break;
        case P2IV_P2IFG3 : break;
        case P2IV_P2IFG4 :
            //disable interrupt 
            P2IE &= ~BIT4;
            //toggle led
            P9OUT ^= BIT7;
            int1_status = 1;
            //bmi2_get_int_status(&int1_status_result, &bmi);
            __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
            break;
        case P2IV_P2IFG5 : 
            //disable interrupt 
            P2IE &= ~BIT5;
            //toggle led
            P1OUT ^= BIT0;
            int2_status = 1;
            //bmi2_get_int_status(&int2_status_result, &bmi);
            __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
            break;
        case P2IV_P2IFG6 : break;
        case P2IV_P2IFG7 : break;
    }
}
