#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "stdio.h"
#include "tmp007.h"
#include "opt3001.h"
#include "i2c_driver.h"
#include "demo_sysctl.h"
#include "uart_driver.h"
#include "bmi160_support.h"
#include "bme280_support.h"
#include <ti/devices/msp432p4xx/inc/msp432.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

//Definitions
#define USING_BOSCH_BP
#define CPU_FREQ            (48000000)
#define SAMPLE_TIME_1       (53)
#define SAMPLE_TIME_2       (26)
#define SAMPLE_TIME_4       (13)
#define SAMPLE_TIME_6       (8)
#define SAMPLE_TIME_8       (6)
#define SAMPLE_TIME_10      (5)
#define NUM_AVGR_SUMS       (2) //x**2 frames

//Function Prototypes
void simpleDelay();
void stopWakeUpTimerA(void);
void startCrystalOscillator(void);
void setSystemClock(uint32_t CPU_Frequency);
void startWakeUpTimerA(uint16_t ulClockMS);
int32_t movingAvg(int prevAvg, int16_t newValue);

//Global Data
const uint8_t wdtWakeUpPeriod [8] = {
        WDT_A_CLOCKDIVIDER_2G,
        WDT_A_CLOCKDIVIDER_128M,
        WDT_A_CLOCKDIVIDER_8192K,
        WDT_A_CLOCKDIVIDER_512K,
        WDT_A_CLOCKDIVIDER_32K,
        WDT_A_CLOCKDIVIDER_8192,
        WDT_A_CLOCKDIVIDER_512,
        WDT_A_CLOCKDIVIDER_64,
};

const uint8_t timeSamplesBMI [6] = {
        SAMPLE_TIME_1,      //Sample at 1 time per second
        SAMPLE_TIME_2,      //Sample at 2 times per second
        SAMPLE_TIME_4,      //Sample at 4 times per second
        SAMPLE_TIME_6,      //Sample at 6 times per second
        SAMPLE_TIME_8,      //Sample at 8 times per second
        SAMPLE_TIME_10,     //Sample at 10 times per second
};

//Default time sample values for each sensor
volatile uint8_t sampleTimePeriodBMI = 2;

// Changed by the GUI -  default ~ 0.0156 seconds  1/32KHz * WDT_A_CLOCKDIVIDER_512
volatile uint8_t wdtWakeUpPeriodIndex = 6;

// BMI160/BMM150
BMI160_RETURN_FUNCTION_TYPE returnValue;
struct bmi160_gyro_t        s_gyroXYZ;
struct bmi160_mag_xyz_s32_t s_magcompXYZ;

//Timer Counter
uint16_t WDTcount = 0;

//Receive UART Variables
#define NUM_RX_CHARS 64
int numChars = 0;
int numMsgsRx = 0;
int tempIndex = 5;
char rxMsgData[NUM_RX_CHARS] = "";

//Calibration off-sets
s16 gyro_off_x;
s16 gyro_off_y;
s16 gyro_off_z;

//gesture recognition
uint16_t gyroAbsX, gyroAbsY, gyroAbsZ;
int16_t prevGyroX = 0;
int16_t prevGyroY = 0;
int16_t prevGyroZ = 0;
int32_t gyroAvgX = 0.0;
int32_t gyroAvgY = 0.0;
int32_t gyroAvgZ = 0.0;

//Sensor Status Variables
bool BMI_on = true;

int main(void)  {
    volatile uint32_t index;

    //Stop WDT and disabling master interrupts
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);

    //Enabling SRAM Bank Retention
    SYSCTL->SRAM_BANKRET |= SYSCTL_SRAM_BANKEN_BNK7_EN;
        for (index = 0;index < 100;index++);

    #ifdef USE_LPM
        //Turn off PSS high-side & low-side supervisors to minimize power consumption
        MAP_PSS_disableLowSide();
        MAP_PSS_disableHighSide();
    #endif

        //Configure clocks
        startCrystalOscillator();
        setSystemClock(CPU_FREQ);

    #ifdef USING_BOSCH_BP
        //Initialize uart
        uartInit();

        //Initialize i2c
        initI2C();

    #endif

        MAP_Interrupt_enableMaster();

    #ifdef USING_BOSCH_BP
        //Initialize bmi160 sensor
        bmi160_initialize_sensor();
        bmi160_set_foc_gyro_enable(0x01, &gyro_off_x, &gyro_off_y, &gyro_off_z);
    #endif

    // Using the wdt as interval timer to wake up from LPM3
    while(1) {
        // Stop WDT
        MAP_WDT_A_holdTimer();
        MAP_WDT_A_clearTimer();
        MAP_Interrupt_disableInterrupt(INT_WDT_A);
        //Set frequency back to full speed if reading/converting data
        if(WDTcount%timeSamplesBMI[sampleTimePeriodBMI] == 0) {
            if(1) {
                gyroAbsX = abs(s_gyroXYZ.x);
                gyroAbsY = abs(s_gyroXYZ.y);
                gyroAbsZ = abs(s_gyroXYZ.z);
                gyroAvgX = movingAvg(gyroAvgX, gyroAbsX);
                gyroAvgY = movingAvg(gyroAvgY, gyroAbsY);
                gyroAvgZ = movingAvg(gyroAvgZ, gyroAbsZ);
                if(gyroAvgX > 1000 || gyroAvgY > 1000 || gyroAvgZ > 1000) {
                    if(gyroAvgY > gyroAvgX) {
                        if(gyroAvgY > gyroAvgZ) {
                            if(s_gyroXYZ.y > 300) {
                                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN2);
                                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
                                simpleDelay();
                            }
                            else if(s_gyroXYZ.y < -1300) {
                                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
                                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 + GPIO_PIN0);
                                simpleDelay();
                            }
                        } else {
                            //z-axis
                            if(s_gyroXYZ.z > 900) {
                                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
                                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 + GPIO_PIN2);
                                simpleDelay();
                            }
                            else if(s_gyroXYZ.z < -700) {
                                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
                                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2 + GPIO_PIN0);
                                simpleDelay();
                            }
                        }
                    } else if( gyroAvgZ > gyroAvgX ) {
                        if(s_gyroXYZ.z > 0) {
                            //z-axis
                            if(s_gyroXYZ.z > 900) {
                                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
                                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 + GPIO_PIN2);
                                simpleDelay();
                            } else if(s_gyroXYZ.z < -700) {
                                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
                                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2 + GPIO_PIN0);
                                simpleDelay();
                            }
                        }
                    } else {
                        //x-axis
                        if(s_gyroXYZ.x > 900) {
                            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 + GPIO_PIN2);
                            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
                            simpleDelay();
                        } else if(s_gyroXYZ.x < -700)
                        {
                            GPIO_setOutputLowOnPin(GPIO_PORT_P2, 0x0003);
                            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
                            simpleDelay();
                        }
                    }
                }
                else
                {
                    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
                }
            }
        }

        //Reset WDTcount when max count has been reached
        if(WDTcount == 53) {
            //Reset WDT counter after reaching roughly one second
            WDTcount = 0;
        }
        WDTcount++;

        // Configure WDT
        // For LPM3 Clock Source should be BCLK or VLOCLK
        MAP_WDT_A_initIntervalTimer(WDT_A_CTL_SSEL_3/*WDT_A_CLOCKSOURCE_ACLK*/,
                wdtWakeUpPeriod[wdtWakeUpPeriodIndex]);
        MAP_Interrupt_enableInterrupt(INT_WDT_A);
        
        // Start WDT
        MAP_WDT_A_startTimer();
        
        //Go to LPM0 (Cannot use LPM3 because we won't accurately receive UART data)
        MAP_PCM_gotoLPM0();
    }
    
/*
Funtion: 

    Works as a simple moving averager. Used for gesture recognition.

*/

int32_t movingAvg(int prevAvg, int16_t newValue) {
    return (((prevAvg << NUM_AVGR_SUMS) + newValue - prevAvg) >> NUM_AVGR_SUMS);
}

/*
Function:

   The following function is responsible for starting XT1 in the
   MSP432 that is used to source the internal FLL that drives the
   MCLK and SMCLK.

*/

void startCrystalOscillator(void) {

    /* Configuring pins for peripheral/crystal HFXT*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring pins for peripheral/crystal LFXT*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
}

/*
Function:
    The following function is responsible for setting up the system clock at a specified frequency.

*/

void setSystemClock(uint32_t CPU_Frequency) {
    /* 

    Setting the external clock frequency. This API is optional, but will
    come in handy if the user ever wants to use the getMCLK/getACLK/etc
    functions.

    */
    MAP_CS_setExternalClockSourceFrequency(32768, CPU_Frequency);
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_32KHZ);

    //Before we start we have to change VCORE to 1 to support the 24MHz frequency
    MAP_PCM_setCoreVoltageLevel(PCM_AM_LDO_VCORE0);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);

    //Starting HFXT and LFXT in non-bypass mode without a timeout. 
    MAP_CS_startHFXT(false);
    MAP_CS_startLFXT(false);

    /* 
    Initializing the clock source as follows:
        MCLK = HFXT/2 = 24MHz
        ACLK = LFXT = 32KHz
        HSMCLK = HFXT/4 = 6MHz
        SMCLK = HFXT/2 = 12MHz
        BCLK  = REFO = 32kHz
    */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);
    MAP_CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    MAP_CS_initClockSignal(CS_BCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

void configureGPIO(void) {
    //Configure I/O to minimize power consumption before going to sleep
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P7, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P10, PIN_ALL8);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3);
    
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, PIN_ALL8);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, PIN_ALL8);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, (PIN_ALL8 & ~GPIO_PIN6));
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, (PIN_ALL8 & ~(GPIO_PIN1 | GPIO_PIN6)));
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, (PIN_ALL8 & ~(GPIO_PIN0 | GPIO_PIN2)));
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, (PIN_ALL8 & ~GPIO_PIN7));
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, PIN_ALL8);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, PIN_ALL8);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, PIN_ALL8);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P10, PIN_ALL8);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3);
}

void startWakeUpTimerA(uint16_t ulClockMS) {
    ulClockMS = (ulClockMS * 32768)/1000;

    //TimerA UpMode Configuration Parameter 
    Timer_A_UpModeConfig upConfig = {
            TIMER_A_CLOCKSOURCE_ACLK,               // ACLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_1,          // ACLK/1 = 32KHz
            ulClockMS,                              // tick period
            TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,     // Enable CCR0 interrupt
            TIMER_A_SKIP_CLEAR                      // Clear value
    };

    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_0);

    MAP_Interrupt_enableInterrupt(INT_TA0_0);
    MAP_Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
}

void stopWakeUpTimerA(void) {
    MAP_Interrupt_disableInterrupt(INT_TA0_0);
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
}

/*

Function: 
    TA0_0_IRQHandler

*/
void TA0_0_IRQHandler(void) {
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_0);

    #ifdef USE_LPM
        MAP_Interrupt_disableSleepOnIsrExit();
    #endif
}

/*

Function: 
    WDT_A_IRQHandler

*/
void WDT_A_IRQHandler(void) {
    //MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    //Waking up from LMP3 take us to PCM_AM_LDO_VCORE0 instead of PCM_AM_LF_VCORE0
    //MAP_PCM_setPowerState(PCM_AM_LDO_VCORE0);
    //MAP_PCM_setCoreVoltageLevel(PCM_AM_DCDC_VCORE0);

    #ifdef USE_LPM
        MAP_Interrupt_disableSleepOnIsrExit();
    #endif
}

/*

Function: 
    PORT1_IRQHandler

*/

void PORT1_IRQHandler(void) {
    uint32_t debounce;
    uint32_t status;

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);

    if(status & GPIO_PIN1) {}

    //Delay for switch debounce
    for(debounce = 0; debounce < 10000; debounce++) {
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
    }

    #ifdef USE_LPM
        MAP_Interrupt_disableSleepOnIsrExit();
    #endif
}

/*

Function: 
    PORT5_IRQHandler

*/

void PORT5_IRQHandler(void) {
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);

    MAP_GPIO_disableInterrupt(GPIO_PORT_P5, GPIO_PIN2);
    MAP_Interrupt_disableInterrupt(INT_PORT5);

    if(status & GPIO_PIN2) {}

    //Delay for switch debounce 
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

    #ifdef USE_LPM
        MAP_Interrupt_disableSleepOnIsrExit();
    #endif
}

/*

Function: 
    _system_pre_init

*/
int _system_pre_init(void) {
    // Hold watchdog timer
    MAP_WDT_A_holdTimer();                        
    // Perform C/C++ global data initialization
    return 1;
}

void simpleDelay() {
    int i = 0;
    do {
        i++;
    } while(i < 500000);
}