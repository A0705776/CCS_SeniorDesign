/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"


/*
 * Author - Charles Asquith, Pavani Deepthi Tenneti and Shubhangi Mundhada
 * This program enables the TM4C123 launchpad to use several peripherals
 * to follow the wall and ultimately solve the maze.
 * The peripherals used for this project are as follows:
 *      UART
 *      ADC
 *      TIMER
 *      GPIO
 */

/*
 * Include statements
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.c"
#include "driverlib/adc.c"
#include "driverlib/uart.c"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.c"
#include "driverlib/pwm.c"
#include "inc/hw_ints.h"
#include "driverlib/timer.c"
#include "driverlib/udma.h"
#include "inc/hw_udma.h"
#include "inc/hw_uart.h"

/*
 * Define statements
 */
#define PWM_FREQUENCY       10000
#define PWM_ADJUST          83
#define ADC_MAX_VALUE       4096
#define TARGET_VALUE        (ADC_MAX_VALUE / 2)
#define SEQUENCE_ONE        1
#define SEQUENCE_TWO        2
#define SEQUENCE_THREE      3
#define SEQUENCE_FOUR       4
#define PRIORITY_ZERO       0
#define PRIORITY_ONE        1
#define STEP_ZERO           0
#define BUFFER_SIZE         20


/*
 * Global declarations
 */

//ADC values
uint32_t rightSensorValue = 0;
uint32_t frontSensorValue = 0;

//PWM values
volatile uint32_t PWM_CLOCK, PWM_LOAD;
volatile uint32_t TWO_SECONDS = 80000000;
volatile uint32_t PWM_PERIOD = 20;

// Variables for PID implementation
volatile float proportionalRight;
volatile float lastProportionalRight = 250;
volatile float integralRight = 0;
volatile float derivativeRight;
volatile float pidRight;

// Variables for Ping pong buffers
volatile int buffer[BUFFER_SIZE];
volatile int buffer_2[BUFFER_SIZE];
volatile int temp_buffer[BUFFER_SIZE];
volatile int error = 0;
int i = 0;
int j = 0;
int count = 0;
int count_2 = 0;
int swap = 3;

char command[2] = "  ";

/*
 * Function definitions
 */
void ConfigureUART(void);
void ConfigurePWM(void);
void ConfigureGPIOOutput(void);
void ConfigureADC(void);
void PID(int RightValue, int FrontValue);
void ConfigureTimer(void);
void TimerInit(void);
void printingBuffer(int[]);
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void ConfigurePeripherals(void);

/*
 * In this function, we are enabling the UART peripheral
 * We first enable GPIO port B and UART1. UART1 is the terminal that connects to the bluetooth
 *
 * In port B, we configure pins 0 and 1 as the UART1 RX and TX pins respectively.
 * We also configure the UART1 clock and set the
 *      module: 1
 *      Baud rate: 115200
 *      Clock speed: 16000000/ 16[MHz]
 */
void ConfigureUART(void)
{
    /* Enable the clocks to PortB and UART1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    /* Configure PortB pins 0 & 1 for UART1 RX & TX, respectively */
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Set the UART1 module's clock source */
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE));

    /* Configure UART1 */
    // UART module: 1
    // Baud rate: 115200
    // UART clock speed: 16 [MHz]
    UARTStdioConfig(1, 115200, 16000000);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_TX);
}

/*
 * In this function, we enable the Pulse Width Modulation (PWM) peripheral
 *
 * We initially set the PWM clock to (16[MHz]/64).
 * We enable PWM1 module along with GPIO port D.
 *
 * We configure pins 0 and 1 of Port D as the PWM module 1, generator 0 enable pins.
 * The PWM clock speed and load values are calculated which will be used to set the duty cycle.
 *
 * The PWM generator is configured as a count down mode with the dead band disabled.
 *
 * The PWM output state is set to true, which enables the motors.
 */
void ConfigurePWM(void)
{
    /* Set the PWM module's clock divider */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    /* Enable the clock for PWM1 and PortD */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    /* Configure PortD pins 0 & 1 for PWM module 1, generator 0 usage */
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);

    /* Calculate the PWM clock and load values */
    PWM_CLOCK = SysCtlClockGet() / 64;
    PWM_LOAD = (PWM_CLOCK / PWM_FREQUENCY) - 1;

    /* Configure M1PWM0 for count down mode */
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);

    /* Set the period of the PWM generator */
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, PWM_LOAD);

    /* Specify the duty cycle for the PWM signal */
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWM_ADJUST * PWM_LOAD / 100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWM_ADJUST * PWM_LOAD / 100);

    /* Enable PWM output */
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);

    /* Enable the timer/counter for M1PWM0 */
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

/*
 * In this function, the GPIO Ports and pins are configured.
 * Port F is used to enable the LED on the launchpad as well as control the reflectance sensor.
 *
 * Pins 2 and 3 of Port B are configured as the phase pins on the motor driver which are utilized to set the direction of the motors.
 */
void ConfigureGPIOOutput(void)
{
    /* Enable the clock for PortF and PortB */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    /* Configure PortF pins 1&2 and PortB pins 2&3 for GPIO output */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

/*
 * This function is used to configure the ADC peripheral.
 * The output of the sensor is connected to pins 2 and 3 of Port E.
 *
 * In order to use two ADC sensors at the same time, the sequences were disabled.
 * The requires sequences (SEQUENCE_ONE and SEQUENCE_TWO) were utilized, so the
 * ADC was re-enabled utilizing these sequences with different priorities.
 *
 */
void ConfigureADC(void) {

    /* Enable the clock for ADC0 and PortE */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    /* Configure PortE pins 2 & 3 for ADC usage */
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    /* Disable sequencers to ensure safe reconfiguration of them */
    ADCSequenceDisable(ADC0_BASE, SEQUENCE_ONE);
    ADCSequenceDisable(ADC0_BASE, SEQUENCE_TWO);
    ADCSequenceDisable(ADC0_BASE, SEQUENCE_THREE);
    ADCSequenceDisable(ADC0_BASE, SEQUENCE_FOUR);

    /* Configure sequence priorities and triggers */
    // SS0: unused
    // SS1 - to sample right sensor: trigger = timer, priority = 0
    // SS2 - to sample front sensor: trigger = timer, priority = 1
    // SS3: unused
    ADCSequenceConfigure(ADC0_BASE, SEQUENCE_ONE, ADC_TRIGGER_PROCESSOR,
    PRIORITY_ZERO);
    ADCSequenceConfigure(ADC0_BASE, SEQUENCE_TWO, ADC_TRIGGER_PROCESSOR,
    PRIORITY_ONE);

    /* Configuring sequence steps for sequence 1 */
    // Step 0: sample right sensor, end of sequence
    ADCSequenceStepConfigure(ADC0_BASE, SEQUENCE_ONE, STEP_ZERO,
            (ADC_CTL_CH0 | ADC_CTL_END));

    /* Configuring sequence steps for sequence 2 */
    // Step 0: sample front sensor, end of sequence
    ADCSequenceStepConfigure(ADC0_BASE, SEQUENCE_TWO, STEP_ZERO,
            (ADC_CTL_CH1 | ADC_CTL_END));

    /* Re-enable our now newly configured sequences */
    ADCSequenceEnable(ADC0_BASE, SEQUENCE_ONE);
    ADCSequenceEnable(ADC0_BASE, SEQUENCE_TWO);

}

/*
 * The Timer1IntHandler triggers the ADC0_Base to acquire the ADC samples from the
 * front and right sensors every 50[ms].
 * These values are sent to a PID function to ensure the robot follows the wall using a PID controller.
 */
void Timer1IntHandler(void) {

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* Trigger the sample sequence 1 */
    ADCProcessorTrigger(ADC0_BASE, SEQUENCE_ONE);

    /* Get results from sample sequence 1 */
    ADCSequenceDataGet(ADC0_BASE, SEQUENCE_ONE, &rightSensorValue);

    /* Trigger sample sequence 2 */
    ADCProcessorTrigger(ADC0_BASE, SEQUENCE_TWO);

    /* Get results from sample sequence 2 */
    ADCSequenceDataGet(ADC0_BASE, SEQUENCE_TWO, &frontSensorValue);

    /* Perform PID on the sensor values */
    PID(rightSensorValue, frontSensorValue);

}

/*
 *  The Timer0IntHandler, which has the highest priority, will print the error values on to tera term.
 *  A variable "swap" ensures that ping-pong mechanism is followed. buffer[] and buffer_2[] prints out
 *  alternate error values.
 *
 *  This interrupt is enabled every 2 seconds.
 *
 *  The UARTBusy function is 0 or NULL, it implies that values are being printed onto tera term while the other
 *  buffer is being filled.
 */
void Timer0IntHandler(void) {

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if (swap == 0) {

        for (j = 0; j < BUFFER_SIZE; ++j) {

            UARTprintf("%d\n\r", buffer[j]);

            if(UARTBusy(UART1_BASE)) {

                buffer_2[j] = temp_buffer[j];

            }

        }

        swap = 1;

    }

    else if (swap == 1) {

        for (j = 0; j < BUFFER_SIZE; ++j) {

            UARTprintf("%d\n\r", buffer_2[j]);

            if(UARTBusy(UART1_BASE)) {

                buffer[j] = temp_buffer[j];

            }

        }

        swap = 0;

    }

    i = 0;

}

/*
 *  This function essentially follows a PID controller. Gain values for the proportional, integral and
 *  derivative terms are selected to tune the robot to follow the wall.
 *
 *  Initially, in the function, the reflectance sensor reads the value of the surface underneath. If a low value is read by the GPIO pins, then the reflactance sensors are over a black or dark colored background.
 *  We update the value of "count" to differentiate between a 1/4" black line and a 1/2" black line.
 *  If a 1/2" black line is detected, it disables the PWM module and enters a while(1) loop.
 *
 *  The proportional gain = 1/20
 *  The integral gain = 1/10000
 *  The differential gain = 3/2
 *
 *  The target value chosen is 2000, and the (current ADC values - target value) is defined as the error values which will be stored in a buffer.
 *  The ranges determine how close it is to the wall and what actions must be taken to prevent the robot from crashing into the wall.
 *
 *  Finally, the error values are stored in the buffer which will be used in the Timer0IntHandler()
 */
void PID( RightValue, FrontValue) {

    count = 0;
    while (!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0x02)) {

        swap = 0;
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 12);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 4);

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 15 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 15 * PWM_LOAD / 100);

        ++count;

        if (count > 900) {

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);

            swap = 3;

            while (1) {

            }

        }

        UARTprintf("\t\b\b\b\b\b\b\b");

    }

    count = 0;

    //UARTprintf ("The error value: %d\n\r", error);
    proportionalRight = (RightValue - TARGET_VALUE) / 20;

    /* Calculate integral terms */
    integralRight = (RightValue - TARGET_VALUE);

    /* Calculate derivative terms */
    derivativeRight = ((RightValue - TARGET_VALUE) - lastProportionalRight)
            * (3 / 2);

    /* Calculate PID result */
    pidRight = proportionalRight + (integralRight / 10000) + derivativeRight;

    /* Update some values for proper calculations of the next PID update*/
    lastProportionalRight = (RightValue - TARGET_VALUE);

    if ((pidRight < 25) && (FrontValue > 2000)) // Turn left and check if dead end
            {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 12);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 12);

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 99 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 99 * PWM_LOAD / 100);

    } else if (pidRight > 27 && FrontValue < 1500)  // Turn Left
            {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 12);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 4);

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 99 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 15 * PWM_LOAD / 100);
    } else if (pidRight < -25 && FrontValue < 1500) // Turn Right
            {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 12);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 4);

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 15 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 99 * PWM_LOAD / 100);
    } else if ((pidRight > -25 && pidRight < 25) && FrontValue < 1500)// Go straight
            {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 12);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 4);

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 99 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 99 * PWM_LOAD / 100);
    }

    error = TARGET_VALUE - RightValue;

        if (i < BUFFER_SIZE) {

            temp_buffer[i] = error;

        }

        ++i;

}

/*
 *  This function enables and configures the timer peripheral.
 *  The timer is configured as PERIODIC_UP.
 *  We utilize two timers. One to sample ADC values every 50[ms] and the other to print values 2[s].
 */
void ConfigureTimer(void) {

    /* Enable the clock to timer 0 and timer1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    /* Configure the timer */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC_UP);

}

/*
 *  The TIMER0 has a load value of TWO_SECONDS and TIMER_1 has a load value (PWM-PERIOD - 1)
 *  The interrupts for the timer are enabled in this function.
 */
void TimerInit(void) {

    /* Set the timer load value */
    TimerLoadSet(TIMER0_BASE, TIMER_A, TWO_SECONDS);
    TimerLoadSet(TIMER1_BASE, TIMER_A, PWM_PERIOD - 1);

    /* Enable interrupts for timer A */
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);

    /* Enables interrupts for timer A timeout event */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable timer A */
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

/*
 *  This function calls the various functions that enable the peripherals used in this project.
 *  We also set the System's clock to 40[MHz].
 *  The peripherals enabled using this function are:
 *          UART
 *          PWM
 *          GPIO PINS
 *          ADC
 *          TIMER
 */
void ConfigurePeripherals(void) {

    FPULazyStackingEnable();

    /* Set system clock */
    SysCtlClockSet(
    SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    ConfigureUART();
    ConfigurePWM();
    ConfigureGPIOOutput();
    ConfigureADC();
    ConfigureTimer();

}



/*
 *  ======== main ========
 */
int main(void)
{

    /* Configure peripherals */
       ConfigurePeripherals();

       /* Enable global interrupts */
       IntMasterEnable();

       PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
       PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false);

       UARTprintf(
               "Team 5:\n\rPavani Tenneti\n\rCharles Asquith\n\rShubangi Mundhada\n\r");

       UARTgets(command, strlen(command) + 1);
       UARTprintf("\n");
       if (!strcmp(command, "GO"))    // Start
               {
           UARTprintf("Starting motors...\n\r");
           PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
           PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
       }

       /* Initialize timer */
       TimerInit();

       while (true) {

           /*
            do nothing*/

       }


    /* Start BIOS */
    BIOS_start();

    return (0);
}
