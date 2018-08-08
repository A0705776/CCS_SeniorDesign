/*
 * TivaC_GPIO_Config.h
 *
 *  Created on: Mar 20, 2017
 *      Author: Alamin Ahmed
 */

#ifndef TIVAC_GPIO_CONFIG_H_
#define TIVAC_GPIO_CONFIG_H_

#define PWMPeriod       1024
#define PWMPulseWidth   0
#define MaxUARTLength   20
#define HIGH            true
#define LOW             false
#define INPUT           'I'
#define OUTPUT          'O'
#define ADC             'A'
#define PWM             'P'
#define UART            'U'

//For General GPIO Number assigned to actual Pin Number on the chip
enum{
    PF1 = 29,
    PB6 = 1,
    PF4 = 5,
    PE3 = 6,
    PE2 = 7,
    PE1 = 8,
    PE0 = 9,
    PD7 = 10,
    PC7 = 13,
    PC6 = 14,
    PC5 = 15,
    PC4 = 16,
    PA0 = 17,
    PA1 = 18,
    PA2 = 19,
    PA3 = 20,
    PA4 = 21,
    PA5 = 22,
    PA6 = 23,
    PA7 = 24,
    PF0 = 28,
    PB7 = 4,
    PF2 = 30,
    PF3 = 31,
    PD4 = 43,
    PD5 = 44,
    PB0 = 45,
    PB1 = 46,
    PB2 = 47,
    PB3 = 48,
    PC3 = 49,
    PC2 = 50,
    PC1 = 51,
    PC0 = 52,
    PD6 = 53,
    PB5 = 57,
    PB4 = 58,
    PE4 = 59,
    PE5 = 60,
    PD0 = 61,
    PD1 = 62,
    PD2 = 63,
    PD3 = 64
};
//For UART
enum{
    U0 = PA1,
    U1 = PB1,
    U2 = PD7,
    U3 = PC7,
    U4 = PC5,
    U5 = PE5,
    U6 = PD5,
    U7 = PE1,
    //************* Tx/Rx ****************
    U0T = PA1,
    U0R = PA0,
    U1T = PB1,
    U1R = PB0,
    U2T = PD7,
    U2R = PD6,
    U3T = PC7,
    U3R = PC6,
    U4T = PC5,
    U4R = PC4,
    U5T = PE5,
    U5R = PE4,
    U6T = PD5,
    U6R = PD4,
    U7T = PE1,
    U7R = PE0
};
//this make things works
void setPinsToWTF(void);

//set all the pins to output
void setPinsToDefault(void);

//Main Function, have to call 1st when initializing GPIO
void GPIOSet(uint8_t pin, char function);

//Write HIGH/LOW to the Output Pin
void GPIOWrite(uint8_t pin,_Bool state);

//Read the pin state
_Bool GPIORead(uint8_t pin);

void InterruptEnable(uint8_t pin);
void InterruptDisable(uint8_t pin);
void InterruptClear(uint8_t pin);
uint32_t InterruptStatus(uint8_t pin);

//used by GPIOSet function to initialize the inner setup of the pins
void GPIOPinInfo(uint8_t pin, uint32_t *ui32Peripheral, uint32_t *ui32Port, uint8_t *ui8Pins);

//used by GPIOSet function to initialize the PWM setup of the pins
void PWMPinInfo(uint8_t pin, uint32_t *ui32PinConfig, uint32_t *ui32Peripheral,uint32_t *PWMui32Base,uint32_t *PWMui32Gen,uint32_t *ui32PWMOut,uint32_t *ui32PWMOutBits);

//set the PWM Duty cycle (between 0 to PWMPeriod)
void setPWMDuty(uint8_t pin,uint32_t duty);

//turn on the PWM generator
void setPWMOn(uint8_t pin);

//turn off the PWM generator
void setPWMOff(uint8_t pin);

//read the ADC data (Processor Trigger)
uint32_t ADCRead(uint32_t pin);

//set up the pin to ADC generator Base 0
void ADC0Setup(uint8_t pin, uint32_t sequence);

//set up the pin to ADC generator Base 1
void ADC1Setup(uint8_t pin, uint32_t sequence);

//used by GPIOSet function and ADC1Setup/ADC0Setup to initialize the ADC
void ADCPinInfo(uint8_t pin, uint32_t *channel);

//used by GPIOSet function and UARTSetBaud function to  the UART
void UARTPinInfo(uint8_t pin, uint32_t *UARTui32Peripheral, uint32_t *UARTui32PinConfig, uint32_t *UARTui32Base);

//set the baud rate of the UART base
void UARTSetBaud(uint8_t pin, uint32_t BaudRate);

//enable the UART Interrupt
void UARTEnableInt(uint8_t pin);

//disable the UART Interrupt
void UARTDisableInt(uint8_t pin);

//clear the Interrupt Flag/s
void UARTClearInt(uint8_t pin);

//put char
void UARTPutChar(uint8_t pin, unsigned char data);

//get a Char from UART base
char UARTGetChar(uint8_t pin);

//write String to UART base
void UARTStringPut(uint8_t pin, char* porn);

//write String to UART base with new line at the end
void UARTPrintln(uint8_t pin, char* data);

//write String to UART base without new line at the end
void UARTPrint(uint8_t pin, char* data);

//read String from UART base (limited by MaxUARTLength)
char* UARTStringGet(uint8_t pin);

//write integer date to UART base
void UARTIntegerPut(uint8_t pin,int32_t data);

//used by UARTIntegerPut to convert integer to string
char* integerToString(int32_t i, char b[]);

//read integer value from UART base
uint32_t UARTIntegerGet(uint8_t pin);

//used by UARTIntegerGet to convert string to integer
uint32_t stringToInteger(char* str);

#endif /* TIVAC_GPIO_CONFIG_H_ */
