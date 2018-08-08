/*
 * TivaC_GPIO_Config.h
 *
 *  Created on: Mar 20, 2017
 *      Author: Alamin Ahmed
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "TivaC_GPIO_Config.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


    uint32_t ui32Peripheral;
    uint32_t ui32Port;
    uint8_t  ui8Pins;
    uint32_t PWMui32PinConfig;
    uint32_t PWMui32Peripheral;
    uint32_t PWMui32Base;
    uint32_t PWMui32Gen;
    uint32_t ui32PWMOut;
    uint32_t ui32PWMOutBits;
    uint8_t ADCSetupInfo[12][3] ={{PE3,0,0},
                                  {PE2,0,0},
                                  {PE1,0,0},
                                  {PE0,0,0},
                                  {PD3,0,0},
                                  {PD2,0,0},
                                  {PD1,0,0},
                                  {PD0,0,0},
                                  {PE5,0,0},
                                  {PE4,0,0},
                                  {PB4,0,0},
                                  {PB5,0,0}};
    uint32_t channel;
    uint32_t ADCBase;
    uint32_t UARTui32Peripheral;
    uint32_t UARTui32PinConfig;
    uint32_t UARTui32Base;
    char StringBuff[MaxUARTLength];
    char CharBuff;
    uint32_t index = 0;
    static _Bool firstTimeUesd = false;

void setPinsToWTF(void){
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}

void setPinsToDefault(void){
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                          //enable the ADC0 peripheral
//    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);      //reset to apply changes
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);                          //enable the ADC0 peripheral
//    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC1);      //reset to apply changes

    uint8_t counter = 1;
    GPIOSet(counter,OUTPUT);
    counter = 5; //skip 2,3
    for(;counter < 11;counter++){
        GPIOSet(counter,OUTPUT);
    }
    counter = 13; //skip 11, 12
    for(;counter < 25;counter++){
        GPIOSet(counter,OUTPUT);
    }
    counter = 28; // skip 25,26,27
    for(;counter < 32;counter++){
        GPIOSet(counter,OUTPUT);
    }
    counter = 43; //skip 32-42
    for(;counter < 54;counter++){
        GPIOSet(counter,OUTPUT);
    }
    counter = 57; //skip 54,55,56
    for(;counter < 65;counter++){
        GPIOSet(counter,OUTPUT);
    }
}

void GPIOSet(uint8_t pin, char function){
    if(!firstTimeUesd){
        firstTimeUesd = true;
        setPinsToWTF();
        //setPinsToDefault();
    }
    GPIOPinInfo(pin,&ui32Peripheral,&ui32Port,&ui8Pins);
    switch(function){
    case 'I':
        SysCtlPeripheralEnable(ui32Peripheral);
        GPIOPinTypeGPIOInput(ui32Port, ui8Pins);
        break;
    case 'O':
        SysCtlPeripheralEnable(ui32Peripheral);
        GPIOPinTypeGPIOOutput(ui32Port, ui8Pins);
        break;
    case 'P':
        PWMPinInfo(pin, &PWMui32PinConfig, &PWMui32Peripheral, &PWMui32Base, &PWMui32Gen, &ui32PWMOut, &ui32PWMOutBits);
        SysCtlPeripheralEnable(PWMui32Peripheral);
        GPIOPinConfigure(PWMui32PinConfig);
        GPIOPinTypePWM(ui32Port, ui8Pins);

        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                                                 //Configure PWM Clock to match system
        PWMGenConfigure(PWMui32Base, PWMui32Gen, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); //Configure PWM Options
        PWMGenPeriodSet(PWMui32Base, PWMui32Gen, PWMPeriod);                                //Set the Period (expressed in clock ticks)
        PWMPulseWidthSet(PWMui32Base, ui32PWMOut,PWMPulseWidth);                            //Set PWM duty-0%
        PWMGenEnable(PWMui32Base, PWMui32Gen);                                              // Enable the PWM generator
        PWMOutputState(PWMui32Base, ui32PWMOutBits, false);                                 // Turn off the Output pins
        break;
    case 'A':
        SysCtlPeripheralEnable(ui32Peripheral);         //enable the ADC peripheral
        GPIOPinTypeADC(ui32Port, ui8Pins);              //Activate ADC function
        break;
    case 'U':
        UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
        SysCtlPeripheralEnable(UARTui32Peripheral);     //***
        GPIOPinConfigure(UARTui32PinConfig);            //***
        GPIOPinTypeUART(ui32Port, ui8Pins);
        break;
    default:                                            //output
        SysCtlPeripheralEnable(ui32Peripheral);
        GPIOPinTypeGPIOOutput(ui32Port, ui8Pins);
        break;
    }
}
char UARTGetChar(uint8_t pin){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    return UARTCharGet(UARTui32Base);
}
void UARTClearInt(uint8_t pin){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    UARTIntClear(UARTui32Base,(UART_INT_RT | UART_INT_RX));
}
void UARTPutChar(uint8_t pin, unsigned char data){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    UARTCharPut(UARTui32Base, data);
}
void UARTStringPut(uint8_t pin, char* data){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    uint8_t i = 0;
    uint8_t length = strlen(data);
    for (; i < length; i++){
        UARTCharPut(UARTui32Base, data[i]);
    }
    UARTCharPut(UARTui32Base,'\r');
    UARTCharPut(UARTui32Base,'\n');
}
char* UARTStringGet(uint8_t pin){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    index = 0;
    memset(StringBuff,0,MaxUARTLength);
    while(UARTCharsAvail(UARTui32Base)){
        CharBuff = UARTCharGet(UARTui32Base);
        if (CharBuff == '\r' | CharBuff == '\n'){
               break;
        }else if(index < MaxUARTLength){
            StringBuff[index] = CharBuff;
        }else{
            UARTStringPut(UARTui32Base,"Too many CHARs you Retard");
            break;
        }
        index++;
    }
    return StringBuff;
}
//following function is borrowed from the Internet :p
char* integerToString(int32_t i, char b[]){
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}
void UARTIntegerPut(uint8_t pin,int32_t data){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    memset(StringBuff,0,MaxUARTLength);
    integerToString(data,StringBuff);
    UARTStringPut(pin,StringBuff);
}
uint32_t stringToInteger(char* str){
    uint32_t mult = 1;
    uint32_t data = 0;
    uint32_t length = strlen(str);
    int32_t i;
    for(i = length -1 ; i >= 0 ; i--){
        data = data + ((int)str[i] -48)*mult;
        mult = mult*10;
    }
    return data;
}
uint32_t UARTIntegerGet(uint8_t pin){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    strcpy(StringBuff,UARTStringGet(pin));
    return stringToInteger(StringBuff);
}
void UARTDisableInt(uint8_t pin){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    UARTIntDisable(UARTui32Base,(UART_INT_RT | UART_INT_RX)); // Receive Timeout Interrupt Mask + Receive Interrupt Mask
}
void UARTEnableInt(uint8_t pin){
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    UARTIntEnable(UARTui32Base,(UART_INT_RT | UART_INT_RX)); // Receive Timeout Interrupt Mask + Receive Interrupt Mask
}
void UARTSetBaud(uint8_t pin, uint32_t BaudRate){
    //system clock
    //8 bit | one stop bit | no parity
    UARTPinInfo(pin, &UARTui32Peripheral, &UARTui32PinConfig, &UARTui32Base);
    UARTConfigSetExpClk(UARTui32Base,SysCtlClockGet(),BaudRate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
void UARTPinInfo(uint8_t pin, uint32_t *UARTui32Peripheral, uint32_t *UARTui32PinConfig, uint32_t *UARTui32Base){
    switch(pin){
    case PA1: *UARTui32Peripheral = SYSCTL_PERIPH_UART0; *UARTui32PinConfig = GPIO_PA1_U0TX; *UARTui32Base = UART0_BASE; break;
    case PA0: *UARTui32Peripheral = SYSCTL_PERIPH_UART0; *UARTui32PinConfig = GPIO_PA0_U0RX; *UARTui32Base = UART0_BASE; break;
    case PB1: *UARTui32Peripheral = SYSCTL_PERIPH_UART1; *UARTui32PinConfig = GPIO_PB1_U1TX; *UARTui32Base = UART1_BASE; break;
    case PB0: *UARTui32Peripheral = SYSCTL_PERIPH_UART1; *UARTui32PinConfig = GPIO_PB0_U1RX; *UARTui32Base = UART1_BASE; break;
    case PD7: *UARTui32Peripheral = SYSCTL_PERIPH_UART2; *UARTui32PinConfig = GPIO_PD7_U2TX; *UARTui32Base = UART2_BASE; break;
    case PD6: *UARTui32Peripheral = SYSCTL_PERIPH_UART2; *UARTui32PinConfig = GPIO_PD6_U2RX; *UARTui32Base = UART2_BASE; break;
    case PC7: *UARTui32Peripheral = SYSCTL_PERIPH_UART3; *UARTui32PinConfig = GPIO_PC7_U3TX; *UARTui32Base = UART3_BASE; break;
    case PC6: *UARTui32Peripheral = SYSCTL_PERIPH_UART3; *UARTui32PinConfig = GPIO_PC6_U3RX; *UARTui32Base = UART3_BASE; break;
    case PC5: *UARTui32Peripheral = SYSCTL_PERIPH_UART4; *UARTui32PinConfig = GPIO_PC5_U4TX; *UARTui32Base = UART4_BASE; break;
    case PC4: *UARTui32Peripheral = SYSCTL_PERIPH_UART4; *UARTui32PinConfig = GPIO_PC4_U4RX; *UARTui32Base = UART4_BASE; break;
    case PE5: *UARTui32Peripheral = SYSCTL_PERIPH_UART5; *UARTui32PinConfig = GPIO_PE5_U5TX; *UARTui32Base = UART5_BASE; break;
    case PE4: *UARTui32Peripheral = SYSCTL_PERIPH_UART5; *UARTui32PinConfig = GPIO_PE4_U5RX; *UARTui32Base = UART5_BASE; break;
    case PD5: *UARTui32Peripheral = SYSCTL_PERIPH_UART6; *UARTui32PinConfig = GPIO_PD5_U6TX; *UARTui32Base = UART6_BASE; break;
    case PD4: *UARTui32Peripheral = SYSCTL_PERIPH_UART6; *UARTui32PinConfig = GPIO_PD4_U6RX; *UARTui32Base = UART6_BASE; break;
    case PE1: *UARTui32Peripheral = SYSCTL_PERIPH_UART7; *UARTui32PinConfig = GPIO_PE1_U7TX; *UARTui32Base = UART7_BASE; break;
    case PE0: *UARTui32Peripheral = SYSCTL_PERIPH_UART7; *UARTui32PinConfig = GPIO_PE0_U7RX; *UARTui32Base = UART7_BASE; break;
    default:  *UARTui32Peripheral = SYSCTL_PERIPH_UART0; *UARTui32PinConfig = GPIO_PA1_U0TX; *UARTui32Base = UART0_BASE; break; //PA1
    }
}
uint32_t ADCRead(uint32_t pin){
    uint8_t counter;
    uint32_t result = 0;
    for(counter = 0;counter < 12;counter++){
        if(ADCSetupInfo[counter][0] == pin) break;
    }
    if(ADCSetupInfo[counter][2] == 0) ADCBase = ADC0_BASE;
    else ADCBase = ADC1_BASE;

    ADCIntClear(ADCBase,ADCSetupInfo[counter][1]);
    ADCProcessorTrigger(ADCBase, ADCSetupInfo[counter][1]);
    while(!ADCIntStatus(ADCBase, ADCSetupInfo[counter][1], false)){}
    ADCSequenceDataGet(ADCBase, ADCSetupInfo[counter][1], &result);
    return result;
}
void ADC1Setup(uint8_t pin, uint32_t sequence){
    ADCPinInfo(pin, &channel);

    uint8_t counter;
    for(counter = 0;counter < 12;counter++){
        if(ADCSetupInfo[counter][0] == pin) break;
    }
    ADCSetupInfo[counter][1] = sequence;
    ADCSetupInfo[counter][2] = 1;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);                          //enable the ADC0 peripheral
    ADCSequenceDisable(ADC1_BASE, sequence);                             //disable sequencer
    ADCSequenceConfigure(ADC1_BASE, sequence, ADC_TRIGGER_PROCESSOR, 0); //ADC Base, ADC_TRIGGER_PROCESSOR
    ADCSequenceStepConfigure(ADC1_BASE,sequence,0,channel|ADC_CTL_IE|ADC_CTL_END);
    //ADCIntEnable(ADC1_BASE, sequence);
    ADCSequenceEnable(ADC1_BASE, sequence);                              //enable ADC sequencer
    //IntMasterEnable();
}
void ADC0Setup(uint8_t pin, uint32_t sequence){
    ADCPinInfo(pin, &channel);

    uint8_t counter;
    for(counter = 0;counter < 12;counter++){
        if(ADCSetupInfo[counter][0] == pin) break;
    }
    ADCSetupInfo[counter][1] = sequence;
    ADCSetupInfo[counter][2] = 0;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                          //enable the ADC0 peripheral
    ADCSequenceDisable(ADC0_BASE, sequence);                             //disable sequencer
    ADCSequenceConfigure(ADC0_BASE, sequence, ADC_TRIGGER_PROCESSOR, 0); //ADC Base, ADC_TRIGGER_PROCESSOR
    ADCSequenceStepConfigure(ADC0_BASE,sequence,0,channel|ADC_CTL_IE|ADC_CTL_END);
    //ADCIntEnable(ADC1_BASE, sequence);
    ADCSequenceEnable(ADC0_BASE, sequence);                              //enable ADC sequencer
    //IntMasterEnable();                                                   //Enable global interrupts
}
void ADCPinInfo(uint8_t pin, uint32_t *channel){
    switch(pin){
    case PE3: *channel = ADC_CTL_CH0; break;
    case PE2: *channel = ADC_CTL_CH1; break;
    case PE1: *channel = ADC_CTL_CH2; break;
    case PE0: *channel = ADC_CTL_CH3; break;
    case PD3: *channel = ADC_CTL_CH4; break;
    case PD2: *channel = ADC_CTL_CH5; break;
    case PD1: *channel = ADC_CTL_CH6; break;
    case PD0: *channel = ADC_CTL_CH7; break;
    case PE5: *channel = ADC_CTL_CH8; break;
    case PE4: *channel = ADC_CTL_CH9; break;
    case PB4: *channel = ADC_CTL_CH10; break;
    case PB5: *channel = ADC_CTL_CH11; break;
    default:  *channel = ADC_CTL_CH0; break; //PE3
    }
}
void setPWMDuty(uint8_t pin,uint32_t duty){
    PWMPinInfo(pin, &PWMui32PinConfig, &PWMui32Peripheral, &PWMui32Base, &PWMui32Gen, &ui32PWMOut, &ui32PWMOutBits);
    PWMPulseWidthSet(PWMui32Base, ui32PWMOut,duty);                                         //Set PWM duty
}
void setPWMOn(uint8_t pin){
    PWMPinInfo(pin, &PWMui32PinConfig, &PWMui32Peripheral, &PWMui32Base, &PWMui32Gen, &ui32PWMOut, &ui32PWMOutBits);
    PWMOutputState(PWMui32Base, ui32PWMOutBits, true);                                     // Turn on the Output pins
}
void setPWMOff(uint8_t pin){
    PWMPinInfo(pin, &PWMui32PinConfig, &PWMui32Peripheral, &PWMui32Base, &PWMui32Gen, &ui32PWMOut, &ui32PWMOutBits);
    PWMOutputState(PWMui32Base, ui32PWMOutBits, false);                                     // Turn off the Output pins
}
void PWMPinInfo(uint8_t pin, uint32_t *ui32PinConfig, uint32_t *ui32Peripheral,uint32_t *PWMui32Base,uint32_t *PWMui32Gen,uint32_t *ui32PWMOut,uint32_t *ui32PWMOutBits){
    switch(pin){
    case PB6: *ui32PinConfig = GPIO_PB6_M0PWM0; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_0; *ui32PWMOut = PWM_OUT_0; *ui32PWMOutBits = PWM_OUT_0_BIT; break;
    case PC5: *ui32PinConfig = GPIO_PC5_M0PWM7; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_3; *ui32PWMOut = PWM_OUT_7; *ui32PWMOutBits = PWM_OUT_7_BIT; break;
    case PC4: *ui32PinConfig = GPIO_PC4_M0PWM6; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_3; *ui32PWMOut = PWM_OUT_6; *ui32PWMOutBits = PWM_OUT_6_BIT; break;
    case PB7: *ui32PinConfig = GPIO_PB7_M0PWM1; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_0; *ui32PWMOut = PWM_OUT_1; *ui32PWMOutBits = PWM_OUT_1_BIT; break;
    case PB5: *ui32PinConfig = GPIO_PB5_M0PWM3; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_1; *ui32PWMOut = PWM_OUT_3; *ui32PWMOutBits = PWM_OUT_3_BIT; break;
    case PB4: *ui32PinConfig = GPIO_PB4_M0PWM2; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_1; *ui32PWMOut = PWM_OUT_2; *ui32PWMOutBits = PWM_OUT_2_BIT; break;
    case PE4: *ui32PinConfig = GPIO_PE4_M0PWM4; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_2; *ui32PWMOut = PWM_OUT_4; *ui32PWMOutBits = PWM_OUT_4_BIT; break;
    case PE5: *ui32PinConfig = GPIO_PE5_M0PWM5; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_2; *ui32PWMOut = PWM_OUT_5; *ui32PWMOutBits = PWM_OUT_5_BIT; break;

    case PF1: *ui32PinConfig = GPIO_PF1_M1PWM5; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_2; *ui32PWMOut = PWM_OUT_5; *ui32PWMOutBits = PWM_OUT_5_BIT; break;
    case PA6: *ui32PinConfig = GPIO_PA6_M1PWM2; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_1; *ui32PWMOut = PWM_OUT_2; *ui32PWMOutBits = PWM_OUT_2_BIT; break;
    case PA7: *ui32PinConfig = GPIO_PA7_M1PWM3; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_1; *ui32PWMOut = PWM_OUT_3; *ui32PWMOutBits = PWM_OUT_3_BIT; break;
    case PF0: *ui32PinConfig = GPIO_PF0_M1PWM4; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_2; *ui32PWMOut = PWM_OUT_4; *ui32PWMOutBits = PWM_OUT_4_BIT; break;
    case PF2: *ui32PinConfig = GPIO_PF2_M1PWM6; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_3; *ui32PWMOut = PWM_OUT_6; *ui32PWMOutBits = PWM_OUT_6_BIT; break;
    case PF3: *ui32PinConfig = GPIO_PF3_M1PWM7; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_3; *ui32PWMOut = PWM_OUT_7; *ui32PWMOutBits = PWM_OUT_7_BIT; break;
    case PD0: *ui32PinConfig = GPIO_PD0_M1PWM0; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_0; *ui32PWMOut = PWM_OUT_0; *ui32PWMOutBits = PWM_OUT_0_BIT; break;
    case PD1: *ui32PinConfig = GPIO_PD1_M1PWM1; *ui32Peripheral = SYSCTL_PERIPH_PWM1; *PWMui32Base = PWM1_BASE; *PWMui32Gen = PWM_GEN_0; *ui32PWMOut = PWM_OUT_1; *ui32PWMOutBits = PWM_OUT_1_BIT; break;
    default:  *ui32PinConfig = GPIO_PB6_M0PWM0; *ui32Peripheral = SYSCTL_PERIPH_PWM0; *PWMui32Base = PWM0_BASE; *PWMui32Gen = PWM_GEN_0; *ui32PWMOut = PWM_OUT_0; *ui32PWMOutBits = PWM_OUT_0_BIT; break; //PB6
    }
}
_Bool GPIORead(uint8_t pin){
    GPIOPinInfo(pin,&ui32Peripheral,&ui32Port,&ui8Pins);
    if(GPIOPinRead(ui32Port,ui8Pins) == ui8Pins) return true;
    else return false;
}
void GPIOWrite(uint8_t pin,_Bool state){
    GPIOPinInfo(pin,&ui32Peripheral,&ui32Port,&ui8Pins);
    if(state) GPIOPinWrite(ui32Port, ui8Pins,ui8Pins);
    else      GPIOPinWrite(ui32Port, ui8Pins,0);
}
void InterruptEnable(uint8_t pin){
    GPIOPinInfo(pin,&ui32Peripheral,&ui32Port,&ui8Pins);
    GPIOIntEnable(ui32Port,ui8Pins);
}
void InterruptDisable(uint8_t pin){
    GPIOPinInfo(pin,&ui32Peripheral,&ui32Port,&ui8Pins);
    GPIOIntDisable(ui32Port,ui8Pins);
}
void InterruptClear(uint8_t pin){
    GPIOPinInfo(pin,&ui32Peripheral,&ui32Port,&ui8Pins);
    GPIOIntClear(ui32Port,ui8Pins);
}
uint32_t InterruptStatus(uint8_t pin){
    GPIOPinInfo(pin,&ui32Peripheral,&ui32Port,&ui8Pins);
    return GPIOIntStatus(ui32Port,true);
}
void GPIOPinInfo(uint8_t pin, uint32_t *ui32Peripheral, uint32_t *ui32Port, uint8_t *ui8Pins){
    switch(pin){
    case PF1: *ui32Peripheral = SYSCTL_PERIPH_GPIOF; *ui32Port = GPIO_PORTF_BASE; *ui8Pins = GPIO_PIN_1; break;
    case PB6: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_6; break;
    case PF4: *ui32Peripheral = SYSCTL_PERIPH_GPIOF; *ui32Port = GPIO_PORTF_BASE; *ui8Pins = GPIO_PIN_4; break;
    case PE3: *ui32Peripheral = SYSCTL_PERIPH_GPIOE; *ui32Port = GPIO_PORTE_BASE; *ui8Pins = GPIO_PIN_3; break;
    case PE2: *ui32Peripheral = SYSCTL_PERIPH_GPIOE; *ui32Port = GPIO_PORTE_BASE; *ui8Pins = GPIO_PIN_2; break;
    case PE1: *ui32Peripheral = SYSCTL_PERIPH_GPIOE; *ui32Port = GPIO_PORTE_BASE; *ui8Pins = GPIO_PIN_1; break;
    case PE0: *ui32Peripheral = SYSCTL_PERIPH_GPIOE; *ui32Port = GPIO_PORTE_BASE; *ui8Pins = GPIO_PIN_0; break;
    case PD7: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_7; break;
    case PC7: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_7; break;
    case PC6: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_6; break;
    case PC5: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_5; break;
    case PC4: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_4; break;
    case PA0: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_0; break;
    case PA1: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_1; break;
    case PA2: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_2; break;
    case PA3: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_3; break;
    case PA4: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_4; break;
    case PA5: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_5; break;
    case PA6: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_6; break;
    case PA7: *ui32Peripheral = SYSCTL_PERIPH_GPIOA; *ui32Port = GPIO_PORTA_BASE; *ui8Pins = GPIO_PIN_7; break;
    case PF0: *ui32Peripheral = SYSCTL_PERIPH_GPIOF; *ui32Port = GPIO_PORTF_BASE; *ui8Pins = GPIO_PIN_0; break;
    case PB7: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_7; break;
    case PF2: *ui32Peripheral = SYSCTL_PERIPH_GPIOF; *ui32Port = GPIO_PORTF_BASE; *ui8Pins = GPIO_PIN_2; break;
    case PF3: *ui32Peripheral = SYSCTL_PERIPH_GPIOF; *ui32Port = GPIO_PORTF_BASE; *ui8Pins = GPIO_PIN_3; break;
    case PD4: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_4; break;
    case PD5: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_5; break;
    case PB0: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_0; break;
    case PB1: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_1; break;
    case PB2: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_2; break;
    case PB3: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_3; break;
    case PC3: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_3; break;
    case PC2: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_2; break;
    case PC1: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_1; break;
    case PC0: *ui32Peripheral = SYSCTL_PERIPH_GPIOC; *ui32Port = GPIO_PORTC_BASE; *ui8Pins = GPIO_PIN_0; break;
    case PD6: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_6; break;
    case PB5: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_5; break;
    case PB4: *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_4; break;
    case PE4: *ui32Peripheral = SYSCTL_PERIPH_GPIOE; *ui32Port = GPIO_PORTE_BASE; *ui8Pins = GPIO_PIN_4; break;
    case PE5: *ui32Peripheral = SYSCTL_PERIPH_GPIOE; *ui32Port = GPIO_PORTE_BASE; *ui8Pins = GPIO_PIN_5; break;
    case PD0: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_0; break;
    case PD1: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_1; break;
    case PD2: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_2; break;
    case PD3: *ui32Peripheral = SYSCTL_PERIPH_GPIOD; *ui32Port = GPIO_PORTD_BASE; *ui8Pins = GPIO_PIN_3; break;
    default:  *ui32Peripheral = SYSCTL_PERIPH_GPIOB; *ui32Port = GPIO_PORTB_BASE; *ui8Pins = GPIO_PIN_6; break; //PB6
    }
}
