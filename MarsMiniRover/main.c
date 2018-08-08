/*
 *  ======================================== Main.c ==========================================
 *  Author:         Alamin Ahmed
 *  Date Created:   10/07/2017
 */

/* XDCtools Header files */
#include <xdc/std.h>                        //mandatory - have to include first, for BIOS types
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <xdc/std.h>                        //mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h>                //mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>                //needed for any Log_info() call
#include <xdc/cfg/global.h>                 //header file for statically defined objects/handles

/* Board Header file */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driverlib/sysctl.h"
#include "Semaphore.h"


// ============================== User Header file =================================
#include "TivaC_GPIO_Config.h"
#include "Nokia_5110.h"
// =============================== User Definition =================================
#define             LeftMotor_Dir              PB3              //output
#define             LeftMotor_PWM              PD1              //pwm
#define             RightMotor_Dir             PB2              //output
#define             RightMotor_PWM             PD0              //pwm

#define             Front_ADC                  PE2              //adc
#define             Right_ADC                  PE3              //adc pd1 is ont working, need to check the pcb


#define             Front_Sequence             2                //adc sequence
#define             Right_Sequence             3                //adc sequence
//#define             Right_Encoder              PF2              //input
//#define             Left_Encoder               PF3              //input
#define             Bluetooth_UART_TX          PB1              //uart
#define             Bluetooth_UART_RX          PB0              //uart
#define             User_UART_TX               PA1              //uart
#define             User_UART_RX               PA0              //uart
#define             UART_Baudrate              115200           //uart baudrate
#define             ADC_SampleRate             1                //ADC sample rate
//#define             Red_LED                    PF1              //pwm

#define             Reflectro_Sensor           PF1              //input
//-----LCD-----
//#define             LCD_LIGHT                  PB2              //output
//#define             LCD_CE                     PA6              //output
//#define             LCD_RST                    PA7              //output
//#define             LCD_DC                     PB4              //output
//#define             LCD_CLK                    PE4              //output
//#define             LCD_DIN                    PE5              //output

// =============================== Global Variable =================================

//-----STATE-----
enum{
    Contrler,
    Normal
};

//-----UART-----
_Bool               UART_Contrler            = false;
_Bool               UARTNewChar_Flag         = false;
//-----Used BY Functions-----
static char         cmd;
static uint16_t     Idle_Counter             = 0;
//-----Robot-----
static uint16_t     Right_Speed              = 0;
static uint16_t     Left_Speed               = 0;
static uint16_t     Speed                    = 0;


// ============================ Function Declaration ==============================
void GPIOInit(void);
void MoveForward(void);
void MoveBackward(void);
void MoveSlightRight(void);
void MoveSlighLeft(void);
void MoveRight(void);
void MoveLeft(void);
void RotateRight(void);
void RotateLeft(void);
void StopMoving(void);
void StartMoving(void);
void Robot_Idle(void);
void ADCClockFunc(void);
void ADCSwiFunc(void);
void User_UART_Hwi(void);
void Bluetooth_UART_Hwi(void);
void DefaultBluetoothSetup(void);
uint16_t SetSpeed(uint16_t speed);
void Contrler_Task(void);
void MoveBackRight(void);
void MoveBackLeft(void);
void LCDSwi(void);
void Bluetooth_UART_Swi(void);
void ControlerTimer(void);
char getHexShiftPlease(int32_t number, int32_t shiftby);
void Shoot(void);
void MissUp(void);
void MissDown(void);
void PID_Task(void);
// ===================================== Main ======================================
int main(void){

    GPIOInit();
    UARTPrintln(User_UART_TX,"===== Tiva-c Initiated =====");
    UARTPrintln(Bluetooth_UART_TX,"===== Tiva-c Initiated =====");

    BIOS_start();
    return 0;
}
void GPIOInit(void){
    //----------------------ADC-------------------------
    GPIOSet(Front_ADC,ADC);
    ADC0Setup(Front_ADC,Front_Sequence);
    GPIOSet(Right_ADC,ADC);
    ADC0Setup(Right_ADC,Right_Sequence);

    //----------------------PWM-------------------------
    GPIOSet(LeftMotor_PWM,PWM);
    GPIOSet(RightMotor_PWM,PWM);
    //GPIOSet(Red_LED,PWM);
    //setPWMOn(Red_LED);

    //--------------------Output------------------------
    GPIOSet(LeftMotor_Dir,OUTPUT);
    GPIOSet(RightMotor_Dir,OUTPUT);
    //---------------------Input------------------------

    //---------------------UART-------------------------
    GPIOSet(User_UART_TX,UART);
    GPIOSet(User_UART_RX,UART);
    UARTSetBaud(User_UART_TX,UART_Baudrate);
    UARTEnableInt(User_UART_RX);
    GPIOSet(Bluetooth_UART_TX,UART);
    GPIOSet(Bluetooth_UART_RX,UART);
    UARTSetBaud(Bluetooth_UART_TX,UART_Baudrate);
    UARTEnableInt(Bluetooth_UART_RX);
    //------------------Interrupt----------------------

}



void User_UART_Hwi(void){
    UARTClearInt(User_UART_RX);
    UARTStringGet(User_UART_RX);
}
void DefaultBluetoothSetup(void){
    StopMoving();
    UART_Contrler            = false;
    UARTNewChar_Flag         = false;
    UARTPrintln(Bluetooth_UART_TX,"Reset");
}
void Bluetooth_UART_Hwi(void){
    UARTClearInt(Bluetooth_UART_RX);
    if(UART_Contrler){
        Semaphore_post(controlerSem);
    }else{
        char ID;
        ID = UARTGetChar(Bluetooth_UART_RX);
        switch(ID){
        case '!': DefaultBluetoothSetup(); break;
        case '#': UART_Contrler = true; UARTPrintln(Bluetooth_UART_TX,"Controller Mode"); break;
        }
    }
}


void Contrler_Task(void){
    SetSpeed((uint16_t)PWMPeriod/2); //about 50% dutycycle
    while(1){
        Semaphore_pend(controlerSem,BIOS_WAIT_FOREVER);
        cmd = UARTGetChar(Bluetooth_UART_RX);
        switch(cmd){
            case '1': MoveForward(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Up"); break;
            case '2': MoveBackward(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Down"); break;
            case '3': MoveRight(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Right"); break;
            case '4': MoveLeft(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Left"); break;
            case '5': MoveSlightRight(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Up Right");break;
            case '6': MoveSlighLeft(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Up Left"); break;
            case '7': MoveBackRight(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Down Right"); break;
            case '8': MoveBackLeft(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Down Left"); break;
            case 'R': RotateRight(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Rotate Right"); break;
            case 'L': RotateLeft(); StartMoving(); UARTPrintln(Bluetooth_UART_TX,"Rotate Left"); break;
            case 'S': SetSpeed(UARTIntegerGet(Bluetooth_UART_RX)); UARTPrintln(Bluetooth_UART_TX,"Set Speed to: "); UARTIntegerPut(Bluetooth_UART_TX,Right_Speed); break;
            case '!': DefaultBluetoothSetup(); break;
        }
        Timer_start(stopMotors);
    }
}

uint16_t SetSpeed(uint16_t speed){
    Right_Speed = speed;
    Left_Speed = speed;
    Speed = speed;
    setPWMDuty(LeftMotor_PWM,Left_Speed);
    setPWMDuty(RightMotor_PWM,Right_Speed);
    return speed;
}
void MoveForward(void){
    GPIOWrite(LeftMotor_Dir,LOW);
    GPIOWrite(RightMotor_Dir,LOW);
    setPWMDuty(LeftMotor_PWM,Left_Speed);
    setPWMDuty(RightMotor_PWM,Right_Speed);
}
void MoveBackward(void){
    GPIOWrite(LeftMotor_Dir,HIGH);
    GPIOWrite(RightMotor_Dir,HIGH);
    setPWMDuty(LeftMotor_PWM,Speed);
    setPWMDuty(RightMotor_PWM,Speed);
}
void MoveBackRight(void){
    GPIOWrite(LeftMotor_Dir,HIGH);
    GPIOWrite(RightMotor_Dir,HIGH);
    setPWMDuty(LeftMotor_PWM,Left_Speed);
    setPWMDuty(RightMotor_PWM,(uint16_t)Right_Speed/2);
}
void MoveBackLeft(void){
    GPIOWrite(LeftMotor_Dir,HIGH);
    GPIOWrite(RightMotor_Dir,HIGH);
    setPWMDuty(LeftMotor_PWM,(uint16_t)Left_Speed/2);
    setPWMDuty(RightMotor_PWM,Right_Speed);
}
void MoveSlightRight(void){
    GPIOWrite(LeftMotor_Dir,LOW);
    GPIOWrite(RightMotor_Dir,LOW);
    setPWMDuty(LeftMotor_PWM,Left_Speed);
    setPWMDuty(RightMotor_PWM,(uint16_t)Right_Speed/2);
}
void MoveSlighLeft(void){
    GPIOWrite(LeftMotor_Dir,LOW);
    GPIOWrite(RightMotor_Dir,LOW);
    setPWMDuty(LeftMotor_PWM,(uint16_t)Left_Speed/2);
    setPWMDuty(RightMotor_PWM,Right_Speed);
}
void MoveRight(void){
    GPIOWrite(LeftMotor_Dir,LOW);
    GPIOWrite(RightMotor_Dir,LOW);
    setPWMDuty(LeftMotor_PWM,Left_Speed);
    setPWMDuty(RightMotor_PWM,1);
}
void MoveLeft(void){
    GPIOWrite(LeftMotor_Dir,LOW);
    GPIOWrite(RightMotor_Dir,LOW);
    setPWMDuty(LeftMotor_PWM,1);
    setPWMDuty(RightMotor_PWM,Right_Speed);
}
void RotateRight(void){
    GPIOWrite(LeftMotor_Dir,LOW);
    GPIOWrite(RightMotor_Dir,HIGH);
    setPWMDuty(LeftMotor_PWM,Left_Speed);
    setPWMDuty(RightMotor_PWM,Right_Speed);
}
void RotateLeft(void){
    GPIOWrite(LeftMotor_Dir,HIGH);
    GPIOWrite(RightMotor_Dir,LOW);
    setPWMDuty(LeftMotor_PWM,Left_Speed);
    setPWMDuty(RightMotor_PWM,Right_Speed);
}
void StopMoving(void){
    setPWMOff(LeftMotor_PWM);
    setPWMOff(RightMotor_PWM);

}
void StartMoving(void){
    setPWMOn(LeftMotor_PWM);
    setPWMOn(RightMotor_PWM);
}
//just for the color changing
void Robot_Idle(void){

//    for(Idle_Counter = 1;  Idle_Counter < PWMPeriod;Idle_Counter++){
//        setPWMDuty(Red_LED,Idle_Counter);
////        UARTPrint(User_UART_TX,"chicken: ");
////        UARTIntegerPut(User_UART_TX,Idle_Counter);
////        UARTPrint(User_UART_TX,"\r\n");
//        SysCtlDelay(100000);
//    }
//    for(Idle_Counter--;  Idle_Counter > 1;Idle_Counter--){
//        setPWMDuty(Red_LED,Idle_Counter);
////        UARTPrint(User_UART_TX,"chicken: ");
////        UARTIntegerPut(User_UART_TX,Idle_Counter);
////        UARTPrint(User_UART_TX,"\r\n");
//        SysCtlDelay(100000);
//    }
}
