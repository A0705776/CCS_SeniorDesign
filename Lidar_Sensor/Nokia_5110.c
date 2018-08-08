/*
 * Nokia_5110.c
 *
 *  Created on: Apr 3, 2017
 *      Author: Alamin
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driverlib/sysctl.h"

#include "TivaC_GPIO_Config.h"
#include "Nokia_5110.h"

static uint8_t CursorX = 0;
static uint8_t CursorY = 0;
static uint8_t SmallCarCounter = 0;

void LCDSetContrast(uint8_t value){
    LCDWriteCmd(0x21);  //LCD extended commands
    LCDWriteCmd(value);  //set LCD Vop (contrast)B0--D0
    LCDWriteCmd(0x20);  //LCD basic commands
    LCDWriteCmd(0x0C);  //LCD normal video
}
void LCDInit(void){
    GPIOWrite(LCD_RST,LOW);
    SysCtlDelay(10);
    GPIOWrite(LCD_RST,HIGH);

    LCDWriteCmd(0x21);  //LCD extended commands
    LCDWriteCmd(0xB9);  //set LCD Vop (contrast)B0--D0 //c0
    LCDWriteCmd(0x04);  //set temp coefficent //04
    LCDWriteCmd(0x14);  //LCD bias mode 1:40 //14
    LCDWriteCmd(0x20);  //LCD basic commands
    //LCDWriteCmd(0x09);  //LCD all segments on
    LCDWriteCmd(0x0C);  //LCD normal video
    LCD_LED_OFF();
}
void LCD_LED_ON(void){
    GPIOWrite(LCD_LIGHT,LOW);
}
void LCD_LED_OFF(void){
    GPIOWrite(LCD_LIGHT,HIGH);
}
void LCDXY(uint16_t x,uint16_t y){
    LCDWriteCmd(0x80 | x);  //column
    LCDWriteCmd(0x40 | y);  //row
    CursorX = x;
    CursorY = y;
}
void LCDAllPix(void){
    LCDWriteCmd(0x20);  //LCD basic commands
    LCDWriteCmd(0x09);  //LCD all segments on
    LCDWriteCmd(0x0C);  //LCD normal video
}
void LCDClear(void){
    uint16_t counter = 0;
    for(;counter < 504;counter++){
        LCDWriteData(0x00);
    }
    LCDXY(0,0);
}
void LCDWriteStringInv(char *characters){
    uint8_t counter = 0;
    uint16_t length = strlen(characters);
    while(counter < length){
        LCDWriteCharInv(characters[counter++]);
    }
}
void LCDWriteString(char *characters){
    while(*characters){
        LCDWriteChar(*characters++);
    }
}
void LCDWriteCharInv(char character){
    uint8_t counter = 0;
    for(;counter < 5; counter++){
        LCDWriteDataInv(ASCII[character - 0x20][counter]);
    }
    LCDWriteData(0xFF);
}
void LCDWriteChar(char character){
    uint8_t counter = 0;
    for(;counter < 5; counter++){
        LCDWriteData(ASCII[character - 0x20][counter]);
    }
    LCDWriteData(0x00);
}
void LCDWriteFace(void) {

uint16_t i;
for (i=0;i<504;i++)
    LCDWriteData(Face[i]);

}
void LCDSmallCarAnimation(void){
    uint16_t i;
    if(SmallCarCounter >= 9){SmallCarCounter = 0;}
    for (i=0;i<504;i++){
        LCDWriteData(SmallCarAnimation[SmallCarCounter][i]);
    }
    SmallCarCounter++;
}
void LCDWriteDataInv(uint8_t cmd){
    GPIOWrite(LCD_DC,HIGH);
    GPIOWrite(LCD_CE,LOW);
    uint8_t counter = 0;
    for(;counter < 8;counter++){
        GPIOWrite(LCD_CLK,LOW);
        if(cmd & LCDMaskBytes[counter]){
            GPIOWrite(LCD_DIN,LOW);
        }else{
            GPIOWrite(LCD_DIN,HIGH);
        }
        SysCtlDelay(10);
        GPIOWrite(LCD_CLK,HIGH);
        SysCtlDelay(10);
    }
    GPIOWrite(LCD_CE,HIGH);
}
void LCDWriteData(uint8_t cmd){
    GPIOWrite(LCD_DC,HIGH);
    GPIOWrite(LCD_CE,LOW);
    uint8_t counter = 0;
    for(;counter < 8;counter++){
        GPIOWrite(LCD_CLK,LOW);
        if(cmd & LCDMaskBytes[counter]){
            GPIOWrite(LCD_DIN,HIGH);
        }else{
            GPIOWrite(LCD_DIN,LOW);
        }
        SysCtlDelay(10);
        GPIOWrite(LCD_CLK,HIGH);
        SysCtlDelay(10);
    }
    GPIOWrite(LCD_CE,HIGH);
}
void LCDWriteCmd(uint8_t cmd){
    GPIOWrite(LCD_DC,LOW);
    GPIOWrite(LCD_CE,LOW);
    uint8_t counter = 0;
    for(;counter < 8;counter++){
        GPIOWrite(LCD_CLK,LOW);
        if(cmd & LCDMaskBytes[counter]){
            GPIOWrite(LCD_DIN,HIGH);
        }else{
            GPIOWrite(LCD_DIN,LOW);
        }
        SysCtlDelay(10);
        GPIOWrite(LCD_CLK,HIGH);
        SysCtlDelay(10);
    }
    GPIOWrite(LCD_CE,HIGH);
}
