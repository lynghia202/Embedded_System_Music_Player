#ifndef __LCD_I2C_H
#define __LCD_I2C_H

#include "stm32f10x.h"
#define LCD_I2C_ADDR      0x4E  
#define LCD_I2C           I2C1
#define LCD_BACKLIGHT     0x08
#define LCD_ENABLE        0x04
#define LCD_READ_WRITE    0x02
#define LCD_REGISTER_SEL  0x01

void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void LCD_I2C_Write(uint8_t data);
void LCD_BacklightOn(void);
void LCD_BacklightOff(void);


#endif
