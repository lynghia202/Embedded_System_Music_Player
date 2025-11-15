#include "lcd_i2c.h"
#include "delay.h"  
static uint8_t g_backlightState = LCD_BACKLIGHT;
void I2C1_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &gpio);

    I2C_InitTypeDef i2c;
    I2C_DeInit(I2C1);
    i2c.I2C_ClockSpeed = 100000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

void LCD_I2C_Write(uint8_t data) {
    while (I2C_GetFlagStatus(LCD_I2C, I2C_FLAG_BUSY));
    I2C_GenerateSTART(LCD_I2C, ENABLE);
    while (!I2C_CheckEvent(LCD_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(LCD_I2C, LCD_I2C_ADDR, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(LCD_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(LCD_I2C, data);
    while (!I2C_CheckEvent(LCD_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(LCD_I2C, ENABLE);
}

void LCD_SendHalf(uint8_t data) {
    LCD_I2C_Write(data | g_backlightState | LCD_ENABLE);
    delay_us(1);
    LCD_I2C_Write(data | g_backlightState);
    delay_us(50);
}

void LCD_SendCommand(uint8_t cmd) {
    uint8_t high = (cmd & 0xF0);
    uint8_t low  = (cmd << 4) & 0xF0;
    LCD_SendHalf(high);
    LCD_SendHalf(low);
}

void LCD_SendData(uint8_t data) {
    uint8_t high = (data & 0xF0) | LCD_REGISTER_SEL;
    uint8_t low  = ((data << 4) & 0xF0) | LCD_REGISTER_SEL;
    LCD_SendHalf(high);
    LCD_SendHalf(low);
}

void LCD_Init(void) {
    I2C1_Init();
		g_backlightState = LCD_BACKLIGHT;
    delay_ms(50);

    LCD_SendHalf(0x30);
    delay_ms(5);
    LCD_SendHalf(0x30);
    delay_us(200);
    LCD_SendHalf(0x30);
    delay_us(200);
    LCD_SendHalf(0x20);  // 4-bit mode
    delay_us(200);

    LCD_SendCommand(0x28); // 4-bit, 2 line
    LCD_SendCommand(0x0C); // Display ON
    LCD_SendCommand(0x06); // Entry mode
    LCD_Clear();
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_SendCommand(addr);
}

void LCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_BacklightOn(void) {
    g_backlightState = LCD_BACKLIGHT;
    LCD_I2C_Write(g_backlightState); 
}

void LCD_BacklightOff(void) {
    g_backlightState = 0x00;
    LCD_I2C_Write(g_backlightState); 
}

