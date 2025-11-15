#include "stm32f10x.h"
#include "delay.h"

const char keypad_keys[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

const uint16_t row_pins[] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3};
const uint16_t col_pins[] = {GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7};
const uint32_t all_row_pins = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
const uint32_t all_col_pins = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;


void Keypad_Configure_For_Scan(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio_row;
    // Row pins (PA0-PA3) is Output Push-Pull
    gpio_row.GPIO_Pin = all_row_pins;
    gpio_row.GPIO_Mode = GPIO_Mode_Out_PP; 
    gpio_row.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_row);

    GPIO_InitTypeDef gpio_col;
    // Col pins (PA4-PA7) is Input Pull-up
    gpio_col.GPIO_Pin = all_col_pins;
    gpio_col.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOA, &gpio_col);
    
    GPIO_SetBits(GPIOA, all_row_pins);
}


void Keypad_Configure_For_Interrupt(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio_row;
    // Row pins (PA0-PA3) is Input Pull-up
    gpio_row.GPIO_Pin = all_row_pins;
    gpio_row.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOA, &gpio_row);

    GPIO_InitTypeDef gpio_col;
    // Col pins (PA4-PA7) is Output Push-Pull
    gpio_col.GPIO_Pin = all_col_pins;
    gpio_col.GPIO_Mode = GPIO_Mode_Out_PP; 
    gpio_col.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_col);
    
    //Pull all of the col pins to LOW
    GPIO_ResetBits(GPIOA, all_col_pins);
}

void Keypad_EXTI_Init(void) {

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);

    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // Phát hi?n c?nh xu?ng
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitTypeDef NVIC_InitStruct;
    
    uint32_t keypad_irq_priority = 5; 

    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = keypad_irq_priority;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;


    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&NVIC_InitStruct);
}
char Keypad_Scan(void) {
    uint8_t row, col;   
    for (row = 0; row < 4; row++) {
        GPIO_ResetBits(GPIOA, row_pins[row]);       
        for (col = 0; col < 4; col++) {
            if (GPIO_ReadInputDataBit(GPIOA, col_pins[col]) == Bit_RESET) {
                delay_ms(20); 
                if (GPIO_ReadInputDataBit(GPIOA, col_pins[col]) == Bit_RESET) {
                    char key = keypad_keys[row][col];
                    while (GPIO_ReadInputDataBit(GPIOA, col_pins[col]) == Bit_RESET);                    
                    GPIO_SetBits(GPIOA, row_pins[row]);
                    return key;
                }
            }
        }        
        GPIO_SetBits(GPIOA, row_pins[row]);
    }
    return 0; 
}