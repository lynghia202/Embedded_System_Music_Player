#ifndef __KEYPAD_4X4_H
#define __KEYPAD_4X4_H

#include "stm32f10x.h"
char Keypad_Scan(void);
void Keypad_Configure_For_Scan(void);   
void Keypad_Configure_For_Interrupt(void); 
void Keypad_EXTI_Init(void);             

#endif
