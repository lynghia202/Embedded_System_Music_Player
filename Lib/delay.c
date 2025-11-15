#include "delay.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

void delay_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        TickType_t delayTicks = pdMS_TO_TICKS(ms);
        if (ms > 0 && delayTicks == 0) {
            delayTicks = 1;
        }

        if (delayTicks > 0) {
            vTaskDelay(delayTicks);
        }
    } else {
        
        volatile uint32_t count = ms * (SystemCoreClock / 8 / 1000); 
        for(uint32_t i = 0; i < count; ++i) {
            __NOP(); // No operation
        }
    }
}

void delay_us(uint32_t us) {
    volatile uint32_t count = us * (SystemCoreClock / 8 / 1000000); 
    for(uint32_t i = 0; i < count; ++i) {
        __NOP(); // No operation
    }
}