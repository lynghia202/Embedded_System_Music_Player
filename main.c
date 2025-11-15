#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>  
#include <stdlib.h> 
#include "string.h"
#include "lcd_i2c.h"
#include "keypad_4x4.h"
#include "dfplayer.h"

// ==================== HARDWARE CONFIG ====================
#define DFPLAYER_USART         USART1
#define DFPLAYER_USART_RCC     RCC_APB2Periph_USART1 
#define DFPLAYER_TX_PIN        GPIO_Pin_9  
#define DFPLAYER_RX_PIN        GPIO_Pin_10 
#define DFPLAYER_GPIO_PORT     GPIOA
#define DFPLAYER_GPIO_RCC      RCC_APB2Periph_GPIOA

// ==================== APP CONFIG ====================
#define MAX_TRACK_NUMBER       999
#define DFPLAYER_CMD_DELAY_MS  120
#define KEYPAD_DEBOUNCE_MS     80
#define LCD_UPDATE_RATE_MS     300
#define IDLE_TIMEOUT_MS        30000

// ==================== FREERTOS OBJECTS ====================
static QueueHandle_t g_keypadQueue = NULL;
static QueueHandle_t g_lcdQueue = NULL;
static SemaphoreHandle_t g_i2cMutex = NULL;
static SemaphoreHandle_t g_uartMutex = NULL;
static SemaphoreHandle_t g_keypadSemaphore = NULL;

static TickType_t lastActivityTime = 0;
static volatile uint8_t isLcdBacklightOn = 1; // volatile for thread safety

// ==================== DATA STRUCTURES ====================
typedef enum {
    PLAYER_STOPPED,
    PLAYER_PLAYING,
    PLAYER_PAUSED
} PlayerState_t;

typedef struct {
    uint8_t volume;
    uint16_t currentTrack;
    PlayerState_t state;
} PlayerStatus_t;

typedef struct {
    char line1[17];
    char line2[17];
} LcdMessage_t;

// ==================== FUNCTION PROTOTYPES ====================
static void prvSetupHardware(void);
static void vKeypadScanTask(void *pvParameters);
static void vLcdDisplayTask(void *pvParameters);
static void vPlayerLogicTask(void *pvParameters);
static void vIdleMonitorTask(void *pvParameters);
static void SendToLcd(const char* line1, const char* line2);
static void DisableAllKeypadIRQ(void);
static void EnableAllKeypadIRQ(void);
static void SafeDFPlayer_Next(void);
static void SafeDFPlayer_Previous(void);
static void SafeDFPlayer_Pause(void);
static void SafeDFPlayer_Resume(void);
static void SafeDFPlayer_PlayTrack(uint16_t track);
static void SafeDFPlayer_SetVolume(uint8_t volume);

// ==================== LCD DISPLAY TASK ====================
static void vLcdDisplayTask(void *pvParameters) {
    LcdMessage_t msg;
    char prevLine1[17] = {0};
    char prevLine2[17] = {0};

    if (xSemaphoreTake(g_i2cMutex, portMAX_DELAY) == pdPASS) {
        LCD_Init();
        LCD_Clear();
        xSemaphoreGive(g_i2cMutex);
    }
    
    for (;;) {
        if (xQueueReceive(g_lcdQueue, &msg, pdMS_TO_TICKS(LCD_UPDATE_RATE_MS)) == pdPASS) {
            
            if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdPASS) {
                
                if (strcmp(prevLine1, msg.line1) != 0) {
                    LCD_SetCursor(0, 0);
                    LCD_Print("                ");
                    LCD_SetCursor(0, 0);
                    LCD_Print(msg.line1);
                    strncpy(prevLine1, msg.line1, 16);
                }
                
                if (strcmp(prevLine2, msg.line2) != 0) {
                    LCD_SetCursor(1, 0);
                    LCD_Print("                ");
                    LCD_SetCursor(1, 0);
                    LCD_Print(msg.line2);
                    strncpy(prevLine2, msg.line2, 16);
                }
                
                xSemaphoreGive(g_i2cMutex);
            }
        }
    }
}

// ==================== SEND TO LCD ====================
static void SendToLcd(const char* line1, const char* line2) {
    LcdMessage_t msg;
    
    strncpy(msg.line1, line1, 16);
    msg.line1[16] = '\0';
    
    strncpy(msg.line2, line2, 16);
    msg.line2[16] = '\0';
    
    xQueueOverwrite(g_lcdQueue, &msg);
    lastActivityTime = xTaskGetTickCount();
}

// ==================== SAFE DFPLAYER COMMANDS ====================
static void SafeDFPlayer_PlayTrack(uint16_t track) {
    if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS) {
        DFPlayer_PlayTrack(track);
        xSemaphoreGive(g_uartMutex);
        vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
    }
}

static void SafeDFPlayer_SetVolume(uint8_t volume) {
    if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS) {
        DFPlayer_SetVolume(volume);
        xSemaphoreGive(g_uartMutex);
        vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
    }
}

static void SafeDFPlayer_Next(void) {
    if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS) {
        DFPlayer_Next();
        xSemaphoreGive(g_uartMutex);
        vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
    }
}

static void SafeDFPlayer_Previous(void) {
    if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS) {
        DFPlayer_Previous();
        xSemaphoreGive(g_uartMutex);
        vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
    }
}

static void SafeDFPlayer_Pause(void) {
    if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS) {
        DFPlayer_Pause();
        xSemaphoreGive(g_uartMutex);
        vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
    }
}

static void SafeDFPlayer_Resume(void) {
    if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS) {
        DFPlayer_Resume();
        xSemaphoreGive(g_uartMutex);
        vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
    }
}

// ==================== PLAYER LOGIC TASK ====================
static void vPlayerLogicTask(void *pvParameters) {
    char key;
    char trackBuffer[4] = {0};
    uint8_t bufferIndex = 0;
    uint8_t needsDelayBeforeUpdate = 0;
    
    PlayerStatus_t status = {
        .volume = 15,
        .currentTrack = 1,
        .state = PLAYER_STOPPED
    };
		SendToLcd("STM32 Player", "Booting...");
		vTaskDelay(pdMS_TO_TICKS(2000)); 
    SafeDFPlayer_SetVolume(status.volume);
    SendToLcd("STM32 Player", "Ready...");
		vTaskDelay(pdMS_TO_TICKS(1500)); 
    char line1[17], line2[17];
    snprintf(line1, 17, "Track: %03d", status.currentTrack);
    snprintf(line2, 17, "Vol:%02d State:---", status.volume);
    SendToLcd(line1, line2);

    for (;;) {
        if (xQueueReceive(g_keypadQueue, &key, portMAX_DELAY) == pdPASS) {
            
            lastActivityTime = xTaskGetTickCount();
            needsDelayBeforeUpdate = 0;
            
            if (isLcdBacklightOn == 0) {
                continue;
            }

            if (bufferIndex > 0) {
                // ====== CH? Ð? CH?N BÀI ======
                
                switch (key) {
                    case '0': case '1': case '2': case '3': case '4':
                    case '5': case '6': case '7': case '8': case '9':
                        if (bufferIndex < 3) {
                            trackBuffer[bufferIndex++] = key;
                            trackBuffer[bufferIndex] = '\0';
                            
                            snprintf(line1, 17, "Select: %s_", trackBuffer);
                            snprintf(line2, 17, "D=Play, C=Back");
                            SendToLcd(line1, line2);
                        }
                        break;
                    
                    case 'D':
                        {
                            uint16_t trackNum = atoi(trackBuffer);
                            if (trackNum > 0 && trackNum <= MAX_TRACK_NUMBER) {
                                status.currentTrack = trackNum;
                                SafeDFPlayer_PlayTrack(status.currentTrack);
                                status.state = PLAYER_PLAYING;
                                
                                snprintf(line1, 17, "Play T:%03d", status.currentTrack);
                                SendToLcd(line1, "State: PLAYING");
                                needsDelayBeforeUpdate = 1;
                            } else {
                                SendToLcd("Invalid Track", "1-999 only");
                                needsDelayBeforeUpdate = 1;
                            }
                            
                            bufferIndex = 0;
                            memset(trackBuffer, 0, sizeof(trackBuffer));
                        }
                        break;

                    case 'C': // BACKSPACE - FIXED
                        if (bufferIndex > 0) { // Ki?m tra ð? tránh underflow
                            bufferIndex--;
                            trackBuffer[bufferIndex] = '\0';
                            
                            if (bufferIndex > 0) {
                                snprintf(line1, 17, "Select: %s_", trackBuffer);
                                snprintf(line2, 17, "D=Play, C=Back");
                                SendToLcd(line1, line2);
                            } else {
                                SendToLcd("Selection", "Cancelled");
                                needsDelayBeforeUpdate = 1;
                            }
                        }
                        break;
                        
                    default:
                        SendToLcd("Invalid key...", "D=Play, C=Back");
                        vTaskDelay(pdMS_TO_TICKS(300));
                        snprintf(line1, 17, "Select: %s_", trackBuffer);
                        snprintf(line2, 17, "D=Play, C=Back");
                        SendToLcd(line1, line2);
                        break;
                }

            } else {
                // ====== CH? Ð? PHÁT NH?C ======
                
                switch (key) {
                    case '0': case '1': case '2': case '3': case '4':
                    case '5': case '6': case '7': case '8': case '9':
                        bufferIndex = 0;
                        trackBuffer[bufferIndex++] = key;
                        trackBuffer[bufferIndex] = '\0';
                        
                        snprintf(line1, 17, "Select: %s_", trackBuffer);
                        snprintf(line2, 17, "D=Play, C=Back");
                        SendToLcd(line1, line2);
                        break;
                    
                    case 'A':
                        if (status.currentTrack < MAX_TRACK_NUMBER) status.currentTrack++;
                        else status.currentTrack = 1;
                        SafeDFPlayer_Next();
                        status.state = PLAYER_PLAYING;
                        snprintf(line1, 17, "Track: %03d", status.currentTrack);
                        SendToLcd(line1, "State: NEXT");
                        needsDelayBeforeUpdate = 1;
                        break;

                    case 'B':
                        if (status.currentTrack > 1) status.currentTrack--;
                        else status.currentTrack = MAX_TRACK_NUMBER;
                        SafeDFPlayer_Previous();
                        status.state = PLAYER_PLAYING;
                        snprintf(line1, 17, "Track: %03d", status.currentTrack);
                        SendToLcd(line1, "State: PREV");
                        needsDelayBeforeUpdate = 1;
                        break;

                    case 'C':
                        if (status.state == PLAYER_PLAYING) {
                            SafeDFPlayer_Pause();
                            status.state = PLAYER_PAUSED;
                            SendToLcd("State: PAUSED", "Press C resume");
                            needsDelayBeforeUpdate = 1;
                        } else if (status.state == PLAYER_PAUSED) {
                            SafeDFPlayer_Resume();
                            status.state = PLAYER_PLAYING;
                            SendToLcd("State: PLAYING", "");
                            needsDelayBeforeUpdate = 1;
                        }
                        break;

                    case 'D':
                        SafeDFPlayer_PlayTrack(status.currentTrack);
                        status.state = PLAYER_PLAYING;
                        SendToLcd("Replaying...", "");
                        needsDelayBeforeUpdate = 1;
                        break;

                    case '#':
                        if (status.volume < 30) {
                            status.volume++;
                            SafeDFPlayer_SetVolume(status.volume);
                            snprintf(line2, 17, "Volume: %02d/30", status.volume);
                            SendToLcd("Volume UP", line2);
                            needsDelayBeforeUpdate = 1;
                        }
                        break;

                    case '*':
                        if (status.volume > 0) {
                            status.volume--;
                            SafeDFPlayer_SetVolume(status.volume);
                            snprintf(line2, 17, "Volume: %02d/30", status.volume);
                            SendToLcd("Volume DOWN", line2);
                            needsDelayBeforeUpdate = 1;
                        }
                        break;

                    default:
                        break;
                }
            }
            
            // Delay n?u c?n
            if (needsDelayBeforeUpdate) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            
            // C?p nh?t màn h?nh chính
            if (bufferIndex > 0) {
                snprintf(line1, 17, "Select: %s_", trackBuffer);
                snprintf(line2, 17, "D=Play, C=Back");
                SendToLcd(line1, line2);
            } else {
                const char* stateStr = (status.state == PLAYER_PLAYING) ? "PLAY" :
                                       (status.state == PLAYER_PAUSED) ? "PAUS" : "STOP";
                
                snprintf(line1, 17, "T:%03d %s V:%02d", status.currentTrack, stateStr, status.volume);
                snprintf(line2, 17, "A>B<C|| D=Play");
                SendToLcd(line1, line2);
            }
        }
    }
}

// ==================== KEYPAD SCAN TASK ====================
static void DisableAllKeypadIRQ(void) {
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI1_IRQn);
    NVIC_DisableIRQ(EXTI2_IRQn);
    NVIC_DisableIRQ(EXTI3_IRQn);
}

static void EnableAllKeypadIRQ(void) {
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
}

static void vKeypadScanTask(void *pvParameters) {
    char key;
    
    for (;;) {
        if (xSemaphoreTake(g_keypadSemaphore, portMAX_DELAY) == pdPASS) {
            
            DisableAllKeypadIRQ();
            vTaskDelay(pdMS_TO_TICKS(KEYPAD_DEBOUNCE_MS));
            
            Keypad_Configure_For_Scan();
            key = Keypad_Scan();
            Keypad_Configure_For_Interrupt();
            
            EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3);
            EnableAllKeypadIRQ();
            
            if (key != 0) {
                xQueueSend(g_keypadQueue, &key, 0);
            }
        }
    }
}

// ==================== IDLE MONITOR TASK ====================
static void vIdleMonitorTask(void *pvParameters) {
    TickType_t currentTime;
    
    for (;;) {
        currentTime = xTaskGetTickCount();
        
        if ((currentTime - lastActivityTime) > pdMS_TO_TICKS(IDLE_TIMEOUT_MS)) {
            if (isLcdBacklightOn) {
                if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdPASS) {
                    LCD_BacklightOff();// Turn off backlight
                    xSemaphoreGive(g_i2cMutex);
                    isLcdBacklightOn = 0;
                }
            }
        } else {
            if (!isLcdBacklightOn) {
                if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdPASS) {
                   LCD_BacklightOn(); // Turn on backlight
                    xSemaphoreGive(g_i2cMutex);
                    isLcdBacklightOn = 1;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== HARDWARE SETUP ====================
static void prvSetupHardware(void) {
    RCC_APB2PeriphClockCmd(DFPLAYER_GPIO_RCC | RCC_APB2Periph_GPIOA | 
                           RCC_APB2Periph_GPIOB | DFPLAYER_USART_RCC | 
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = DFPLAYER_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(DFPLAYER_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = DFPLAYER_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DFPLAYER_GPIO_PORT, &GPIO_InitStructure);
    
    DFPlayer_Init(DFPLAYER_USART);

    Keypad_Configure_For_Interrupt();
    Keypad_EXTI_Init();
}

// ==================== INTERRUPT HANDLERS ====================
static void Keypad_EXTI_Handler(uint32_t EXTI_Line, IRQn_Type IRQn) {
    if (EXTI_GetITStatus(EXTI_Line) != RESET) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        xSemaphoreGiveFromISR(g_keypadSemaphore, &xHigherPriorityTaskWoken);
        NVIC_DisableIRQ(IRQn);
        EXTI_ClearITPendingBit(EXTI_Line);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void EXTI0_IRQHandler(void) {
    Keypad_EXTI_Handler(EXTI_Line0, EXTI0_IRQn);
}

void EXTI1_IRQHandler(void) {
    Keypad_EXTI_Handler(EXTI_Line1, EXTI1_IRQn);
}

void EXTI2_IRQHandler(void) {
    Keypad_EXTI_Handler(EXTI_Line2, EXTI2_IRQn);
}

void EXTI3_IRQHandler(void) {
    Keypad_EXTI_Handler(EXTI_Line3, EXTI3_IRQn);
}

// ==================== MAIN ====================
int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    prvSetupHardware();

    g_keypadQueue = xQueueCreate(10, sizeof(char));
    g_lcdQueue = xQueueCreate(1, sizeof(LcdMessage_t));
    g_i2cMutex = xSemaphoreCreateMutex();
    g_uartMutex = xSemaphoreCreateMutex();
    g_keypadSemaphore = xSemaphoreCreateBinary();

    configASSERT(g_keypadQueue && g_lcdQueue && g_i2cMutex && 
                 g_uartMutex && g_keypadSemaphore);

    lastActivityTime = xTaskGetTickCount();

    xTaskCreate(vPlayerLogicTask, "Logic", 512, NULL, 2, NULL);
    xTaskCreate(vLcdDisplayTask, "LCD", 256, NULL, 1, NULL);
    xTaskCreate(vKeypadScanTask, "Keypad", 128, NULL, 3, NULL);
    xTaskCreate(vIdleMonitorTask, "Idle", 128, NULL, 1, NULL);

    vTaskStartScheduler();
    for (;;);
}
// ==================== FREERTOS HOOKS ====================
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // Disable interrupts and halt
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationMallocFailedHook(void) {
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationIdleHook(void) {
    // Enter sleep mode when idle
    __WFI();
}