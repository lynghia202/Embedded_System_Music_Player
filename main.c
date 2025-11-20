#include "FreeRTOS.h"
#include "dfplayer.h"
#include "keypad_4x4.h"
#include "lcd_i2c.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f10x.h"
#include "string.h"
#include "task.h"
#include "timers.h"
#include <stdio.h>
#include <stdlib.h>

// ==================== CAU HINH PHAN CUNG (HARDWARE CONFIG) ====================
#define DFPLAYER_USART USART1
#define DFPLAYER_USART_RCC RCC_APB2Periph_USART1
#define DFPLAYER_TX_PIN GPIO_Pin_9
#define DFPLAYER_RX_PIN GPIO_Pin_10
#define DFPLAYER_GPIO_PORT GPIOA
#define DFPLAYER_GPIO_RCC RCC_APB2Periph_GPIOA

// ==================== CAU HINH UNG DUNG (APP CONFIG) ====================
#define MAX_TRACK_NUMBER 999      // Max 999 bai
#define DFPLAYER_CMD_DELAY_MS 120 // Delay giua cac lenh de DF kip xu ly
#define KEYPAD_DEBOUNCE_MS 80     //  Debounce Keypad
#define IDLE_TIMEOUT_MS 30000     // Thoi gian cho 30s truoc khi tat man hinh

// ==================== DOI TUONG FREERTOS ====================
// Queue: Hang doi truyen du lieu giua cac Task (Thread-Safe)
static QueueHandle_t		g_keypadQueue = NULL;
// Nhan phim tu Task Quet ->Gui sang Task Logic static QueueHandle_t g_lcdQueue = NULL;
// Gui tu Task Logic -> LCD

// Mutex: Bao ve tai nguyen dung chung
static SemaphoreHandle_t	g_i2cMutex = NULL;
// Bao ve bus I2C (Tranh Timer va Task LCD cung truy cap)
static SemaphoreHandle_t	g_uartMutex = NULL;
// Bao ve UART (Tranh gui lenh chong cheo len DFPlayer)

// Semaphore: Co bao hieu ngat cho ban phi
static SemaphoreHandle_t	g_keypadSemaphore = NULL;

// Timer: Bo dinh thoi mem (Software Timer), dung de dem nguoc tat den nen
static TimerHandle_t		g_backlightTimer = NULL;

// Bien toan cuc luu trang thai den ne
static volatile uint8_t		isLcdBacklightOn = 1;

// ==================== CAU TRUC DU LIEU ====================
typedef enum
{
	PLAYER_STOPPED, // Nhac dang dung
	PLAYER_PLAYING, // Nhac dang phat
	PLAYER_PAUSED   // Nhac tam dung
}							PlayerState_t;

typedef struct
{
	uint8_t volume;        // Muc am luong
	uint16_t currentTrack; // So thu tu bai
	PlayerState_t state;   // Trang thai hien tai
}							PlayerStatus_t;

typedef struct
{
	char line1[17]; // Dong 1 LCD
	char line2[17]; // Dong 2 LCD
}							LcdMessage_t;

// ==================== FUNCTION PROTOTYPES ====================
static void					prvSetupHardware(void);
static void					vLowPower_ConfigUnusedPins(void);
static void					vKeypadScanTask(void *pvParameters);
static void					vLcdDisplayTask(void *pvParameters);
static void					vPlayerLogicTask(void *pvParameters);
static void					vIdleMonitorTask(void *pvParameters);
static void					SendToLcd(const char *line1, const char *line2);
static void					vBacklightTimerCallback(TimerHandle_t xTimer);
static void					DisableAllKeypadIRQ(void);
static void					EnableAllKeypadIRQ(void);
static void					SafeDFPlayer_Next(void);
static void					SafeDFPlayer_Previous(void);
static void					SafeDFPlayer_Pause(void);
static void					SafeDFPlayer_Resume(void);
static void					SafeDFPlayer_PlayTrack(uint16_t track);
static void					SafeDFPlayer_SetVolume(uint8_t volume);

// ==================== TIMER CALLBACK ====================
// Ham nay duoc RTOS goi tu dong khi Timer dem du 30s
static void	vBacklightTimerCallback(TimerHandle_t xTimer)
{
	// Tat den nen LCD de tiet kiem nang luong
	if (isLcdBacklightOn)
	{
		// Xin quyen truy cap I2C (Mutex) truoc khi gui lenh
		if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdPASS)
		{
			LCD_BacklightOff(); // Tat backlight
			xSemaphoreGive(g_i2cMutex);
			isLcdBacklightOn = 0;
		}
	}
}

// ==================== HAM BAT BACKLIGHT ====================
static void	WakeUpBacklight(void)
{
	// 1. Reset Timer 30s lai tu dau (Reload)
	// Moi lan nguoi dung bam nut, thoi gian dem nguoc se quay lai 30s
	xTimerReset(g_backlightTimer, 0);
	// 2. Neu den nen dang tat thi bat len ngay lap tuc
	if (isLcdBacklightOn == 0)
	{
		if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdPASS)
		{
			LCD_BacklightOn();
			xSemaphoreGive(g_i2cMutex);
			isLcdBacklightOn = 1;
		}
	}
}

// ==================== TASK HIEN THI LCD ====================
static void	vLcdDisplayTask(void *pvParameters)
{
	LcdMessage_t	msg;

	char prevLine1[17] = {0}; // Bo dem luu noi dung cu
	char prevLine2[17] = {0}; // Bo dem luu noi dung cu
	// Khoi tao LCD
	if (xSemaphoreTake(g_i2cMutex, portMAX_DELAY) == pdPASS)
	{
		LCD_Init();
		LCD_Clear();
		xSemaphoreGive(g_i2cMutex);
	}
	for (;;)
	{
		// portMAX_DELAY giup Task "Ngu dong" (Blocked) tuyet doi
		// khi khong co tin nhan trong Queue. Khong ton CPU de kiem tra.
		if (xQueueReceive(g_lcdQueue, &msg, portMAX_DELAY) == pdPASS)
		{
			// Khi co data, xin Mutex de cap nhat man hinh
			if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdPASS)
			{
				// Toi uu hoa: Chi gui lenh I2C neu noi dung thuc su thay doi
				// Giup giam tai cho bus I2C va CPU
				if (strcmp(prevLine1, msg.line1) != 0)
				{
					LCD_SetCursor(0, 0);
					LCD_Print("                "); // Xoa dong cu
					LCD_SetCursor(0, 0);
					LCD_Print(msg.line1);
					strncpy(prevLine1, msg.line1, 16);
				}
				if (strcmp(prevLine2, msg.line2) != 0)
				{
					LCD_SetCursor(1, 0);
					LCD_Print("                "); // Xoa dong cu
					LCD_SetCursor(1, 0);
					LCD_Print(msg.line2);
					strncpy(prevLine2, msg.line2, 16);
				}
				xSemaphoreGive(g_i2cMutex); // Tra Mutex sau khi dung xong
			}
		}
	}
}

// ==================== HAM GUI DU LIEU VAO QUEUE LCD ====================
static void	SendToLcd(const char *line1, const char *line2)
{
	LcdMessage_t	msg;

	strncpy(msg.line1, line1, 16);
	msg.line1[16] = '\0';
	strncpy(msg.line2, line2, 16);
	msg.line2[16] = '\0';
	xQueueOverwrite(g_lcdQueue, &msg);
}

// ==================== CAC HAM GUI LENH DFPLAYER ====================
// Tat ca cac ham nay deu dung Mutex de bao ve cong UART, tranh xung dot du lieu

//	Chon track
static void	SafeDFPlayer_PlayTrack(uint16_t track)
{
	if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS)
	{
		DFPlayer_PlayTrack(track);
		xSemaphoreGive(g_uartMutex);
		vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
	}
}

// Chinh vol
static void	SafeDFPlayer_SetVolume(uint8_t volume)
{
	if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS)
	{
		DFPlayer_SetVolume(volume);
		xSemaphoreGive(g_uartMutex);
		vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
	}
}

// Next bai
static void	SafeDFPlayer_Next(void)
{
	if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS)
	{
		DFPlayer_Next();
		xSemaphoreGive(g_uartMutex);
		vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
	}
}

// Prev bai
static void	SafeDFPlayer_Previous(void)
{
	if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS)
	{
		DFPlayer_Previous();
		xSemaphoreGive(g_uartMutex);
		vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
	}
}

// Pause
static void	SafeDFPlayer_Pause(void)
{
	if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS)
	{
		DFPlayer_Pause();
		xSemaphoreGive(g_uartMutex);
		vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
	}
}

// Resume
static void	SafeDFPlayer_Resume(void)
{
	if (xSemaphoreTake(g_uartMutex, pdMS_TO_TICKS(200)) == pdPASS)
	{
		DFPlayer_Resume();
		xSemaphoreGive(g_uartMutex);
		vTaskDelay(pdMS_TO_TICKS(DFPLAYER_CMD_DELAY_MS));
	}
}

// ==================== TASK LOGIC TRUNG TAM (CORE LOGIC) ====================
static void	vPlayerLogicTask(void *pvParameters)
{
	char			key;
	uint8_t			bufferIndex;
	PlayerStatus_t	status = {.volume;
	char			line1[17], line2[17];
	uint16_t		trackNum;
	const char		*stateStr = (status.state == PLAYER_PLAYING) ? "PLAY" : (status.state == PLAYER_PAUSED) ? "PAUS" : "STOP";
	uint8_t			needsDelayBeforeUpdate;

	status = {.volume = 15, .currentTrack;
	status = {.volume = 15, .currentTrack = 1, .state;
	char trackBuffer[4] = {0}; // Bo dem nhap so bai hat (VD: "123")
	bufferIndex = 0;
	needsDelayBeforeUpdate = 0;
	// Co hieu de giu man hinh thong bao (UX)
	status = {.volume = 15, .currentTrack = 1, .state = PLAYER_STOPPED};
	// Khoi dong he thong
	WakeUpBacklight();
	SendToLcd("STM32 Player", "Booting...");
	vTaskDelay(pdMS_TO_TICKS(2000));
	SafeDFPlayer_SetVolume(status.volume);
	SendToLcd("STM32 Player", "Ready...");
	vTaskDelay(pdMS_TO_TICKS(1500));
	// Hien thi man hinh chinh lan dau
	snprintf(line1, 17, "Track: %03d", status.currentTrack);
	snprintf(line2, 17, "Vol:%02d State:---", status.volume);
	SendToLcd(line1, line2);
	for (;;)
	{
		// Task nay se ngu cho den khi Keypad Task gui phim vao Queue
		if (xQueueReceive(g_keypadQueue, &key, portMAX_DELAY) == pdPASS)
		{
			// 1. Bam nut -> Bat den nen LCD va Reset Timer
			WakeUpBacklight();
			needsDelayBeforeUpdate = 0;
			// ====== LOGIC NHAP SO BAI HAT ======
			if (bufferIndex > 0)
			{
				switch (key)
				{
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
					if (bufferIndex < 3)
					{
						trackBuffer[bufferIndex++] = key;
						trackBuffer[bufferIndex] = '\0';
						snprintf(line1, 17, "Select: %s_", trackBuffer);
						snprintf(line2, 17, "D=Play, C=Back");
						SendToLcd(line1, line2);
					}
					break ;
				case 'D': // Enter
				{
					trackNum = atoi(trackBuffer);
					if (trackNum > 0 && trackNum <= MAX_TRACK_NUMBER)
					{
						status.currentTrack = trackNum;
						SafeDFPlayer_PlayTrack(status.currentTrack);
						status.state = PLAYER_PLAYING;
						snprintf(line1, 17, "Play T:%03d", status.currentTrack);
						SendToLcd(line1, "State: PLAYING");
						needsDelayBeforeUpdate = 1;
					}
					else
					{
						SendToLcd("Invalid Track", "1-999 only");
						needsDelayBeforeUpdate = 1;
					}
					bufferIndex = 0;
					memset(trackBuffer, 0, sizeof(trackBuffer));
				}
				break ;
				case 'C': // BACKSPACE
					if (bufferIndex > 0)
					{
						bufferIndex--;
						trackBuffer[bufferIndex] = '\0';
						if (bufferIndex > 0)
						{
							snprintf(line1, 17, "Select: %s_", trackBuffer);
							snprintf(line2, 17, "D=Play, C=Back");
							SendToLcd(line1, line2);
						}
						else
						{
							SendToLcd("Selection", "Cancelled");
							needsDelayBeforeUpdate = 1;
						}
					}
					break ;
				default: // Cac phim khac khong co tac dung khi dang nhap so
					SendToLcd("Invalid key...", "D=Play, C=Back");
					vTaskDelay(pdMS_TO_TICKS(300));
					snprintf(line1, 17, "Select: %s_", trackBuffer);
					snprintf(line2, 17, "D=Play, C=Back");
					SendToLcd(line1, line2);
					break ;
				}
			}
			//======LOGIC DIEU KHIEN NHAC TRUC TIEP======
			else
			{
				switch (key)
				{
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
					// Bat dau nhap so bai hat
					bufferIndex = 0;
					trackBuffer[bufferIndex++] = key;
					trackBuffer[bufferIndex] = '\0';
					snprintf(line1, 17, "Select: %s_", trackBuffer);
					snprintf(line2, 17, "D=Play, C=Back");
					SendToLcd(line1, line2);
					break ;
				case 'A': // Next
					if (status.currentTrack < MAX_TRACK_NUMBER)
						status.currentTrack++;
					else
						status.currentTrack = 1;
					SafeDFPlayer_Next();
					status.state = PLAYER_PLAYING;
					snprintf(line1, 17, "Track: %03d", status.currentTrack);
					SendToLcd(line1, "State: NEXT");
					needsDelayBeforeUpdate = 1;
					break ;
				case 'B': // Prev
					if (status.currentTrack > 1)
						status.currentTrack--;
					else
						status.currentTrack = MAX_TRACK_NUMBER;
					SafeDFPlayer_Previous();
					status.state = PLAYER_PLAYING;
					snprintf(line1, 17, "Track: %03d", status.currentTrack);
					SendToLcd(line1, "State: PREV");
					needsDelayBeforeUpdate = 1;
					break ;
				case 'C': // Pasue/Resume
					if (status.state == PLAYER_PLAYING)
					{
						SafeDFPlayer_Pause();
						status.state = PLAYER_PAUSED;
						SendToLcd("State: PAUSED", "Press C resume");
						needsDelayBeforeUpdate = 1;
					}
					else if (status.state == PLAYER_PAUSED)
					{
						SafeDFPlayer_Resume();
						status.state = PLAYER_PLAYING;
						SendToLcd("State: PLAYING", "");
						needsDelayBeforeUpdate = 1;
					}
					break ;
				case 'D': // Play/Replay
					SafeDFPlayer_PlayTrack(status.currentTrack);
					status.state = PLAYER_PLAYING;
					SendToLcd("Replaying...", "");
					needsDelayBeforeUpdate = 1;
					break ;
				case '#': // Vol+
					if (status.volume < 30)
					{
						status.volume++;
						SafeDFPlayer_SetVolume(status.volume);
						snprintf(line2, 17, "Volume: %02d/30", status.volume);
						SendToLcd("Volume UP", line2);
						needsDelayBeforeUpdate = 1;
					}
					break ;
				case '*': // Vol-
					if (status.volume > 0)
					{
						status.volume--;
						SafeDFPlayer_SetVolume(status.volume);
						snprintf(line2, 17, "Volume: %02d/30", status.volume);
						SendToLcd("Volume DOWN", line2);
						needsDelayBeforeUpdate = 1;
					}
					break ;
				default:
					break ;
				}
			}
			// Hieu ung UX: Giu man hinh thong bao (VD: "Volume UP") trong 0.5s de nguoi dung kip doc
			if (needsDelayBeforeUpdate)
			{
				vTaskDelay(pdMS_TO_TICKS(500));
			}
			// Ve lai man hinh chinh sau khi xu ly xong
			if (bufferIndex > 0)
			{
				snprintf(line1, 17, "Select: %s_", trackBuffer);
				snprintf(line2, 17, "D=Play, C=Back");
				SendToLcd(line1, line2);
			}
			else
			{
				snprintf(line1, 17, "T:%03d %s V:%02d", status.currentTrack,
					stateStr, status.volume);
				snprintf(line2, 17, "A>B<C|| D=Play");
				SendToLcd(line1, line2);
			}
		}
	}
}

// ==================== TASK QUET PHIM ====================
static void	DisableAllKeypadIRQ(void)
{
	NVIC_DisableIRQ(EXTI0_IRQn);
	NVIC_DisableIRQ(EXTI1_IRQn);
	NVIC_DisableIRQ(EXTI2_IRQn);
	NVIC_DisableIRQ(EXTI3_IRQn);
}

static void	EnableAllKeypadIRQ(void)
{
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
}

static void	vKeypadScanTask(void *pvParameters)
{
	char	key;

	for (;;)
	{
		// Task nay ngu cho den khi co tin hieu Semaphore tu Ngat (ISR)
		if (xSemaphoreTake(g_keypadSemaphore, portMAX_DELAY) == pdPASS)
		{
			DisableAllKeypadIRQ();                         // Tat ngat
			vTaskDelay(pdMS_TO_TICKS(KEYPAD_DEBOUNCE_MS)); // Debounce
			Keypad_Configure_For_Scan();
			// Cau hinh GPIO de quet ma tran
			key = Keypad_Scan(); // Doc phim duoc bam
			Keypad_Configure_For_Interrupt();
			// Cau hinh lai GPIO cho Ngat
			EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3);
			// Xoa co ngat
			EnableAllKeypadIRQ();
			// Gui phim vao Queue cho Task Logic xu ly
			if (key != 0)
			{
				xQueueSend(g_keypadQueue, &key, 0);
			}
		}
	}
}

// ==================== CAU HINH TOI UU HOA NANG LUONG ====================
static void	vLowPower_ConfigUnusedPins(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	uint16_t			UsedPins_A;
	uint16_t			UsedPins_B;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC,
		ENABLE);
	// Chuyen chan ve che do Analog Input (AIN) de ngat mach so dau vao
	// Giup giam dong ro (Leakage Current) va toi uu nang luong
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// Port A: Chi giu lai cac chan dang dung, con lai tat
	UsedPins_A = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & ~UsedPins_A;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Port B: Chi giu lai chan I2C, con lai tat
	UsedPins_B = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & ~UsedPins_B;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// Disable port C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
}

static void	prvSetupHardware(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;

	// 1. Cau hinh chan thua truoc de toi uu nang luong
	vLowPower_ConfigUnusedPins();
	// 2. Disable ngoai vi khong dung den
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1,
		DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_SPI2 | RCC_APB1Periph_USB | RCC_APB1Periph_DAC | RCC_APB1Periph_WWDG,
		DISABLE);
	// 3. Bat Clock cho cac ngoai vi can thiet
	RCC_APB2PeriphClockCmd(DFPLAYER_GPIO_RCC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | DFPLAYER_USART_RCC | RCC_APB2Periph_AFIO,
		ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// 4. UART Config
	GPIO_InitStructure.GPIO_Pin = DFPLAYER_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(DFPLAYER_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = DFPLAYER_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DFPLAYER_GPIO_PORT, &GPIO_InitStructure);
	DFPlayer_Init(DFPLAYER_USART);
	// 5. Keypad Config
	Keypad_Configure_For_Interrupt();
	Keypad_EXTI_Init();
}
// ==================== ISR ====================
static void	Keypad_EXTI_Handler(uint32_t EXTI_Line, IRQn_Type IRQn)
{
	BaseType_t	xHigherPriorityTaskWoken;

	if (EXTI_GetITStatus(EXTI_Line) != RESET)
	{
		xHigherPriorityTaskWoken = pdFALSE;
		// Gui Semaphore de danh thuc Task Quet Phim
		xSemaphoreGiveFromISR(g_keypadSemaphore, &xHigherPriorityTaskWoken);
		// Tam thoi tat ngat de xu ly trong Task
		NVIC_DisableIRQ(IRQn);
		EXTI_ClearITPendingBit(EXTI_Line);
		// Yeu cau chuyen doi ngu canh (Context Switch) ngay lap tuc neu can
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void	EXTI0_IRQHandler(void)
{
	Keypad_EXTI_Handler(EXTI_Line0, EXTI0_IRQn);
}

void	EXTI1_IRQHandler(void)
{
	Keypad_EXTI_Handler(EXTI_Line1, EXTI1_IRQn);
}

void	EXTI2_IRQHandler(void)
{
	Keypad_EXTI_Handler(EXTI_Line2, EXTI2_IRQn);
}

void	EXTI3_IRQHandler(void)
{
	Keypad_EXTI_Handler(EXTI_Line3, EXTI3_IRQn);
}

// ==================== MAIN ====================
int	main(void)
{
	// 1. Cau hinh nhom uu tien ngat (Bat buoc voi STM32 + FreeRTOS)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	// 2. Khoi tao phan cung
	prvSetupHardware();
	// 3. Tao cac doi tuong OS (Queue, Mutex, Semaphore)
	g_keypadQueue = xQueueCreate(10, sizeof(char));
	g_lcdQueue = xQueueCreate(1, sizeof(LcdMessage_t));
	g_i2cMutex = xSemaphoreCreateMutex();
	g_uartMutex = xSemaphoreCreateMutex();
	g_keypadSemaphore = xSemaphoreCreateBinary();
	// Tao Timer: 30s, One-shot (Chay 1 lan roi dung)
	g_backlightTimer = xTimerCreate("Backlight", pdMS_TO_TICKS(IDLE_TIMEOUT_MS), pdFALSE, (void *)0, vBacklightTimerCallback);
	// 4. Kiem tra loi tao doi tuong (Debug)
	configASSERT(g_keypadQueue && g_lcdQueue && g_i2cMutex && g_uartMutex && g_keypadSemaphore && g_backlightTimer);
	// 5. Tao cac Task
	xTaskCreate(vPlayerLogicTask, "Logic", 512, NULL, 2, NULL);
	xTaskCreate(vLcdDisplayTask, "LCD", 256, NULL, 1, NULL);
	xTaskCreate(vKeypadScanTask, "Keypad", 128, NULL, 3, NULL);
	// 6. Bat dau He dieu hanh
	vTaskStartScheduler();
	for (;;);
}
// ==================== FREERTOS HOOKS (DEBUG) ====================
void	vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	// Treo may va ngat he thong neu bi tran bo nho Stack
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

void	vApplicationMallocFailedHook(void)
{
	// Treo may neu thieu RAM (Heap)
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

void	vApplicationIdleHook(void)
{
	// CPU se vao che do Ngu (Sleep) khi khong co Task nao chay de tiet kiem dien
	__WFI();
}
