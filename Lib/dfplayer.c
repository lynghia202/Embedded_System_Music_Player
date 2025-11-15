#include "dfplayer.h"

// Bi?n c?c b? ð? lýu con tr? USART
static USART_TypeDef* DF_USART;

// Buffer ð? g?i l?nh
static uint8_t g_cmdBuffer[10];

// Hàm g?i 1 byte qua UART
static void DF_WriteByte(uint8_t byte) {
    while (USART_GetFlagStatus(DF_USART, USART_FLAG_TXE) == RESET);
    USART_SendData(DF_USART, byte);
    while (USART_GetFlagStatus(DF_USART, USART_FLAG_TC) == RESET);
}

/**
 * @brief Tính toán Checksum 16-bit cho DFPlayer
 */
static uint16_t DFPlayer_CalculateChecksum(void) {
    uint16_t sum = 0;
    for (int i = 1; i < 7; i++) { // Ch? tính t? Ver -> DataL
        sum += g_cmdBuffer[i];
    }
    return -sum;
}

/**
 * @brief Kh?i t?o UART
 */
void DFPlayer_Init(USART_TypeDef* usart) {
    DF_USART = usart;
    
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(DF_USART, &USART_InitStructure);
    USART_Cmd(DF_USART, ENABLE);
}

/**
 * @brief G?i l?nh hoàn ch?nh
 */
void DFPlayer_SendCommand(uint8_t cmd, uint16_t data) {
    g_cmdBuffer[0] = 0x7E; // Start Byte
    g_cmdBuffer[1] = 0xFF; // Version
    g_cmdBuffer[2] = 0x06; // Length (không tính start/end/checksum)
    g_cmdBuffer[3] = cmd;  // Command Byte
    g_cmdBuffer[4] = 0x00; // Feedback (0x01 = Yêu c?u ph?n h?i, 0x00 = Không)
    g_cmdBuffer[5] = (uint8_t)(data >> 8); // Data High
    g_cmdBuffer[6] = (uint8_t)(data);      // Data Low
    
    uint16_t checksum = DFPlayer_CalculateChecksum();
    g_cmdBuffer[7] = (uint8_t)(checksum >> 8); // Checksum High
    g_cmdBuffer[8] = (uint8_t)(checksum);      // Checksum Low
    
    g_cmdBuffer[9] = 0xEF; // End Byte

    // G?i 10 bytes
    for (int i = 0; i < 10; i++) {
        DF_WriteByte(g_cmdBuffer[i]);
    }
}

// Implement các hàm ti?n ích
void DFPlayer_PlayTrack(uint16_t trackNum) {
    DFPlayer_SendCommand(0x03, trackNum); // 0x03 = Play track
}

void DFPlayer_Pause(void) {
    DFPlayer_SendCommand(0x0E, 0);
}

void DFPlayer_Resume(void) {
    DFPlayer_SendCommand(0x0D, 0);
}

void DFPlayer_Next(void) {
    DFPlayer_SendCommand(0x01, 0);
}

void DFPlayer_Previous(void) {
    DFPlayer_SendCommand(0x02, 0);
}

void DFPlayer_SetVolume(uint8_t volume) {
    if (volume > 30) volume = 30;
    DFPlayer_SendCommand(0x06, volume);
}

void DFPlayer_VolumeUp(void) {
    DFPlayer_SendCommand(0x04, 0);
}

void DFPlayer_VolumeDown(void) {
    DFPlayer_SendCommand(0x05, 0);
}