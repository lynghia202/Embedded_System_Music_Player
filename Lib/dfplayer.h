#ifndef __DFPLAYER_H
#define __DFPLAYER_H

#include "stm32f10x.h"

/**
 * @brief Kh?i t?o UART dùng cho DFPlayer.
 * @param usart: Con tr? t?i USART (ví d?: USART2)
 */
void DFPlayer_Init(USART_TypeDef* usart);

/**
 * @brief G?i m?t l?nh chung t?i DFPlayer.
 * @param cmd: M? l?nh (ví d?: 0x0C cho Play, 0x0E cho Pause)
 * @param data: D? li?u 16-bit (ví d?: s? th? t? bài hát)
 */
void DFPlayer_SendCommand(uint8_t cmd, uint16_t data);

// Các hàm ti?n ích
void DFPlayer_PlayTrack(uint16_t trackNum);
void DFPlayer_Pause(void);
void DFPlayer_Resume(void);
void DFPlayer_Next(void);
void DFPlayer_Previous(void);
void DFPlayer_SetVolume(uint8_t volume); // 0-30
void DFPlayer_VolumeUp(void);
void DFPlayer_VolumeDown(void);

#endif // __DFPLAYER_H