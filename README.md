PHÂN CHIA CÔNG VIỆC – BÀI TẬP LỚN HỆ THỐNG NHÚNG
I.	Công việc chung cho tất cả: 
Tất cả cần phải nắm được luồng hoạt động chi tiết sau:
1.	Phím bấm -> kích hoạt Ngắt EXTI.
2.	ISR (Hàm ngắt) -> gửi Semaphore -> đánh thức vKeypadScanTask.
3.	vKeypadScanTask -> đọc phím, gửi vào Queue Phím (g_keypadQueue).
4.	vPlayerLogicTask -> nhận phím từ Queue, xử lý logic.
5.	vPlayerLogicTask -> gửi lệnh cho DFPlayer (qua g_uartMutex).
6.	vPlayerLogicTask -> gửi lệnh hiển thị vào Queue LCD (g_lcdQueue).
7.	vLcdDisplayTask -> nhận lệnh từ Queue, hiển thị (qua g_i2cMutex).
8.	vIdleMonitorTask -> kiểm tra thời gian, tắt/bật đèn nền (qua g_i2cMutex).
II.	Phân công công việc
1.	ĐỨC: Keypad 4x4 & Ngắt
•	Task chính: vKeypadScanTask
•	Thư viện phụ trách: keypad_4x4.c và keypad_4x4.h
•	Những gì cần học và nắm rõ:
1.	STM32 (Phần cứng):
	GPIO: Cách cấu hình chân là Input (kéo lên/kéo xuống) và Output.
	EXTI (Ngắt ngoài): Đây là trọng tâm. Học cách cấu hình ngắt. Hiểu tại sao cần ngắt.
	NVIC (Bộ điều khiển ngắt): Cách NVIC_Init để cho phép ngắt, và tại sao NVIC_DisableIRQ / EnableAllKeypadIRQ lại quan trọng.
2.	FreeRTOS (Khái niệm):
	Binary Semaphore: Hiểu tại sao dùng xSemaphoreCreateBinary.
	ISR & Task: Nắm rõ xSemaphoreGiveFromISR (để dùng trong ngắt) và xSemaphoreTake (để dùng trong task).
	Debounce (Chống nảy phím): Hiểu tại sao vTaskDelay (80ms) là cần thiết.
3.	Thư viện (Keypad):
	Giải thích được logic của Keypad_Configure_For_Scan và Keypad_Configure_For_Interrupt (sự tráo đổi giữa Input/Output của Hàng và Cột).
2.	NAM: Hiển thị & Giao tiếp I2C 
•	Task chính: vLcdDisplayTask và vIdleMonitorTask (vì cả hai cùng đụng đến LCD).
•	Thư viện phụ trách: lcd_i2c.c và lcd_i2c.h
•	Những gì cần học và nắm rõ:
1.	STM32 (Phần cứng):
	Giao thức I2C: Đây là trọng tâm. Học I2C là gì, SCL/SDA là gì, Start/Stop condition, và tại sao cần địa chỉ (0x4E).
	I2C Peripheral: Cách I2C_Init và các hàm I2C_GenerateSTART, I2C_SendData, I2C_CheckEvent.
2.	FreeRTOS (Khái niệm):
	Mutex: Đây là trọng tâm. Hiểu xSemaphoreCreateMutex. Phải giải thích được "Race Condition" là gì và tại sao cả vLcdDisplayTask VÀ vIdleMonitorTask đều phải dùng xSemaphoreTake(g_i2cMutex) trước khi động vào LCD.
	Queue (Hàng đợi): Hiểu xQueueReceive (để nhận LcdMessage_t).
3.	Thư viện (LCD):
	Giải thích cách thư viện gửi lệnh 4-bit (hàm LCD_SendHalf).
	Hiểu LCD_SendCommand (gửi lệnh) và LCD_SendData (gửi ký tự) khác nhau thế nào.
3.	QUANG: Audio & Giao tiếp UART 
•	Task chính: Hỗ trợ vPlayerLogicTask (phần gọi lệnh DFPlayer).
•	Thư viện phụ trách: dfplayer.c và dfplayer.h, 1 phần vPlayerLogicTask, các hàm SAFE DFPLAYER COMMANDS
•	Những gì cần học và nắm rõ:
1.	STM32 (Phần cứng):
	Giao thức UART: Đây là trọng tâm. UART là gì? TX/RX là gì? Baud rate (9600) là gì?
	USART Peripheral: Cách USART_Init và USART_SendData để gửi 1 byte.
2.	FreeRTOS (Khái niệm):
	Mutex: Giống Người 2, đây là trọng tâm. Phải hiểu g_uartMutex và giải thích được tại sao cần các hàm SafeDFPlayer_... (ví dụ: nếu không có Mutex, lệnh "Next" và "VolumeUp" gửi cùng lúc sẽ làm DFPlayer bị "loạn" và không hiểu lệnh).
3.	Thư viện (DFPlayer):
	Giải thích được cấu trúc của một gói tin (packet) lệnh của DFPlayer (Start byte, Command, Checksum, End byte).
	Nắm rõ hàm DFPlayer_SendCmd là hàm cốt lõi để gửi 1 gói tin.
4.	NGHĨA: Thiết kế hệ thống
•	Task chính: vPlayerLogicTask và toàn bộ main.c (sự tích hợp).
•	Thư viện phụ trách: main.c (file chính)
•	Những gì cần học và nắm rõ:
1.	FreeRTOS (Toàn cảnh):
	Task & Priority (Ưu tiên): Đây là trọng tâm. Phải giải thích được tại sao vKeypadScanTask có ưu tiên cao nhất (3), vPlayerLogicTask ưu tiên trung bình (2), và vLcdDisplayTask ưu tiên thấp (1).
	Queues (Hàng đợi): Hiểu rõ xQueueCreate, xQueueSend và xQueueReceive. Đây là "keo dán" chính của dự án.
	Tất cả (Tổng quan): Phải hiểu Mutex, Semaphore là gì và sự khác biệt giữa chúng.
	Idle Hook & WFI: Giải thích được vApplicationIdleHook và __WFI() là gì và nó giúp tiết kiệm năng lượng như thế nào.
2.	STM32 (Tổng quan):
	Hiểu prvSetupHardware cấu hình những gì (bật clock cho GPIOA, GPIOB, AFIO, USART1, I2C1).
3.	Thư viện (main.c):
	Nắm rõ logic của vPlayerLogicTask: nó chờ phím (xQueueReceive), dùng switch...case để quyết định, sau đó phân phối công việc đi (gọi SafeDFPlayer_... và SendToLcd).
	Ghép code của 3 người kia lại và đảm bảo nó chạy đúng.

