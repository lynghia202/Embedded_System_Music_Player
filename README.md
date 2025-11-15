BÀI TẬP LỚN HỆ THỐNG NHÚNG: MÁY NGHE NHẠC MP3 (STM32 + FreeRTOS)

Đây là dự án xây dựng một máy nghe nhạc MP3 hoàn chỉnh sử dụng vi điều khiển STM32F103C8T6, module DFPlayer Mini, LCD 16x2 (I2C) và Keypad 4x4.

Hệ thống được xây dựng trên nền tảng hệ điều hành thời gian thực (RTOS) FreeRTOS, tập trung vào kiến trúc đa luồng, tối ưu hóa năng lượng và xử lý an toàn (thread-safe).

Tính năng chính

Phát nhạc MP3 từ thẻ nhớ SD (DFPlayer Mini).

Giao diện điều khiển qua Keypad 4x4:

Chọn bài trực tiếp (ví dụ: 123 + Enter).

Điều khiển: Next, Previous, Pause/Resume, Volume Up/Down.

Hỗ trợ Backspace (phím C) khi nhập sai số.

Hiển thị LCD 16x2 (I2C) với tối ưu chống chớp nháy (anti-flicker).

Tối ưu hóa năng lượng:

Sử dụng Ngắt ngoài (EXTI) cho keypad thay vì quét (polling).

CPU tự động vào chế độ Sleep (__WFI()) khi rảnh (sử dụng Idle Hook).

Tự động tắt đèn nền LCD sau 30 giây không hoạt động và "thức dậy" khi nhấn phím.

Kiến trúc RTOS: 4-Task (Keypad, Logic, LCD, Idle) với cơ chế giao tiếp an toàn (Queues, Mutexes, Semaphores).

Sơ đồ kết nối phần cứng

LCD 16x2 (I2C): PB6 (SCL), PB7 (SDA)

Keypad 4x4:

Hàng (Ngắt): PA0 (R1/EXTI0), PA1 (R2/EXTI1), PA2 (R3/EXTI2), PA3 (R4/EXTI3)

Cột (Output): PA4 (C1), PA5 (C2), PA6 (C3), PA7 (C4)

DFPlayer Mini (UART): PA9 (USART1_TX), PA10 (USART1_RX)

Nguồn:

STM32 & LCD: Cấp nguồn qua USB.

DFPlayer: Cấp nguồn 5V ngoài (Adapter 1A/2A).

QUAN TRỌNG: Nối đất chung (Common GND) giữa STM32, LCD, DFPlayer, và Nguồn ngoài.

Kiến trúc hệ thống & Luồng dữ liệu

Hệ thống được thiết kế theo kiến trúc hướng sự kiện. Luồng hoạt động chi tiết như sau:

Input (Keypad): Nhấn phím -> Kích hoạt Ngắt EXTI.

ISR (Ngắt): Hàm ngắt (ISR) chạy -> "Tặng" Semaphore -> Đánh thức vKeypadScanTask.

Task Keypad (Prio 3): vKeypadScanTask chạy -> Đọc phím (đã chống nảy) -> Gửi phím vào Queue Phím (g_keypadQueue).

Task Logic (Prio 2): vPlayerLogicTask (đang chờ) -> Nhận phím từ Queue -> Xử lý logic nghiệp vụ (ví dụ: trackBuffer, status.volume++).

Output (Audio): vPlayerLogicTask -> Lấy g_uartMutex -> Gửi lệnh cho DFPlayer.

Output (Display): vPlayerLogicTask -> Gửi thông điệp hiển thị vào Queue LCD (g_lcdQueue).

Task Display (Prio 1): vLcdDisplayTask -> Nhận thông điệp -> Lấy g_i2cMutex -> Hiển thị lên LCD.

Task Idle (Prio 1): vIdleMonitorTask -> Kiểm tra lastActivityTime -> Lấy g_i2cMutex -> Tắt/Bật đèn nền.

PHÂN CHIA NHIỆM VỤ

Dự án được chia thành 4 module chính cho 4 thành viên.

1. Nghĩa (Trưởng nhóm & Kiến trúc sư hệ thống)

Task chính: vPlayerLogicTask ("Bộ não") và main.c (Tích hợp).

Thư viện phụ trách: main.c, FreeRTOSConfig.h.

Trọng tâm học tập:

FreeRTOS (Toàn cảnh): Phải nắm rõ sự khác biệt giữa Mutex, Queue, và Semaphore.

Kiến trúc: Giải thích được tại sao vKeypadScanTask có ưu tiên cao nhất (3), vPlayerLogicTask (2), và vLcdDisplayTask (1).

Tối ưu hóa: Giải thích vApplicationIdleHook và __WFI() để tiết kiệm năng lượng.

Logic nghiệp vụ: Nắm rõ logic switch...case, bufferIndex (cho Backspace), và logic "nuốt phím" (wake-on-press).

Tích hợp: Ghép code của 3 thành viên và đảm bảo hệ thống chạy đúng.

2. Đức (Chuyên gia Nhập liệu & Ngắt)

Task chính: vKeypadScanTask

Thư viện phụ trách: keypad_4x4.c, keypad_4x4.h

Trọng tâm học tập:

STM32 (Hardware): EXTI (Ngắt ngoài) và NVIC (Bộ điều khiển ngắt). Đây là cốt lõi.

FreeRTOS (Concepts): Binary Semaphore. Nắm rõ xSemaphoreGiveFromISR (trong ngắt) và xSemaphoreTake (trong task).

Logic: Chống nảy phím (Debounce) và logic tráo đổi I/O của Hàng/Cột (Scan vs. Interrupt mode).

3. Nam (Chuyên gia Hiển thị & I2C)

Task chính: vLcdDisplayTask và vIdleMonitorTask.

Thư viện phụ trách: lcd_i2c.c, lcd_i2c.h.

Trọng tâm học tập:

STM32 (Hardware): Giao thức I2C (Địa chỉ, SCL/SDA).

FreeRTOS (Concepts): Mutex (g_i2cMutex). Phải giải thích được "Race Condition" (Tại sao vLcdDisplayTask và vIdleMonitorTask không được chạy I2C cùng lúc).

Logic: Tối ưu chống chớp nháy (Anti-flicker) bằng cách dùng bộ đệm prevLine và strcmp, và thư viện LCD_BacklightOn/Off.

4. Quang (Chuyên gia Âm thanh & UART)

Task chính: Hỗ trợ vPlayerLogicTask (các hàm SafeDFPlayer_...).

Thư viện phụ trách: dfplayer.c, dfplayer.h.

Trọng tâm học tập:

STM32 (Hardware): Giao thức UART (Baud rate 9600).

FreeRTOS (Concepts): Mutex (g_uartMutex). Giải thích được tại sao lệnh "Next" và "VolumeUp" không được gửi xen kẽ (cần SafeDFPlayer_...).

Phần cứng: Định dạng thẻ SD (thư mục mp3, file 0001.mp3) và Xử lý nguồn (cấp nguồn 5V riêng, nối đất chung, tụ bypass).
