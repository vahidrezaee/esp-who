#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief 
 * 
 * @param key_state_i 
 * @param event_o 
 */
void register_uart( const QueueHandle_t key_state_o,const QueueHandle_t uart_state_o);

// ارسال یک خط متنی به STM32 (به‌صورت خودکار با '\n' خاتمه می‌یابد)
void uart_send_line(const char *str);

// نسخه‌ی printf-style همان تابع، برای پیام‌های دارای عدد مثل "enroll7" یا "grante42"
void uart_send_linef(const char *fmt, ...);