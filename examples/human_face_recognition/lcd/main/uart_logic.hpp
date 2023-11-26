#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"

/**
 * @brief 
 * 
 * @param key_state_i 
 * @param event_o 
 */
void register_uart( const QueueHandle_t key_state_o);