#pragma once

#include <stdint.h>
#include "esp_log.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define BOARD_LCD_MOSI 47
#define BOARD_LCD_MISO -1
#define BOARD_LCD_SCK 21
#define BOARD_LCD_CS 44
#define BOARD_LCD_DC 40
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL 48
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 320
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
#define LCD_HOST SPI2_HOST
#define GPIO_OUTPUT_PIN_SEL (1ULL<<BOARD_LCD_BL)
#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t register_lcd(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb,const QueueHandle_t uart_i);

    void app_lcd_draw_wallpaper();
    void app_lcd_set_color(int color);

#ifdef __cplusplus
}
#endif
