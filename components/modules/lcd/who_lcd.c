#include "who_lcd.h"
#include "esp_camera.h"
#include <string.h>
#include "logo_en_240x240_lcd.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
typedef enum
{
    IDLE = 0,
    DETECT,
    ENROLL,
    RECOGNIZE,
    DELETE_ALL,
    THRESH_DOWN,
    THRESH_UP,
    GOTO_IDLE,
    LCD_OFF,
    LCD_ON,
    DELETE,
   
} recognizer_state_t;
static const char *TAG = "who_lcd";

static esp_lcd_panel_handle_t panel_handle = NULL;
static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueUartI  = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static bool gReturnFB = true;
static recognizer_state_t gEvent = DETECT;
static void task_process_handler(void *arg)
{
    camera_fb_t *frame = NULL;
    recognizer_state_t _gEvent;
    while (true)
    {
      
            if(gEvent == LCD_OFF)
            {
                gEvent = DETECT;
                ESP_LOGI(TAG, "LCD OFF");
                esp_lcd_panel_disp_on_off(panel_handle, false);   
                gpio_set_level(BOARD_LCD_BL, 1);    
            }
            if(gEvent == LCD_ON)
            {
                gEvent = DETECT;
                ESP_LOGI(TAG, "LCD ON");
                esp_lcd_panel_disp_on_off(panel_handle, true);
                gpio_set_level(BOARD_LCD_BL, 0); 
            }
        
        if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
        {
             // ESP_LOGI(TAG, "Draw image\n");
          //  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0/*(BOARD_LCD_V_RES - frame->height)/2*/, frame->width, frame->height, (uint16_t *)frame->buf);
            int yy, ii;
             for (yy =0 , ii = 0 ;  ii<6 ; yy += 2 , ii++ ){
                esp_lcd_panel_draw_bitmap(panel_handle, 0, yy, frame->width, yy+1, &((uint16_t *)frame->buf)[ii*frame->width]);
                esp_lcd_panel_draw_bitmap(panel_handle, 0, yy+1, frame->width, yy+2, &((uint16_t *)frame->buf)[ii*frame->width]);
            }
           
            for ( ;   ii < 240 -20  ; yy += 1 , ii++ ){
                esp_lcd_panel_draw_bitmap(panel_handle, 0, yy, frame->width, yy+1, &((uint16_t *)frame->buf)[ii*frame->width]);
                if(ii%4 ==0)
                {
                  yy++;
                  esp_lcd_panel_draw_bitmap(panel_handle, 0, yy, frame->width, yy+1, &((uint16_t *)frame->buf)[ii*frame->width]);
                }
               
            }
            
            for ( ;  ii< 240 ; yy += 2 , ii++ ){
                esp_lcd_panel_draw_bitmap(panel_handle, 0, yy, frame->width, yy+1, &((uint16_t *)frame->buf)[ii*frame->width]);
                esp_lcd_panel_draw_bitmap(panel_handle, 0, yy+1, frame->width, yy+2, &((uint16_t *)frame->buf)[ii*frame->width]);
            }
        
            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }
        }
    }
}
static void task_event_handler(void *arg)
{
    recognizer_state_t _gEvent;
    while (true)
    {
        xQueueReceive(xQueueUartI, &(_gEvent), portMAX_DELAY);
        gEvent = _gEvent;
    }
}
esp_err_t register_lcd(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb,const QueueHandle_t uartQueue_i)
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t bus_conf = {
        .sclk_io_num = BOARD_LCD_SCK,
        .mosi_io_num = BOARD_LCD_MOSI,
        .miso_io_num = BOARD_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_conf, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BOARD_LCD_DC,
        .cs_gpio_num = BOARD_LCD_CS,
        .pclk_hz = BOARD_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = BOARD_LCD_CMD_BITS,
        .lcd_param_bits = BOARD_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    // ESP_LOGI(TAG, "Install ST7789 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BOARD_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    esp_lcd_panel_invert_color(panel_handle, true);// Set inversion for esp32s3eye

    // turn on display
    esp_lcd_panel_disp_on_off(panel_handle, true);

    app_lcd_set_color(0x000000);
    vTaskDelay(pdMS_TO_TICKS(200));
    app_lcd_draw_wallpaper();
    vTaskDelay(pdMS_TO_TICKS(200));

    xQueueUartI = uartQueue_i;
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    gReturnFB = return_fb;
    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);

    if (xQueueUartI)
        xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);

    return ESP_OK;
}

void app_lcd_draw_wallpaper()
{
    uint16_t *pixels = (uint16_t *)heap_caps_malloc((logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (NULL == pixels)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
        return;
    }
    memcpy(pixels, logo_en_240x240_lcd, (logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t));
    esp_lcd_panel_draw_bitmap(panel_handle, 0, (BOARD_LCD_V_RES - logo_en_240x240_lcd_height)/2 , logo_en_240x240_lcd_width, logo_en_240x240_lcd_height, (uint16_t *)pixels);
    heap_caps_free(pixels);
}

void app_lcd_set_color(int color)
{
    uint16_t *buffer = (uint16_t *)malloc(BOARD_LCD_H_RES * sizeof(uint16_t));
    if (NULL == buffer)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (size_t i = 0; i < BOARD_LCD_H_RES; i++)
        {
            buffer[i] = color;
        }

        for (int y = 0; y < BOARD_LCD_V_RES; y++)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, BOARD_LCD_H_RES, y+1, buffer);
        }

        free(buffer);
    }
}
