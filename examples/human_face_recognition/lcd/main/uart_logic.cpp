#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "uart_logic.hpp"
#include "who_human_face_recognition.hpp"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "who_lcd.h"
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (39)
#define RXD_PIN (38)

#define UART UART_NUM_2

int num = 0;

static QueueHandle_t xQueueKeyStateO = NULL;
static QueueHandle_t XUartToLcdStateO = NULL;

void init(void) 
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
/*
static void tx_task(void *arg)
{
	char* Txdata = (char*) malloc(100);
    while (1) {
        // ESP_LOGI(TX_TASK_TAG, "text send");
    	sprintf (Txdata, "Hello world index = %d\r\n", num++);
        uart_write_bytes(UART, Txdata, strlen(Txdata));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}*/

static void rx_task(void *arg)
{
    
    esp_log_level_set("RX", ESP_LOG_INFO);
    char* data = (char*) malloc(RX_BUF_SIZE+1);
    uart_write_bytes(UART, "Ready67890", strlen("Ready67890"));
    ESP_LOGI("RX", "send data Read \n\r");
     int rxBytes;
     int temppp = 0;
    while (1) {
  
       rxBytes = uart_read_bytes(UART, data, RX_BUF_SIZE, 500 / portTICK_PERIOD_MS);
        // ESP_LOGI("RX", "Read %d bytes", rxBytes);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            static recognizer_state_t recognizer_state = IDLE;
            static recognizer_state_t lcd_state = IDLE;
           // ESP_LOGI("RX", "Read %d bytes: '%s'", rxBytes, data);
          //  uart_write_bytes(UART, data, rxBytes);
            if(strcmp(data,"lcd_off890") == 0)
            {
                 ESP_LOGE("RX", "LCD OFF %s\n\r",data);
                 lcd_state = LCD_OFF;
                 xQueueSend(XUartToLcdStateO, &lcd_state, portMAX_DELAY);
                 
            }
            if(strcmp(data,"lcd_on7890") == 0)
            {
                  ESP_LOGE("RX", "LCD ON %s\n\r",data);
                  lcd_state = LCD_ON;
                  xQueueSend(XUartToLcdStateO, &lcd_state, portMAX_DELAY); 
                  
            }
            if(strcmp(data,"enroll7890")==0)
            {// ESP_LOGI("RX", "Read %d bytes: '%s'", rxBytes, data);

                 recognizer_state = ENROLL;
            }
            else if(strcmp(data,"recognize0")==0)
            {
                 recognizer_state = DETECT;
            }
            else if(strcmp(data,"delall7890")==0)
            {
                 recognizer_state = DELETE_ALL;
            }
            else if(strcmp(data,"thresh_up0")==0)
            {
                 recognizer_state = THRESH_UP;
            }
            else if(strcmp(data,"thresh_dwn")==0)
            {
                 recognizer_state = THRESH_DOWN;
            }
            else if(strcmp(data,"goto_ide90")==0)
            {
                 recognizer_state = GOTO_IDLE;
            }
            else{
                char *pos = strstr(data , "delete");
                if (pos != NULL && strlen(data) ==10)
                {
                    char number [5];
                    strcpy(number , data+ strlen("delete")+1 );
                    int num = atoi(number) +(int)(DELETE );
                    recognizer_state = (recognizer_state_t)num  ;
                     ESP_LOGI("RX", "delete %d ", recognizer_state);
                }
            }
            if(recognizer_state != IDLE)
            {
                 xQueueSend(xQueueKeyStateO, &recognizer_state, portMAX_DELAY);
            }
            
        }
    }
    free(data);
}


void register_uart( const QueueHandle_t key_state_o, const QueueHandle_t uart_lcd_state_o)
{
    xQueueKeyStateO = key_state_o;
    XUartToLcdStateO = uart_lcd_state_o;
    init();
    ESP_LOGI("RX", "uart initialize");
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    // ESP_LOGI("RX", "uart rx");
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
    //ESP_LOGI("RX", "uart tx");
}