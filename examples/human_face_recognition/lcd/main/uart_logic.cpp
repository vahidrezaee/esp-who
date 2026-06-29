#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "esp_log.h"
#include "driver/uart.h"

#include "who_human_face_recognition.hpp"



#define UART    UART_NUM_2
#define TXD_PIN (39)
#define RXD_PIN (38)



#define UART_RING_BUF_SIZE  256     // بافر حلقوی داخلی درایور
#define LINE_BUF_SIZE       64      // طولانی‌ترین خط پروتکل + حاشیه‌ی امن
#define EVENT_QUEUE_LEN     10
#define LINE_TERMINATOR     '\n'

static const char *TAG = "UART";

static QueueHandle_t xQueueKeyStateO  = NULL;
static QueueHandle_t XUartToLcdStateO = NULL;
static QueueHandle_t xUartEventQueue  = NULL;

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
    uart_driver_install(UART, UART_RING_BUF_SIZE, 0, EVENT_QUEUE_LEN, &xUartEventQueue, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


    // هر بار بایت '\n' دیده شود یک رویداد UART_PATTERN_DET صادر می‌شود؛
    // این دقیقاً همان نقشی را دارد که idle-line detection سمت STM32 دارد،
    // با این تفاوت که به‌جای «سکوت روی خط» یک کاراکتر پایان‌دهنده‌ی صریح داریم.
    // توجه: نام این تابع در نسخه‌های مختلف ESP-IDF فرق دارد
    // (در v4.x: uart_enable_pattern_det_baud_intr). اگر کامپایل نشد، نام دقیق
    // را برای IDF خودتان در driver/uart.h چک کنید.
    uart_enable_pattern_det_baud_intr(UART, LINE_TERMINATOR, 1, 9, 0, 0);
    uart_pattern_queue_reset(UART, EVENT_QUEUE_LEN);

}
extern "C" {
    void uart_send_line(const char *str)
    {
        uart_write_bytes(UART, str, strlen(str));
        uart_write_bytes(UART, "\n", 1);
    }

    void uart_send_linef(const char *fmt, ...)
    {
        char buf[LINE_BUF_SIZE];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        uart_send_line(buf);
        ESP_LOGW("tx", "uart tx:%s\n", buf );
    }
}
// تفسیر یک خط کامل (بدون '\n') که از STM32 رسیده است
static void process_line(char *line)
{
   
    if (strstr(line, "off"))
    {
        recognizer_state_t lcd_state = LCD_OFF;
        xQueueSend(XUartToLcdStateO, &lcd_state, portMAX_DELAY);
        return;
    }
    if (strstr(line, "onL"))
    {
        recognizer_state_t lcd_state = LCD_ON;
        xQueueSend(XUartToLcdStateO, &lcd_state, portMAX_DELAY);
        return;
    }

    face_cmd_t cmd = { IDLE, -1 };

    if      (strstr(line, "enr"))             cmd.cmd = ENROLL;
    else if (strstr(line, "rec"))             cmd.cmd = DETECT;
    else if (strstr(line, "rem"))             cmd.cmd = DELETE_ALL;
    else if (strcmp(line, "thresh_up")  == 0) cmd.cmd = THRESH_UP;
    else if (strcmp(line, "thresh_dwn") == 0) cmd.cmd = THRESH_DOWN;
    else if (strstr(line, "got"))             cmd.cmd = GOTO_IDLE;
    else if (strncmp(line, "del", 3) == 0)
    {
        // قبلاً: atoi(data + strlen("del") + 1) -> این یک off-by-one بود که فقط
        // به‌خاطر تساهل atoi در برابر کاراکتر غیررقمی، تصادفاً درست کار می‌کرد.
        // الان عدد دقیقاً بعد از "del" شروع می‌شود، بدون پدینگ صفر و بدون دنباله‌ی "rrr".
        cmd.cmd = DELETE;
        cmd.id  = atoi(line + 3);
    }
    else
    {
        ESP_LOGW(TAG, "unknown command: '%s'", line);
        return;
    }

    // هر دستوری که اینجا برسد قبلاً غیر IDLE تنظیم شده؛ مستقیماً به صف می‌رود
    xQueueSend(xQueueKeyStateO, &cmd, portMAX_DELAY);
}

static void rx_task(void *arg)
{
    uart_event_t event;
    uint8_t line_buf[LINE_BUF_SIZE];
    
    uart_send_line("Ready");
    while (1)
    {
        if (xQueueReceive(xUartEventQueue, &event, portMAX_DELAY) != pdTRUE)
            continue;
       
        switch (event.type)
        {
        case UART_PATTERN_DET:
        {
            int pos = uart_pattern_pop_pos(UART);
            if (pos < 0)
            {
                // صف موقعیت‌های الگو پر شده بود؛ مرز خط را نمی‌دانیم
                uart_flush_input(UART);
                break;
            }
            if (pos >= LINE_BUF_SIZE - 1)
            {
                // خطی بزرگ‌تر از حد انتظار پروتکل؛ احتمالاً نویز -> دور بریز
                uart_read_bytes(UART, line_buf, pos + 1, pdMS_TO_TICKS(20));
                break;
            }
            int len = uart_read_bytes(UART, line_buf, pos, pdMS_TO_TICKS(20));
            
            uint8_t terminator;
            uart_read_bytes(UART, &terminator, 1, pdMS_TO_TICKS(20)); // مصرف خود '\n'
            if (len > 0)
            {
                line_buf[len] = '\0';
                 ESP_LOGI(TAG, "uart rx:%s\n",line_buf);
                process_line((char *)line_buf);
               
            }
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            ESP_LOGW(TAG, "uart overrun, flushing");
            uart_flush_input(UART);
            xQueueReset(xUartEventQueue);
            break;
        default:
            break;
        }
    }
}
    
   



void register_uart( const QueueHandle_t key_state_o, const QueueHandle_t uart_lcd_state_o)
{
    xQueueKeyStateO = key_state_o;
    XUartToLcdStateO = uart_lcd_state_o;
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES-1, NULL);
     
}