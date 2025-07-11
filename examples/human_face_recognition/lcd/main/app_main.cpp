#include "who_camera.h"
#include "who_human_face_recognition.hpp"
#include "who_lcd.h"
#include "who_button.h"
#include "event_logic.hpp"
#include "who_adc_button.h"
#include "uart_logic.hpp"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueLCDFrame = NULL;
static QueueHandle_t xQueueKeyState = NULL;
static QueueHandle_t xQueueEventLogic = NULL;
static QueueHandle_t xQueueUartToLcdState = NULL;

static button_adc_config_t buttons[4] = {{1, 2800, 3000}, {2, 2250, 2450}, {3, 300, 500}, {4, 850, 1050}};

#define GPIO_BOOT GPIO_NUM_0

extern "C" void app_main()
{
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueKeyState = xQueueCreate(1, sizeof(int *));
    xQueueUartToLcdState = xQueueCreate(1, sizeof(int *));
   // xQueueEventLogic = xQueueCreate(1, sizeof(int *));

    register_uart(xQueueKeyState,xQueueUartToLcdState);
    //register_button(GPIO_BOOT, xQueueKeyState);
    register_camera(PIXFORMAT_RGB565,/*FRAMESIZE_QCIF*/  /*FRAMESIZE_HVGA*/ FRAMESIZE_240X240, 2, xQueueAIFrame);
    // register_adc_button(buttons, 4, xQueueKeyState);
   // register_event(xQueueKeyState, xQueueEventLogic);
    register_human_face_recognition(xQueueAIFrame, xQueueKeyState, NULL, xQueueLCDFrame, false);
  //  app_lcd_draw_wallpaper();
    register_lcd(xQueueLCDFrame, NULL, true,xQueueUartToLcdState);
    //app_lcd_draw_wallpaper();
   

}
