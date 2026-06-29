#include "who_human_face_recognition.hpp"

#include <cstring>
#include <cstdarg>


#include "esp_log.h"
#include "esp_camera.h"

#include "dl_image.hpp"
#include "fb_gfx.h"

#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "face_recognition_tool.hpp"

#if CONFIG_MFN_V1
#if CONFIG_S8
#include "face_recognition_112_v1_s8.hpp"
#elif CONFIG_S16
#include "face_recognition_112_v1_s16.hpp"
#endif
#endif

#include "who_ai_utils.hpp"

// functions in  uart_logic.cpp
extern "C" {
    void uart_send_line(const char *str);
    void uart_send_linef(const char *fmt, ...);
}

using namespace dl;

static const char *TAG = "human_face_recognition";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static SemaphoreHandle_t xMutex = NULL;

static recognizer_state_t gEvent = DETECT;
static int32_t            gDeleteId  = -1;   // فقط وقتی gEvent == DELETE معتبر است



static bool gReturnFB = true;
static face_info_t recognize_result;

typedef enum
{
    SHOW_STATE_IDLE,
    SHOW_STATE_DELETE,
    SHOW_STATE_RECOGNIZE,
    SHOW_STATE_ENROLL,
    SHOW_STATE_SHOW_THRESHHOLD,
    SHOW_STATE_DELETE_ALL,
} show_state_t;

#define RGB565_MASK_RED         0xF800
#define RGB565_MASK_GREEN       0x07E0
#define RGB565_MASK_BLUE        0x001F
#define FRAME_DELAY_NUM         52
#define ENROLL_TIMEOUT_FRAMES   200


static void rgb_print(camera_fb_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(camera_fb_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(loc_buf, sizeof(loc_buf), format, args);
    va_end(args);
  
   if (len < 0)
    {
        return 0;
    }

    if (len >= (int)sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
        va_start(args, format);
        vsnprintf(temp, len + 1, format, args);
        va_end(args);
    }

    rgb_print(fb, color, temp);

    if(temp != loc_buf)
    {
        free(temp);
    }

    return len;
}

static void task_process_handler(void *arg)
{

  //  uart_init();
    camera_fb_t *frame = NULL;
    HumanFaceDetectMSR01 detector(0.3F, 0.3F, 10, 0.3F);
    HumanFaceDetectMNP01 detector2(0.4F, 0.3F, 10);

size_t heap_before = esp_get_free_heap_size();
int64_t t0 = esp_timer_get_time();

#if CONFIG_MFN_V1
#if CONFIG_S8
    FaceRecognition112V1S8 *recognizer = new FaceRecognition112V1S8();
#elif CONFIG_S16
    FaceRecognition112V1S16 *recognizer = new FaceRecognition112V1S16();
#endif
#endif


int64_t t1 = esp_timer_get_time();
size_t heap_after1 = esp_get_free_heap_size();

    show_state_t frame_show_state = SHOW_STATE_IDLE;
    recognizer_state_t _gEvent = DETECT;
    recognizer_state_t last_stable_event = DETECT;
    
    int32_t             _gDeleteId       = -1;
    int                 frame_count      = 0;
    TickType_t          idle_announce_tick      = 0;
    TickType_t          recognize_announce_tick = 0;

// زمان‌سنجی برای منطق تایم‌اوت‌ها
    TickType_t          state_changed_tick = xTaskGetTickCount();
    TickType_t          enroll_timeout_tick = xTaskGetTickCount();

    recognizer->set_thresh(0.75);
    int part_ret = recognizer->set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");
    int restor_ret = recognizer->set_ids_from_flash();
    ESP_LOGE("esp-WHO", "thresh %f partitoin %d ids %d", recognizer->get_thresh(),part_ret,restor_ret);
    #if CONFIG_S8
    ESP_LOGE("esp-WHO", " recognizer set 8 bit quantities\n");
    #elif CONFIG_S16
    ESP_LOGE("esp-WHO", " recognizer set 16 bit quantities\n");
     #endif
    while (true)
    {
        // ---- بخش اتمیک مانیتورینگ وضعیت و مدیریت زمان‌سنجی ----
        xSemaphoreTake(xMutex, portMAX_DELAY);
        
        // اگر وضعیت تغییر کرده باشد، مبدا زمان‌سنجی‌ها را بروزرسانی کن
        if (gEvent != last_stable_event)
        {
            state_changed_tick = xTaskGetTickCount();
            if (gEvent == ENROLL)
            {
                enroll_timeout_tick = xTaskGetTickCount(); // شروع تایمر ۲۰ ثانیه‌ای ثبت‌نام
            }
            last_stable_event = gEvent;
        }
        // تایم‌اوت مهم جدید: اگر سیستم در حالت ENROLL باشد و پس از ۲۰ ثانیه کسی نیاید
        if (gEvent == ENROLL)
        {
            if ((xTaskGetTickCount() - enroll_timeout_tick) >= pdMS_TO_TICKS(20000))
            {
                gEvent = IDLE; // هدایت خودکار به حالت استراحت
                uart_send_line("timeout");
                state_changed_tick = xTaskGetTickCount();
                last_stable_event = IDLE;
            }
        }
        // شرط قبلی: اگر سیستم بیش از ۲ دقیقه (۱۲۰ ثانیه) در منو یا حالتی غیر از تشخیص چهره بماند
        if (gEvent != DETECT && gEvent != RECOGNIZE)
        {
            if ((xTaskGetTickCount() - state_changed_tick) >= pdMS_TO_TICKS(120000))
            {
                gEvent = DETECT; // بازگشت خودکار به مود تشخیص اصلی برای باز کردن درب
                state_changed_tick = xTaskGetTickCount();
                last_stable_event = DETECT;
                uart_send_line("timeout_to_detect");
            }
        }
        _gEvent    = gEvent;
        _gDeleteId = gDeleteId;
        xSemaphoreGive(xMutex);
       
       if (_gEvent == IDLE)
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - idle_announce_tick) >= pdMS_TO_TICKS(10000))
            {
                idle_announce_tick = now;
                uart_send_line("idle");
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (_gEvent == DETECT || _gEvent == RECOGNIZE)
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - recognize_announce_tick) >= pdMS_TO_TICKS(10000))
            {
                recognize_announce_tick = now;
                uart_send_line("recognize");
            }
        }
       // ========== پردازش فرامینی که به فریم نیاز ندارند ==========
        if (_gEvent == DELETE || _gEvent == DELETE_ALL ||
            _gEvent == THRESH_UP || _gEvent == THRESH_DOWN ||
            _gEvent == GOTO_IDLE)
        {
            camera_fb_t *drain_frame = NULL;
            if (xQueueReceive(xQueueFrameI, &drain_frame, 0) == pdTRUE)
            {
                if (xQueueFrameO)
                    xQueueSend(xQueueFrameO, &drain_frame, 0);
                else if (gReturnFB)
                    esp_camera_fb_return(drain_frame);
                else
                    free(drain_frame);
            }

            switch (_gEvent)
            {
            case THRESH_DOWN:
                if (recognizer->get_thresh() > 0.30f)
                    recognizer->set_thresh(recognizer->get_thresh() - 0.05f);
                frame_show_state = SHOW_STATE_SHOW_THRESHHOLD;
                xSemaphoreTake(xMutex, portMAX_DELAY);
                if (gEvent == THRESH_DOWN) { gEvent = IDLE; }
                xSemaphoreGive(xMutex);
                break;

            case THRESH_UP:
                if (recognizer->get_thresh() < 0.95f)
                    recognizer->set_thresh(recognizer->get_thresh() + 0.05f);
                frame_show_state = SHOW_STATE_SHOW_THRESHHOLD;
                xSemaphoreTake(xMutex, portMAX_DELAY);
                if (gEvent == THRESH_UP) { gEvent = IDLE; }
                xSemaphoreGive(xMutex);
                break;

            case DELETE_ALL:
                // شرط دوم: عملیات پاک کردن به صورت کامل و بلاکینگ انجام می‌شود و نیمه‌کاره نمی‌ماند
                recognizer->clear_id(true);
                uart_send_line("alldeleted");
                frame_show_state = SHOW_STATE_DELETE_ALL;
                
                xSemaphoreTake(xMutex, portMAX_DELAY);
                if (gEvent == DELETE_ALL) { gEvent = IDLE; }
                xSemaphoreGive(xMutex);
                break;

            case DELETE:
            {
                // شرط دوم: حذف ID به صورت کامل انجام می‌گیرد و با فرامینی مثل gotoidle در وسط کار خراب نمی‌شود
                int deleted = recognizer->delete_id(_gDeleteId, true);
                if (deleted != -1)
                {
                    recognize_result.id = _gDeleteId;
                    uart_send_linef("remove%d", _gDeleteId);
                }
                else
                {
                    recognize_result.id = -1;
                    uart_send_linef("remove-1"); 
                }
                frame_show_state = SHOW_STATE_DELETE;
                
                xSemaphoreTake(xMutex, portMAX_DELAY);
                if (gEvent == DELETE) { 
                    gEvent = IDLE;
                     }
                xSemaphoreGive(xMutex);
                break;
            }
            case GOTO_IDLE:
                xSemaphoreTake(xMutex, portMAX_DELAY);
                if (gEvent == GOTO_IDLE)   
                {
                    gEvent = IDLE;
                }
                xSemaphoreGive(xMutex);
                break;
            default: break;
            }
            continue; 
        }
        if (xQueueReceive(xQueueFrameI, &frame, pdMS_TO_TICKS(200)) != pdTRUE)
        {
            continue;
        }
int64_t t2 = esp_timer_get_time();
size_t heap_after2 = esp_get_free_heap_size();
        bool is_detected = false;
        std::list<dl::detect::result_t> &detect_candidates =
            detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
        std::list<dl::detect::result_t> &detect_results =
            detector2.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_candidates);

        if (detect_results.size() == 1)
        {
            is_detected = true;
        }
        switch (_gEvent)
        {
        case DETECT:
        case RECOGNIZE:   
            if (is_detected)
            {
                int64_t t3 = esp_timer_get_time();
                size_t heap_after3 = esp_get_free_heap_size();
                recognize_result = recognizer->recognize((uint16_t *)frame->buf,
                                                          {(int)frame->height, (int)frame->width, 3},
                                                          detect_results.front().keypoint);
                print_detection_result(detect_results);
                int64_t t4 = esp_timer_get_time();
                size_t heap_after4 = esp_get_free_heap_size();


                ESP_LOGI("BENCH", "RAM used by model: %d bytes", (int)(heap_before - heap_after4));
                ESP_LOGI("BENCH", "Inference time: %lld ms", (t4 - t0) / 1000);
                ESP_LOGI("BENCH", "Free heap now: %d", (int)heap_after4);
                if (recognize_result.id > 0)
                {
                    uart_send_linef("grante%d", recognize_result.id);
                }
                else
                {
                    uart_send_line("who");
                }
                frame_show_state = SHOW_STATE_RECOGNIZE;
            }
            break;
        case ENROLL:
            if (is_detected)
            {
                int enrolled_id = recognizer->enroll_id((uint16_t *)frame->buf,
                                                         {(int)frame->height, (int)frame->width, 3},
                                                         detect_results.front().keypoint, "", true);
                if (enrolled_id > 0)
                {
                    uart_send_linef("enroll%d", enrolled_id);

                    xSemaphoreTake(xMutex, portMAX_DELAY);
                    if (gEvent == ENROLL) { gEvent = IDLE; }
                    xSemaphoreGive(xMutex);

                    frame_show_state = SHOW_STATE_ENROLL;
                }
            }
            break;
        default:
            break;
    }
            // ---- نمایش وضعیت موقت گرافیکی روی فریم خروجی ----
        if (frame_show_state != SHOW_STATE_IDLE)
        {
            switch (frame_show_state)
            {
            case SHOW_STATE_SHOW_THRESHHOLD:
                rgb_printf(frame, RGB565_MASK_GREEN, "Threshhold: %d", (int)(recognizer->get_thresh() * 100));
                break;
            case SHOW_STATE_DELETE_ALL:
                rgb_printf(frame, RGB565_MASK_RED, "ALL IDs Deleted");
                break;
            case SHOW_STATE_DELETE:
                if (recognize_result.id == -1)
                    rgb_printf(frame, RGB565_MASK_RED, "ID not found");
                else
                    rgb_printf(frame, RGB565_MASK_RED, "%d ID deleted", recognize_result.id);
                break;
            case SHOW_STATE_RECOGNIZE:
                if (recognize_result.id > 0)
                    rgb_printf(frame, RGB565_MASK_GREEN, "ID %d", recognize_result.id);
                else
                    rgb_print(frame, RGB565_MASK_RED, "Not Recognized");
                break;
            case SHOW_STATE_ENROLL:
                rgb_printf(frame, RGB565_MASK_BLUE, "Enroll: ID %d", recognizer->get_enrolled_ids().back().id);
                break;
            default:
                break;
            }

            if (++frame_count > FRAME_DELAY_NUM)
            {
                frame_count      = 0;
                frame_show_state = SHOW_STATE_IDLE;
            }
        }
        if (detect_results.size())
        {
            #if !CONFIG_IDF_TARGET_ESP32S3
            print_detection_result(detect_results);
            #endif
            draw_detection_result((uint16_t *)frame->buf, frame->height, frame->width, detect_results);
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

        if (xQueueResult && is_detected)
        {
            xQueueSend(xQueueResult, &recognize_result, portMAX_DELAY);
        }    
    
        
    }
}

// تسک مدیریت رویدادها با مکانیسم محافظت و Handshake اتمیک جهت جلوگیری از تداخل فرامین متوالی حذف
static void task_event_handler(void *arg)
{
    face_cmd_t cmd;
    while (true)
    {
        // دریافت دستور جدید از صف یوارت (بدون از دست رفتن داده‌ها)
        xQueueReceive(xQueueEvent, &cmd, portMAX_DELAY);
        
        while (true) 
        {
            xSemaphoreTake(xMutex, portMAX_DELAY);
            
            // اگر تسک اصلی در حال اجرای یک فرمان حساس و یک‌بار مصرف است، صبر کن تا کارش تمام شود و وضعیت IDLE شود
            if (gEvent == DELETE || gEvent == DELETE_ALL || gEvent == THRESH_UP || gEvent == THRESH_DOWN || gEvent == GOTO_IDLE) 
            {
                xSemaphoreGive(xMutex);
                vTaskDelay(pdMS_TO_TICKS(5)); // ۵ میلی‌ثانیه صبر برای پایان عملیات فلش در تسک اصلی
            } 
            else 
            {
                // زمانی که سیستم آزاد شد، دستور بعدی با امنیت کامل اعمال می‌شود
                gEvent    = cmd.cmd;
                gDeleteId = cmd.id;
                xSemaphoreGive(xMutex);
                break;
            }
        }
    }
}

void register_human_face_recognition(const QueueHandle_t frame_i,
                                     const QueueHandle_t event,
                                     const QueueHandle_t result,
                                     const QueueHandle_t frame_o,
                                     const bool camera_fb_return)
{
    esp_log_level_set("detection_result", ESP_LOG_ERROR); // Sets all components to ERROR level
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent  = event;
    xQueueResult = result;
    gReturnFB    = camera_fb_return;
    xMutex       = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(task_process_handler, TAG, 8 * 1024, NULL, 5, NULL, 0);
    if (xQueueEvent)
        xTaskCreatePinnedToCore(task_event_handler, TAG, 2 * 1024, NULL, 5, NULL, 1);
}
