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

// به جای اینکلود کردن هدر، این خطوط را اضافه کن:
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

#if CONFIG_MFN_V1
#if CONFIG_S8
    FaceRecognition112V1S8 *recognizer = new FaceRecognition112V1S8();
#elif CONFIG_S16
    FaceRecognition112V1S16 *recognizer = new FaceRecognition112V1S16();
#endif
#endif

    show_state_t frame_show_state = SHOW_STATE_IDLE;
    recognizer_state_t _gEvent = DETECT;
    
    int32_t             _gDeleteId       = -1;
    int                 idle_timeout     = 0;
    int                 frame_count      = 0;
    TickType_t          idle_announce_tick      = 0;
    TickType_t          recognize_announce_tick = 0;

    recognizer->set_thresh(0.75);
    int part_ret = recognizer->set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");
    int restor_ret = recognizer->set_ids_from_flash();
    ESP_LOGE("ENROLL", "thresh %f partitoin %d ids %d", recognizer->get_thresh(),part_ret,restor_ret);
      
    while (true)
    {
        // ---- خواندن دستور جاری به‌صورت atomic + منطق timeout ثبت‌نام ----
        xSemaphoreTake(xMutex, portMAX_DELAY);
        if (gEvent == DETECT)
        {
            idle_timeout = 0;
        }
        else
        {
            idle_timeout++;
        }
        if (_gEvent == DETECT && gEvent != DETECT)
        {
            idle_timeout = 0;
        }
        if (_gEvent != ENROLL && gEvent == ENROLL)
        {
            idle_timeout = 0;
        }
        if (idle_timeout > ENROLL_TIMEOUT_FRAMES && _gEvent == ENROLL)
        {
            gEvent       = GOTO_IDLE;
            idle_timeout = 0;
            uart_send_line("timeout");
        }
        
        _gEvent    = gEvent;
        _gDeleteId = gDeleteId;
        xSemaphoreGive(xMutex);

        if (_gEvent == IDLE)
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - idle_announce_tick) >= pdMS_TO_TICKS(1000))
            {
                idle_announce_tick = now;
                uart_send_line("idle");
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (_gEvent == DETECT || _gEvent == RECOGNIZE)
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - recognize_announce_tick) >= pdMS_TO_TICKS(1000))
            {
                recognize_announce_tick = now;
                uart_send_line("recognize");
            }
        }
        if (!xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
        {
            continue;
        }    
    
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
        case RECOGNIZE:   // از نظر پروتکل معادل DETECT: «دنبال یک چهره‌ی شناخته‌شده بگرد»
            if (is_detected)
            {
                recognize_result = recognizer->recognize((uint16_t *)frame->buf,
                                                          {(int)frame->height, (int)frame->width, 3},
                                                          detect_results.front().keypoint);
                print_detection_result(detect_results);

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

        case THRESH_DOWN:
            if (recognizer->get_thresh() > 0.30f)
            {
                recognizer->set_thresh(recognizer->get_thresh() - 0.05f);
            }
            frame_show_state = SHOW_STATE_SHOW_THRESHHOLD;
            break;

        case THRESH_UP:
            if (recognizer->get_thresh() < 0.95f)
            {
                recognizer->set_thresh(recognizer->get_thresh() + 0.05f);
            }
            frame_show_state = SHOW_STATE_SHOW_THRESHHOLD;
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
                    gEvent = GOTO_IDLE;
                    xSemaphoreGive(xMutex);

                    frame_show_state = SHOW_STATE_ENROLL;
                }
            }
            break;

        case DELETE_ALL:
            recognizer->clear_id(true);
            uart_send_line("alldeleted");
            frame_show_state = SHOW_STATE_DELETE_ALL;

            xSemaphoreTake(xMutex, portMAX_DELAY);
            gEvent = GOTO_IDLE;
            xSemaphoreGive(xMutex);
            break;

        case DELETE:
        {
            int deleted = recognizer->delete_id(_gDeleteId, true);
            if (deleted != -1)
            {
                recognize_result.id = _gDeleteId;
                uart_send_linef("remove%d", _gDeleteId);
            }
            else
            {
                recognize_result.id = -1;
            }
            frame_show_state = SHOW_STATE_DELETE;

            xSemaphoreTake(xMutex, portMAX_DELAY);
            gEvent = GOTO_IDLE;
            xSemaphoreGive(xMutex);
            break;
        }

        case GOTO_IDLE:
            xSemaphoreTake(xMutex, portMAX_DELAY);
            gEvent = IDLE;
            xSemaphoreGive(xMutex);
            break;
        default:
            // چیزی برای انجام دادن نیست؛ فریم فقط برای حفظ جریان دوربین/LCD مصرف می‌شود
            break;
        }          
      
        // ---- نمایش نتیجه‌ی آخرین عملیات روی فریم، به مدت چند فریم ----
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

static void task_event_handler(void *arg)
{
    face_cmd_t cmd;
    while (true)
    {
        xQueueReceive(xQueueEvent, &cmd, portMAX_DELAY);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        gEvent    = cmd.cmd;
        gDeleteId = cmd.id;
        xSemaphoreGive(xMutex);
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
