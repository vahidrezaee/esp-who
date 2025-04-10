#include "who_human_face_recognition.hpp"
#include <string>
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

using namespace std;
using namespace dl;

static const char *TAG = "human_face_recognition";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static recognizer_state_t gEvent = DETECT;
static bool gReturnFB = true;
static face_info_t recognize_result;

SemaphoreHandle_t xMutex;

typedef enum
{
    SHOW_STATE_IDLE,
    SHOW_STATE_DELETE,
    SHOW_STATE_RECOGNIZE,
    SHOW_STATE_ENROLL,
    SHOW_STATE_SHOW_THRESHHOLD,
    SHOW_STATE_DELETE_ALL,
} show_state_t;

#define RGB565_MASK_RED 0xF800
#define RGB565_MASK_GREEN 0x07E0
#define RGB565_MASK_BLUE 0x001F
#define FRAME_DELAY_NUM 52
#define NAME_SPACE "ddi"
#define KEY "I"
#define KEY_em "E"
#define KEY_num "sid"
 #define ESP_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...) do {                                \
        if (unlikely(!(a))) {                                                                              \
            ESP_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);        \
            ret = err_code;                                                                                \
            goto goto_tag;                                                                                 \
        }                                                                                                  \
    } while (0)




#include "driver/uart.h"
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (39)
#define RXD_PIN (38)

#define UART UART_NUM_2

void uart_init(void) 
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
esp_err_t write_faces(
    #if CONFIG_MFN_V1
#if CONFIG_S8
    FaceRecognition112V1S8 *recognizer 
#elif CONFIG_S16
    FaceRecognition112V1S16 *recognizer 
#endif
#endif
)
{
    ESP_LOGI(TAG, "Saving settings");
    //settings_check(&g_sys_param);
    nvs_handle_t my_handle = {0};
   
    esp_err_t err = nvs_open(NAME_SPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {

         std::vector<face_info_t>faces = recognizer-> get_enrolled_ids();
	   int size =  faces.size();
        err = nvs_set_blob(my_handle, KEY_num, &size, sizeof(int)); 
		ESP_LOGE(TAG, "Saving settings size in flash %d\n",size);
       //  for(face_info_t face )
   

        for ( int ii = 0 ; ii<size ; ii++) 
        {
        

			std::string key = KEY+std::to_string(ii);
            err = nvs_set_blob(my_handle, key.c_str(), &(faces[ii].id), sizeof(int));
			if(err == ESP_OK)
			{
				ESP_LOGE(TAG, "face %d saved\n",ii);
				ESP_LOGE(TAG, "%s",  key.c_str() );
				ESP_LOGE(TAG, "Saving id %d  name  \n freature:\n\n",faces[ii].id);
			}
			key = KEY_em+std::to_string(ii);
           // Tensor<float> T_temp = recognizer->get_face_emb(faces[ii].id);
           // T_temp.set_auto_free(false);
          //  ESP_LOGE(TAG, "tesor.size %d \n",T_temp.get_size());
           // ESP_LOGE(TAG, "tesor.auto_free %d \n",T_temp.auto_free);
            
            err = nvs_set_blob(my_handle, key.c_str(), recognizer->get_face_emb(faces[ii].id).get_element_ptr(), sizeof(float)*recognizer->get_face_emb(faces[ii].id).get_size());
			if(err == ESP_OK)
			{
				ESP_LOGI(TAG, "face %d saved\n",ii);
				ESP_LOGI(TAG, "%s",  key.c_str() );
				//ESP_LOGI(TAG, "Saving ems %d  \n", sizeof(float)*T_temp.get_size());
                //ESP_LOGI(TAG, "f=%f \n ;f=%lf", T_temp.get_element_ptr()[0],T_temp.get_element_ptr()[0]);

                
              //  T_temp.print_all();
			}
            else
            {
                ESP_LOGE(TAG, "face %d ems failed error %s\n",ii,esp_err_to_name(err));
            }
        }
		
        err |= nvs_commit(my_handle);
		if(err == ESP_OK)
		{
			 ESP_LOGW(TAG, "\nSaving with no error \n");
		}
        else
        {
             ESP_LOGE(TAG, "\nSaving with error %s \n",esp_err_to_name(err));
        }
	
        nvs_close(my_handle);
		
	
		 ESP_LOGE(TAG, "Saving settings ends");
    }
    return ESP_OK == err ? ESP_OK : ESP_FAIL;
}

esp_err_t read_faces(
    #if CONFIG_MFN_V1
#if CONFIG_S8
    FaceRecognition112V1S8 *recognizer 
#elif CONFIG_S16
    FaceRecognition112V1S16 *recognizer 
#endif
#endif
)
{
    nvs_handle_t my_handle = 0;
    esp_err_t ret = nvs_open(NAME_SPACE, NVS_READONLY, &my_handle);
    if (ESP_ERR_NVS_NOT_FOUND == ret) {
        ESP_LOGW(TAG, "Not found, Set to default");
      //  memcpy(&g_sys_param, &g_default_sys_param, sizeof(sys_param_t));
       // write_faces();
        return ESP_OK;
    }

   // ESP_GOTO_ON_FALSE(ESP_OK == ret, ret,err, TAG, "nvs open failed (0x%x)", ret);
    if(ESP_OK !=ret)
    {
        if (my_handle) {
        nvs_close(my_handle);
    }
    return ret;
    }
        int size ;
        size_t len = sizeof(int);

	 std::vector<FaceID<float>* >faceIDs;
     ret = nvs_get_blob(my_handle, KEY_num, &size,&len);
    if(ESP_OK !=ret)
    {
        if (my_handle) {
        nvs_close(my_handle);}
    }
    ESP_LOGE(TAG,"read size in flash %d\n",size);
    
   
	//std::string name = "vahid";
    for (int ii=0 ; ii<size;ii++)
    {
        int ID_temp;
        Tensor<float>T_temp;
        T_temp.set_exponent(0);
        T_temp.set_shape({512});
		std::string key = KEY+std::to_string(ii);
        len = sizeof(int);
        ret = nvs_get_blob(my_handle,key.c_str(), &ID_temp,&len);
        if(ESP_OK !=ret)
        {
            if (my_handle) {
            nvs_close(my_handle);}
            return ret ;
        }
		ESP_LOGE(TAG,"read face in flash %d",ii);
		ESP_LOGE(TAG," id %d", ID_temp);
        
        float * data = new float[512];
        memset(data,0,512);
		key = KEY_em+std::to_string(ii);
        len = 512*sizeof(float);
		ret = nvs_get_blob(my_handle,key.c_str(), data,&len);
        if(ESP_OK !=ret)
        {	
			ESP_LOGE(TAG,"read em of flash (%s)failed %d key%s len %d\n\n",esp_err_to_name(ret),ii,key.c_str(),len);
				
            if (my_handle) {
            nvs_close(my_handle);}
            return ret ;
        }
		ESP_LOGE(TAG,"read face in flash %d\n\n",ii);
        T_temp.set_element(data,true);
       FaceID<float> * rt = new FaceID<float>(ID_temp,T_temp,"");
	   faceIDs.push_back(rt);
	  // T_temp.print_all();
    }
	

   /// ESP_GOTO_ON_FALSE(ESP_OK == ret, ret, err, TAG, "can't read param");
   

	ESP_LOGE(TAG,"\n\nset_ids %d\n\n",ret);
	ESP_LOGE(TAG,"\n\nset_ids %d\n\n",ret);

//std::size_t const half_size = faceIDs.size() / 2;
//std::vector<FaceID<float> *> split_lo(faceIDs.begin(), faceIDs.begin() + half_size);
//std::vector<FaceID<float> *> split_hi(faceIDs.begin() + half_size, faceIDs.end());
//std::vector<FaceID<float> *> split_lo;
//for (int jj=0 ; jj< faceIDs.size()*4;jj++)
//        split_lo.push_back(faceIDs[jj%faceIDs.size()]);
   int sret = recognizer-> set_ids(faceIDs,false);	
	ESP_LOGE(TAG,"\n\n finishset_ids %d\n\n",sret);
   // sret = recognizer-> set_ids(split_hi,false);	
	ESP_LOGE(TAG,"\n\n finishset_ids %d\n\n",sret);
    ESP_LOGE(TAG,"\nids %d\n",recognizer->get_enrolled_id_num());
	//while(1){
	//	sleep(1);
   
	  vTaskDelay(100);
	   nvs_close(my_handle);
	//}
		 for (auto & fad : faceIDs) 
          {
            std::cout<< "delete face"<<fad->id<<"\n";
			  delete(fad);
		  }
        vTaskDelay(100);
	   nvs_close(my_handle);
   // settings_check(&g_sys_param);
    return ret;
///err:
 //   if (my_handle) {
 //       nvs_close(my_handle);
 //   }
 //   return ret;
}
*/
static void rgb_print(camera_fb_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(camera_fb_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
    }
    vsnprintf(temp, len + 1, format, arg);
    va_end(arg);
    rgb_print(fb, color, temp);
    if (len > 64)
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
    recognizer->set_thresh(0.70);
    int ret = recognizer->set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");
    int partition_result = recognizer->set_ids_from_flash();
    ESP_LOGE("ENROLL", "partitoin %d ids %d",ret,partition_result);
 /*   
	esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		//ESP_LOGE("ENROLL", "flash erased\n",);
		 ESP_LOGE("ENROLL", "partitoin %d ids %d",ret,partition_result);
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	ESP_LOGE("read_face","read_face");
	read_faces(recognizer);
	ESP_LOGE("read_face","\nread_face\n");
    */
	int timeout =0;
     
    while (true)
    {
        if(_gEvent == DETECT && gEvent != DETECT )
        {
            ESP_LOGE("CAM1","cpture old %dnew cmd %d\n",_gEvent,gEvent );
            ESP_LOGI("CAM1","timeout %d",timeout );
        }
        xSemaphoreTake(xMutex, portMAX_DELAY);
        if( gEvent == DETECT )
        {
            timeout = 0;
        }
        if(_gEvent == DETECT && gEvent != DETECT )
        {
            timeout = 0;
        }
        if(gEvent != DETECT )
        {
            timeout++;
        }
        if(timeout > 200 && _gEvent == ENROLL)
        {
            ESP_LOGE("timeout","cpture old %dnew cmd %d\n",_gEvent,gEvent );
            ESP_LOGI("timeout","timeout %d",timeout );
            gEvent = GOTO_IDLE;
            timeout = 0;
            char  data_str[10] = { 't','i','m','e','o','u','t','0','0','0'};  
            uart_write_bytes(UART, data_str, 10);
        }

        _gEvent = gEvent;
        xSemaphoreGive(xMutex);


        if (_gEvent != IDLE )
        {
            bool is_detected = false;

            if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
            {
                
                std::list<dl::detect::result_t> &detect_candidates = detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
                std::list<dl::detect::result_t> &detect_results = detector2.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_candidates);
                //std::string name ;
                if (detect_results.size() == 1)
                {
                    is_detected = true;
                }
             
                    switch (_gEvent)
                    {
                     case DETECT:
                     if(is_detected)
                        {
                            recognize_result = recognizer->recognize((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_results.front().keypoint);
                        // ESP_LOGW("RECOGNIZE","id num %d",recognizer->get_enrolled_id_num());
                            //ESP_LOGW("RECOGNIZE","id[0].id %d naem %s num",recognizer->get_enrolled_ids()[0].id,recognizer->get_enrolled_ids()[0].name.c_str());
                            //recognizer->get_face_emb(recognizer->get_enrolled_ids()[0].id).print_all();
                            
                            print_detection_result(detect_results);
                            if (recognize_result.id > 0){
                                char  data_str[10] = { 'g','r','a','n','t','e','0','0','0','0'};
                                data_str [9] =  '0' + recognize_result.id %10 ;
                                data_str [8] =  '0' + (recognize_result.id /10)%10 ;
                                data_str [7] =  '0' + (recognize_result.id /100)%10 ;
                                data_str [6] =  '0' + (recognize_result.id /1000)%10 ;
                                ESP_LOGE("detect","granted %d",recognize_result.id );
                                uart_write_bytes(UART, data_str, 10);
                               // ESP_LOGI("RECOGNIZE", "Similarity: %f, Match ID: %d", recognize_result.similarity, recognize_result.id);
                            }
                            else{
                                    char  data_str[10] = { 'w','h','o','0','0','0','0','0','0','0'};  
                                    uart_write_bytes(UART, data_str, 10);
                                    ESP_LOGE("detect"," who? %d" );
                                 }
                               
                            frame_show_state = SHOW_STATE_RECOGNIZE;
                        }
                        
                                break;
                    case THRESH_DOWN:
                            if ( recognizer->get_thresh() > 0.30)
                            recognizer->set_thresh(recognizer->get_thresh() - 0.05);
                           // ESP_LOGI("Thresh"," thresh_down %f",recognizer->get_thresh());
                             frame_show_state = SHOW_STATE_SHOW_THRESHHOLD;
                                break;
                    case THRESH_UP:
                            if ( recognizer->get_thresh() < 0.95)
                            recognizer->set_thresh(recognizer->get_thresh() + 0.05);
                            frame_show_state = SHOW_STATE_SHOW_THRESHHOLD;
                          //  ESP_LOGI("Thresh"," thresh_up %f",recognizer->get_thresh());
                                break;
                    case ENROLL:
                        if(is_detected)
                        {
                        //name = "vahid" + std::to_string(ii++);//std::to_string(42 + recognizer->get_enrolled_ids().back().id);
                        auto recognize_result = recognizer->enroll_id((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_results.front().keypoint, "", true);
                        if (recognize_result > 0){
                                string old_str = ""+ recognize_result;
                                auto new_str = std::string(4 - std::min(4, (int)old_str.length()), '0') + old_str;
                                new_str = "enroll" + new_str;
                                uart_write_bytes(UART, new_str.c_str(), new_str.length());
                                
                               
                               
                                xSemaphoreTake(xMutex, portMAX_DELAY);
                                gEvent = DETECT;
                                xSemaphoreGive(xMutex);
                                frame_show_state = SHOW_STATE_ENROLL;
                                ESP_LOGI("enroll" ,"enroll ID: %d",  recognize_result);
                            }
                        
                 //       write_faces(recognizer);
                       /* = recognizer->write_ids_to_flash();
                    //    ESP_LOGE("Wrute", "ret % d ", ret);
                        }
                        break;

                    case RECOGNIZE:
                        if(is_detected)
                        {
                            recognize_result = recognizer->recognize((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_results.front().keypoint);
                        // ESP_LOGW("RECOGNIZE","id num %d",recognizer->get_enrolled_id_num());
                            //ESP_LOGW("RECOGNIZE","id[0].id %d naem %s num",recognizer->get_enrolled_ids()[0].id,recognizer->get_enrolled_ids()[0].name.c_str());
                            //recognizer->get_face_emb(recognizer->get_enrolled_ids()[0].id).print_all();
                            
                            print_detection_result(detect_results);
                            if (recognize_result.id > 0){
                                string old_str = ""+ recognize_result.id;
                                auto new_str = std::string(3 - std::min(3, (int)old_str.length()), '0') + old_str;
                                new_str = "granted" + new_str;
                                uart_write_bytes(UART, new_str.c_str(), new_str.length());
                                ESP_LOGI("RECOGNIZE", "Similarity: %f, Match ID: %d", recognize_result.similarity, recognize_result.id);
                            }
                            */
                         /*   else
                                ESP_LOGE("RECOGNIZE", "Similarity: %f, Match ID: %d", recognize_result.similarity, recognize_result.id);
                            frame_show_state = SHOW_STATE_RECOGNIZE;*/
                        }
                        break;
                    case DELETE_ALL:
                        {
                            auto faces = recognizer->get_enrolled_ids();
                            for (auto & face : faces)
                            {
                                ESP_LOGI("DELALL"," id %d deleted\n",face.id);
                                recognizer->delete_id(face.id,true);
                            }
                         //    ESP_LOGI("DELALL"," thershold %f\n\n",recognizer->get_thresh());
                  //           write_faces(recognizer);
                            
                             frame_show_state = SHOW_STATE_DELETE_ALL;
                             xSemaphoreTake(xMutex, portMAX_DELAY);
                             gEvent = DETECT;                
                             xSemaphoreGive(xMutex);
                            char  data_str[10] = { 'a','l','l','d','e','l','e','t','e','d'};  
                            uart_write_bytes(UART, data_str, 10); 
                             ESP_LOGI("DELALL","Done!");
                        }
                        break;
                    case GOTO_IDLE:
                            break;
                    default:
                        vTaskDelay(10);
                        recognize_result.id = recognizer->delete_id(_gEvent-(int)DELETE,true);
                        if(recognize_result.id != -1)
                        {
                            recognize_result.id = _gEvent-(int)DELETE;
                //            write_faces(recognizer);
                            char  data_str[10] = { 'r','e','m','o','v','e','0','0','0','0'};  
                            data_str [9] =  '0' + recognize_result.id %10 ;
                            data_str [8] =  '0' + (recognize_result.id /10)%10 ;
                            data_str [7] =  '0' + (recognize_result.id /100)%10 ;
                            data_str [6] =  '0' + (recognize_result.id /1000)%10 ;
                                                      
                            uart_write_bytes(UART, data_str, 10);  
                        }
                        ESP_LOGE("DELETE", "id %d del",_gEvent-(int)DELETE);
                        ESP_LOGE("DELETE", "% d IDs left", recognizer->get_enrolled_id_num());
                        frame_show_state = SHOW_STATE_DELETE;
                        xSemaphoreTake(xMutex, portMAX_DELAY);
                        gEvent = DETECT;
                        xSemaphoreGive(xMutex);
                        ESP_LOGE("DELETE", "Done");
                        break;
                       // ret = recognizer->write_ids_to_flash();
                      //  ESP_LOGE("Wrute", "ret % d ", ret);
						
                      //  read_faces(recognizer);
						/*{
							int size;
							size_t len = sizeof(int);
							 nvs_handle_t my_handle = 0;
							esp_err_t err = nvs_open(NAME_SPACE, NVS_READWRITE, &my_handle);
							if (err == ESP_OK) {
							ESP_LOGI(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
							 err = nvs_get_blob(my_handle, KEY_num, &size,&len);
							if(ESP_OK !=err)
							{
								ESP_LOGI(TAG, "Error (%s) reading size!\n", esp_err_to_name(err));
								if (my_handle) {
								nvs_close(my_handle);}
							}
							ESP_LOGE(TAG,"read size in flash %d\n",size);
								for (int ii = 0 ; ii<size ; ii++)
								{
									Tensor<float> T_temp;
                                    T_temp.set_exponent(0);
                                    T_temp.set_shape({512});
									len = sizeof(float)*512;
                                    float * data = new float[512];
                                    memset(data,0,512);
									std::string key = KEY_em+std::to_string(ii);
									esp_err_t err = nvs_get_blob(my_handle,key.c_str(), data,&len);
									if(ESP_OK !=err)
									{
										ESP_LOGE(TAG,"read em of flash(%s) failed %d key%s len %d\n\n", esp_err_to_name(err),ii,key.c_str(),len);
										if (my_handle) {
										nvs_close(my_handle);}
										
									}
                                    std::cout<<data[0]<<"\n";
                                     ESP_LOGE(TAG, "f=%f\nlf=%lf", data[0],data[0]);
                                    T_temp.set_element(data,true);

									ESP_LOGW(TAG,"read em of flash success %d key%s len %d\n\n",ii,key.c_str(),len);
									//T_temp.print_all();
									
								}
							}
							nvs_close(my_handle);
						}*/
                        /*
                        recognizer->delete_id(true);
                        ESP_LOGE("DELETE", "% d IDs left", recognizer->get_enrolled_id_num());
                        frame_show_state = SHOW_STATE_DELETE;
                        break;*/


                    /*default:
                        break;*/
                    }
                

                if (frame_show_state != SHOW_STATE_IDLE)
                {
                    static int frame_count = 0;
                    switch (frame_show_state)
                    {
                    case SHOW_STATE_SHOW_THRESHHOLD:
                        //ESP_LOGE("show th","show thersh %f",recognizer->get_thresh());
                        rgb_printf(frame, RGB565_MASK_GREEN, "Threshhold: %d",(int)(recognizer->get_thresh() *100));
                        break;
                    case SHOW_STATE_DELETE_ALL:
                             rgb_printf(frame, RGB565_MASK_RED, "ALL IDs Deleted");
                             break;
                    case SHOW_STATE_DELETE:
                        if(recognize_result.id == -1)
                            rgb_printf(frame, RGB565_MASK_RED, "ID not found", recognizer->get_enrolled_id_num());
                        else
                            rgb_printf(frame, RGB565_MASK_RED, "%d ID deleted",recognize_result.id  );
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
                        frame_count = 0;
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
}

static void task_event_handler(void *arg)
{
    recognizer_state_t _gEvent;
    while (true)
    {
        xQueueReceive(xQueueEvent, &(_gEvent), portMAX_DELAY);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        gEvent = _gEvent;
        xSemaphoreGive(xMutex);
    }
}

void register_human_face_recognition(const QueueHandle_t frame_i,
                                     const QueueHandle_t event,
                                     const QueueHandle_t result,
                                     const QueueHandle_t frame_o,
                                     const bool camera_fb_return)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent = event;
    xQueueResult = result;
    gReturnFB = camera_fb_return;
    xMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(task_process_handler, TAG, 8 * 1024, NULL, 5, NULL, 0);
    if (xQueueEvent)
        xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
}
