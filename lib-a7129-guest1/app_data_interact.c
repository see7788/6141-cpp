#include "app_data_interact.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include <string.h>
#include "gst_log.h"

#define DATA_TASK_PRIORITY  tskIDLE_PRIORITY + 1

#define D_LOG_I(fmt, args...)   ESP_LOGI("[DATA]", fmt, ##args);
#define D_LOG_E(fmt, args...)   ESP_LOGE("[DATA ERR]:>>>>>>>>>>>>", fmt, ##args);

#define APP_BUF_MAX_LEN    255  //最大数据长度
#define QUEUE_MAX_AMOUNT    10  //队列长度

typedef struct{
    uint32_t a_b_len;
    char a_b_buf[APP_BUF_MAX_LEN];
}app_buf_t;

QueueHandle_t app_data_receive_queue;  //数据接收队列

/**************************************************
 * 函数功能：应用数据解析
 * 参数：
 *      输入参数：a_data 数据，a_len 长度
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void app_data_parse(char *a_data,uint32_t len)
{
    /*
        自行添加
    */
}

/**************************************************
 * 函数功能：数据接收任务
 * 参数：
 *      输入参数：无
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void app_data_receive_task(void *args)
{
    D_LOG_I("app_data_receive_task start  >>>>>>>>>>Left Statck:[%d],Left Heep:[%d]<<<<<<<<<<",uxTaskGetStackHighWaterMark(NULL),xPortGetFreeHeapSize());

    app_data_receive_queue = xQueueCreate(QUEUE_MAX_AMOUNT,sizeof(app_buf_t));   //数据队列
    if(app_data_receive_queue == NULL)
    {
        D_LOG_E("app receive queue create fail");
    }

    app_buf_t *rx_wait_parse_buf = NULL;
    rx_wait_parse_buf = malloc(sizeof(app_buf_t));

	if(NULL == rx_wait_parse_buf)
	{
		D_LOG_E("app rx wait parse buf malloc fail");
        return;
	}

    while(1)
    {
        memset((void *)rx_wait_parse_buf,0,sizeof(app_buf_t));
        xQueueReceive(app_data_receive_queue,(void *)rx_wait_parse_buf,portMAX_DELAY);    //等待接收

        if(rx_wait_parse_buf->a_b_len > 0)
        {
            log_ascii("app parse data",(void *)rx_wait_parse_buf->a_b_buf, rx_wait_parse_buf->a_b_len);

            app_data_parse(rx_wait_parse_buf->a_b_buf, rx_wait_parse_buf->a_b_len); //数据解析
        }
        else
        {
            D_LOG_E("app rx wait parse buf enqueue,but empty");
        }
            // vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/**************************************************
 * 函数功能：应用数据发送
 * 参数：
 *      输入参数：a_data 数据，a_len 长度
 *      输出参数：无
 * 结果输出：0 正常，-1 长度过长，-2 内存不足，-3 数据发送失败
 * 编辑者：Guest
**************************************************/
int app_data_send(char *a_data,uint32_t a_len)
{
    if(a_len > APP_BUF_MAX_LEN)
    {
        D_LOG_E("send app data len too long");
        return -1;
    }

    app_buf_t *temp_buf = NULL;
    temp_buf = malloc(sizeof(app_buf_t));
    
	if(NULL == temp_buf)
	{
		D_LOG_E("app send buf malloc fail");
        return -2;
	}

    memset((void *)temp_buf,0,sizeof(app_buf_t));

    memcpy((void *)temp_buf->a_b_buf,a_data,a_len);
    temp_buf->a_b_len = a_len;

    if(xQueueSend(app_data_receive_queue,(void *)temp_buf,0) != pdTRUE)  //失败
    {
        D_LOG_E("send data enqueue fail");
        return -3;
    }

    free(temp_buf);

    return 0;
}


/**************************************************
 * 函数功能：应用数据交互初始化
 * 参数：
 *      输入参数：无
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
void app_data_interact_init(void)
{
    if (pdPASS != xTaskCreate(app_data_receive_task,
                              "app_data_receive_task",
                               2048,
                              NULL,
                              DATA_TASK_PRIORITY,
                              NULL)) {
        D_LOG_E("create app data receive task fail");
    }
    D_LOG_I("app_data_interact_init");        
}

