#include "gst_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include  <string.h>

#define ASCII_LOG_I(fmt, args...)   ESP_LOGI("[LOG_ASCII]", fmt, ##args);
#define ASCII_LOG_E(fmt, args...)   ESP_LOGE("[LOG_ASCII ERR]:>>>>>>>>>>>>", fmt, ##args);

/**************************************************
 * 函数功能：将HEX帧转换为字符类型字符串
 * 参数：
 *      输入参数：sou HEX帧 Des 转换得到的字符串  Length 转换长度
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
 * 注：将获取的数值进行转换   例如数值为0X36  需要将3和6单独提取出来 各占一个字节来发送
	如果一个HEX帧为0X74 0XDE，通过串口打印出来是无法看到的，此时调用此函数可得到目标
	字符串74DE，可以直接在串口上打印出来 
**************************************************/
void ASICC2Char(uint8_t *sou,uint8_t *des,uint8_t length)
{
    uint8_t i = 0;
    int8_t high = 0,low = 0;

    for(i = 0;i < length;i++)        
    {
        high = *sou >> 4;
        low = *sou & 0x0f;
        if(high > 9)
            *des = high + 0x57;
        else if((high >= 0) && (high <= 9))          
        *des = high + 0x30;
        
        des++;
        
        if(low > 9)
        *des = low + 0x57;
        else if((low >= 0) && (low <= 9))
        *des = low + 0x30;
        
        sou++;
        des++;
    }
}
#if 1
/**************************************************
 * 函数功能：打印数据为ASCII码
 * 参数：
 *      输入参数：log_data log字符串，ascii_data 要打印的字符串，len 长度
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
void log_ascii(char *log_data,uint8_t *ascii_data,uint8_t len)
{
    void *temp_buf = NULL;
    temp_buf = malloc(len * 2 + 4);
    if(temp_buf == NULL)
    {
        uint8_t temp_data[200] = {0};
        ASICC2Char(ascii_data,temp_data,len);
        ASCII_LOG_E("alloc faill!!!,%s-(len:%d)-%s",log_data,len,temp_data);
    }
    else
    {
        memset(temp_buf,0,len * 2 + 4);
        ASICC2Char(ascii_data,(uint8_t *)temp_buf,len);

        ASCII_LOG_I("%s-(len:%d)-%s",log_data,len,(uint8_t *)temp_buf);

        free(temp_buf);
    }

    ASCII_LOG_I(">>>>>>>>>>Left Statck:[%d],Left Heep:[%d]<<<<<<<<<<",uxTaskGetStackHighWaterMark(NULL),xPortGetFreeHeapSize());
}
#endif