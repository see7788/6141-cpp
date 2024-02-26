#include "ybl.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "a7129.h"
#include "string.h"
#include "esp_log.h"

//#define YBL_MODULE_DEBUG_OUTPUT_ON
#define ORIGIN_PACKAGE_MAX_LENGTH   7
#define CRC_PACKAGE_MAX_LENGTH      6

#ifdef YBL_MODULE_DEBUG_OUTPUT_ON
    static const char *TAG = "[YBL_DRIVER]";
#else
    #define ESP_LOGI( tag, format, ... )      
#endif

typedef struct ybl_info_struct
{
    uint8_t                     origin_data[ORIGIN_PACKAGE_MAX_LENGTH];
    uint8_t                     output_data[CRC_PACKAGE_MAX_LENGTH];
    uint8_t                     CRC16_High;
    uint8_t                     CRC16_Low;
    ybl_public_data_t           public_data;
    ybl_receive_call_back_t     receive_call_back;
    void*                       receive_call_back_parama;
    SemaphoreHandle_t           rx_sem;
}ybl_info_t;

ybl_info_t ybl_info;

static const uint8_t CRC16_LookupHigh[16] = 
{
    0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70,
    0x81, 0x91, 0xA1, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1
};

static const uint8_t CRC16_LookupLow[16] = 
{
    0x00, 0x21, 0x42, 0x63, 0x84, 0xA5, 0xC6, 0xE7,
    0x08, 0x29, 0x4A, 0x6B, 0x8C, 0xAD, 0xCE, 0xEF
};

static void CRC16_Init(ybl_info_t* this)
{
    this -> CRC16_High = 0x1D;
    this -> CRC16_Low = 0x0F;
}

static void CRC16_Update4Bits(ybl_info_t* this,uint8_t val)
{
    uint8_t t;
    // Step one, extract the Most significant 4 bits of the CRC register
    t = this -> CRC16_High >> 4;
    // XOR in the Message Data into the extracted bits
    t = t ^ val;
    // Shift the CRC Register left 4 bits
    this -> CRC16_High = (this -> CRC16_High << 4) | (this -> CRC16_Low >> 4);
    this -> CRC16_Low = this -> CRC16_Low << 4;
    // Do the table lookups and XOR the result into the CRC Tables
    this -> CRC16_High = this -> CRC16_High ^ CRC16_LookupHigh[t];
    this -> CRC16_Low = this -> CRC16_Low ^ CRC16_LookupLow[t];
}

static void CRC16_Update(ybl_info_t* this,uint8_t val)
{
    CRC16_Update4Bits(this,val >> 4); // High nibble first
    CRC16_Update4Bits(this,val & 0x0F); // Low nibble
}

static void CRC_test(ybl_info_t* this,uint8_t length)
{
    uint8_t i;
    CRC16_Init(this);
    for(i=0;i<length;i++) //length--> the payload length
    CRC16_Update(this,this->origin_data[i]); //Payload--> the data you send
}

static uint8_t Data_Output(ybl_info_t* this) //数据输出
{
    CRC_test(this,3);

    if((this -> CRC16_High == this->origin_data[3]) && (this -> CRC16_Low == this->origin_data[4]))
    {
        CRC_test(this,6);
        if(this -> CRC16_Low == this->origin_data[6])
        {
            this->output_data[0] = this->origin_data[0];
            this->output_data[1] = this->origin_data[1];
            this->output_data[2] = this->origin_data[5]&0x0F;
            this->output_data[3] = ((this->origin_data[5]&0x3F)>>4)*2;
            this->output_data[4] = (this->origin_data[5]>>6) + 1;
            this->output_data[5] = this->origin_data[2];
            return 1; //接收正确
        }
        else
        {
            return 0; //接收错误
        }
    }
    else
    {
        return 0; //接收错误
    }
}

static void a7129_receive_call_back(void* user_parama,uint8_t* buffer,uint8_t size)
{
    ybl_info_t* this = (ybl_info_t*)user_parama;
    ESP_LOGI(TAG,"receive data from a7129 data");
    memcpy(&this->origin_data,buffer,7);
    if(1 == Data_Output(this))
    {
        memcpy(&this->public_data,this->output_data,6);
        ESP_LOGI(TAG,"[ID:%d][TYPE:%d][KEY:%d]",this->public_data.id,this->public_data.type,this->public_data.key);
        if(NULL != this->rx_sem)
        {
            BaseType_t err;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            err = xSemaphoreGiveFromISR(this->rx_sem,&xHigherPriorityTaskWoken);
            if (err != pdTRUE)// check if semaphore give fails
            {
            }
            portYIELD_FROM_ISR();
        }
        if(NULL != this->receive_call_back)
        {
            this->receive_call_back(this->receive_call_back_parama,&this->public_data);
        }
    }
    else
    {
        ESP_LOGI(TAG,"a7129 data crc check failed");
    }
}

void ybl_init(void)
{
    memset(&ybl_info,0,sizeof(ybl_info_t));
    a7129_set_receive_config(NULL,a7129_receive_call_back,(void*)&ybl_info);
}

void ybl_set_receive_config(SemaphoreHandle_t xSemaphore,ybl_receive_call_back_t receive_call_back,void* call_back_parama)
{
    ybl_info_t* this                = &ybl_info;
    this->rx_sem                    = xSemaphore;
    this->receive_call_back         = receive_call_back;
    this->receive_call_back_parama  = call_back_parama;
}


