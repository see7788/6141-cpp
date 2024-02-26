#include "string.h"
#include "main.h"
#include "uart_bus.h"
#include "a7129.h"
#include "esp_log.h"
#include "ybl.h"
#include "esp_log.h"

void ybl_receive_call_back(void* user_parama,ybl_public_data_t* buffer)
{
    ESP_LOGI("MAIN","[ID:%d][TYPE:%d][KEY:%d]",buffer->id,buffer->type,buffer->key);
}


void app_main(void)
{
    show_chip_info();
    a7129_init();
    ybl_init();
    ybl_set_receive_config(NULL,ybl_receive_call_back,NULL);
    while(1)
    {
        vTaskDelay(10);
    }
}

