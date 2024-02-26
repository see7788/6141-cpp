#ifndef __YBL_H__
#define __YBL_H__

#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


typedef struct ybl_public_data_struct
{
    uint32_t    id;
    uint8_t     type;
    uint8_t     key;
}ybl_public_data_t;

typedef void (* ybl_receive_call_back_t)(void* user_parama,ybl_public_data_t* buffer);

void ybl_init(void);
void ybl_set_receive_config(SemaphoreHandle_t xSemaphore,ybl_receive_call_back_t receive_call_back,void* call_back_parama);














#endif