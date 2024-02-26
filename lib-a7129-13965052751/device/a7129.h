#ifndef __A7129_H__
#define __A7129_H__

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef void (* a7129_receive_call_back_t)(void* user_parama,uint8_t* buffer,uint8_t size);

void a7129_init(void);
void a7129_set_receive_config(SemaphoreHandle_t xSemaphore,a7129_receive_call_back_t receive_call_back,void* call_back_parama);
uint32_t a7129_send_data_block(uint8_t* data,uint8_t size);













#endif