#ifndef __UART_H__
#define __UART_H__

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

typedef struct uart_receive_buffer_public_struct
{
    uint8_t*                                uart_receive_buffer;
    uint8_t                                 uart_receive_size;
    uint8_t                                 uart_buffer_max_size;
}uart_receive_buffer_public_t;


void uart_init(void);
uint32_t uart_send(uint8_t device_id,uint8_t* data,uint8_t size);
uint32_t uart_set_receive_buffer(uint8_t device_id,uart_receive_buffer_public_t* buffer,SemaphoreHandle_t xSemaphore,void (*rx_callback)(void));

#endif