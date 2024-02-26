#include "uart_bus.h"
#include "string.h"
#include "stdint.h"
#include "esp_log.h"

#define UART_MODULE_DEBUG_OUTPUT_ON
#define  ARRAY_SIZE(array)                  ((      sizeof(array)       )/(   sizeof(array[0])    ))     

#define UART_MAX_NUM 2

typedef struct uart_config_struct
{
    uint8_t                                 uart_device_id;
    uint8_t                                 uart_fifo_rx_buffer_size;
    uint8_t                                 uart_fifo_tx_buffer_size;
    uint32_t                                uart_baud_rate;
    uart_word_length_t                      uart_data_bits;
    uart_parity_t                           uart_parity;
    uart_stop_bits_t                        uart_stop_bit;
    uart_hw_flowcontrol_t                   uart_flow_control;
    uart_sclk_t                             uart_clock;
    uint8_t                                 queue_size;
    uint8_t                                 intr_flag;
    int32_t                                 rx_pin;
    int32_t                                 tx_pin;
    int32_t                                 rts_pin;
    int32_t                                 cts_pin;
}uart_user_config_t;

typedef struct uart_info_struct
{
    uart_user_config_t*                     uart_config;
    uart_event_t                            uart_event;
    QueueHandle_t                           uart_queue;
    uart_receive_buffer_public_t*           rx_buffer;
    SemaphoreHandle_t                       rx_sem;
    void                                    (*rx_callback)(void);
}uart_info_t;

typedef struct uart_total_info_struct
{
    uart_user_config_t*                     uart_total_config;
    uart_info_t*                            uart_info[UART_MAX_NUM];
    uint8_t                                 uart_num;
}uart_total_info_t;

static const uart_user_config_t uart_config[] = 
{
    {
        .uart_device_id                     = UART_NUM_1,
        .uart_fifo_rx_buffer_size           = 132,
        .uart_fifo_tx_buffer_size           = 0,
        .uart_baud_rate                     = 115200,
        .uart_data_bits                     = UART_DATA_8_BITS,
        .uart_parity                        = UART_PARITY_DISABLE,
        .uart_stop_bit                      = UART_STOP_BITS_1,
        .uart_flow_control                  = UART_HW_FLOWCTRL_DISABLE,
        .uart_clock                         = UART_SCLK_APB,
        .intr_flag                          = 0,
        .queue_size                         = 20,
        .rx_pin                             = 9,
        .tx_pin                             = 8,
        .rts_pin                            = UART_PIN_NO_CHANGE,
        .cts_pin                            = UART_PIN_NO_CHANGE,
    },
};

static uart_total_info_t uart_total_info;

#ifdef UART_MODULE_DEBUG_OUTPUT_ON
    static const char *TAG = "uart_events";
#else
    #define ESP_LOGI( tag, format, ... )      
#endif

void uart_server_task(void *task_prama)
{
    uart_total_info_t*  this = (uart_total_info_t*)task_prama;
    for(int i=0;i<this->uart_num;i++)
    {
        this->uart_info[i] = (uart_info_t*)malloc(sizeof(uart_info_t));
        memset(this->uart_info[i],0,sizeof(uart_info_t));
        this->uart_info[i]->uart_config = &this->uart_total_config[i];
        uart_info_t* uart_driver = this->uart_info[i];
        uart_config_t uart_use_config = 
        {
            .baud_rate = uart_driver->uart_config->uart_baud_rate,
            .data_bits = uart_driver->uart_config->uart_data_bits,
            .parity = uart_driver->uart_config->uart_parity,
            .stop_bits = uart_driver->uart_config->uart_stop_bit,
            .flow_ctrl = uart_driver->uart_config->uart_flow_control,
            .source_clk = uart_driver->uart_config->uart_clock,
        };
        ESP_LOGI(TAG, "[UART READY TO INSTALL DRIVER]");
        uart_driver_install(uart_driver->uart_config->uart_device_id, 
            uart_driver->uart_config->uart_fifo_rx_buffer_size, 
            uart_driver->uart_config->uart_fifo_tx_buffer_size, 
            uart_driver->uart_config->queue_size, 
            &uart_driver->uart_queue, 
            uart_driver->uart_config->intr_flag);
        ESP_LOGI(TAG, "[UART SET PARAMA CONFIG]");
        uart_param_config(uart_driver->uart_config->uart_device_id, &uart_use_config);
        ESP_LOGI(TAG, "[UART SET PINS]");
        uart_set_pin(uart_driver->uart_config->uart_device_id,
            uart_driver->uart_config->tx_pin, 
            uart_driver->uart_config->rx_pin, 
            uart_driver->uart_config->rts_pin,
            uart_driver->uart_config->cts_pin);
    }
    while(1)
    {
        for(int i=0;i<this->uart_num;i++)
        {
            uart_info_t* uart_driver = this->uart_info[i];
            if(xQueueReceive(uart_driver->uart_queue, (void * )&uart_driver->uart_event, 0x01))
            {
                switch(uart_driver->uart_event.type) 
                {
                    case UART_DATA:
                        ESP_LOGI(TAG, "[UART DATA]: %d", uart_driver->uart_event.size);
                        ESP_LOGI(TAG, "[UART DATA MAX SIZE]: %d", uart_driver->rx_buffer->uart_buffer_max_size);
                        if(uart_driver->uart_event.size > uart_driver->rx_buffer->uart_buffer_max_size)
                        {
                            ESP_LOGI(TAG, "[UART DATA IS OVER FLOW]: %d", uart_driver->uart_event.size);
                            uart_flush_input(uart_driver->uart_config->uart_device_id);
                        }
                        else
                        {
                            if(NULL != uart_driver->rx_buffer->uart_receive_buffer)
                            {
                                uart_read_bytes(uart_driver->uart_config->uart_device_id, uart_driver->rx_buffer->uart_receive_buffer, uart_driver->uart_event.size, 200);
                                uart_driver->rx_buffer->uart_receive_size = uart_driver->uart_event.size;
                                if(NULL != uart_driver->rx_sem)
                                {
                                    // BaseType_t err;
                                    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                                    // err = xSemaphoreGiveFromISR(uart_driver->rx_sem,&xHigherPriorityTaskWoken);
                                    // if (err != pdTRUE)// check if semaphore give fails
                                    // {
                                    //     ESP_LOGI(TAG, "[UART DATA GIVE SEM FAILD]");
                                    // }
                                    // portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
                                }
                                if(NULL != uart_driver->rx_callback)
                                {
                                    uart_driver->rx_callback();
                                }
                            }
                        }
                        break;
                    case UART_FIFO_OVF:
                        ESP_LOGI(TAG, "hw fifo overflow");
                        uart_flush_input(uart_driver->uart_config->uart_device_id);
                        xQueueReset(uart_driver->uart_queue);
                        break;
                    //Event of UART ring buffer full
                    case UART_BUFFER_FULL:
                        ESP_LOGI(TAG, "ring buffer full");
                        uart_flush_input(uart_driver->uart_config->uart_device_id);
                        xQueueReset(uart_driver->uart_queue);
                        break;
                    default:
                        break;
                }
            } 
        }
    }
}

uint32_t uart_set_receive_buffer(uint8_t device_id,uart_receive_buffer_public_t* buffer,SemaphoreHandle_t xSemaphore,void (*rx_callback)(void))
{
    uart_total_info.uart_info[device_id]->rx_buffer = buffer;
    uart_total_info.uart_info[device_id]->rx_sem = xSemaphore;
    uart_total_info.uart_info[device_id]->rx_callback = rx_callback;
    return 0;
}

uint32_t uart_send(uint8_t device_id,uint8_t* data,uint8_t size)
{
    return uart_write_bytes(uart_total_info.uart_info[device_id]->uart_config->uart_device_id,data,size);
}

void uart_init(void)
{
    memset(&uart_total_info,0,sizeof(uart_total_info_t));
    uart_total_info.uart_num  = ARRAY_SIZE(uart_config);
    uart_total_info.uart_total_config = (uart_user_config_t*)&uart_config;
    xTaskCreate(uart_server_task, "uart_server_task", 4096, (void*)&uart_total_info, 3, NULL);
}

