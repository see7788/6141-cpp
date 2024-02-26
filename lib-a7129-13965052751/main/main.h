#ifndef __MAIN_H__
#define __MAIN_H__

//#include "stdio.h"
#include "stdint.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "common.h"

#include "uart_bus.h"

void show_chip_info(void);
void task_exit(void);

#endif 