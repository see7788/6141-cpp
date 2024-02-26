#include "task_a7129.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"
#include "A7129reg.h"
#include "A7129config.h"
#include "gst_log.h"
#include "app_rf_parse.h"


#define RF_SCS  14
#define RF_SCK  33
#define RF_SDIO  32
#define RF_GIO2  34

#define SCS_H() gpio_set_level(RF_SCS, 1)
#define SCS_L() gpio_set_level(RF_SCS, 0)

#define SCK_H() gpio_set_level(RF_SCK, 1)
#define SCK_L() gpio_set_level(RF_SCK, 0)

#define SDIO_H() gpio_set_level(RF_SDIO, 1)
#define SDIO_L() gpio_set_level(RF_SDIO, 0)

#define RF_LOG_I(fmt, args...)   ESP_LOGI("[RF]", fmt, ##args);
#define RF_LOG_E(fmt, args...)   ESP_LOGE("[RF ERR]:>>>>>>>>>>>>", fmt, ##args);

#define RF_TASK_PRIORITY  tskIDLE_PRIORITY + 1

// static xQueueHandle gpio_evt_queue = NULL;

QueueHandle_t rf_send_queue;  //串口发送数据队列


xSemaphoreHandle rf_tx_done_semaphore = NULL; //发送完成信号量
xSemaphoreHandle rf_received_semaphore = NULL; //接收到数据信号量
xSemaphoreHandle rf_send_mutex_semaphore = NULL;  //RF数据入队互斥信号量


#define RF_BUF_MAX_LEN    64  //RF数据最大长度
#define QUEUE_MAX_AMOUNT    10  //队列长度

typedef struct{
    uint8_t r_b_len;
    uint8_t r_b_buf[RF_BUF_MAX_LEN];
}rf_buf_t;

typedef enum{
    R_S_TX = 0,
    R_S_RX,
}rf_state_t;

static volatile rf_state_t rf_state = R_S_RX;    //默认接收 


// static uint16_t RxCnt;
// static uint32_t Err_ByteCnt;
// static uint32_t Err_BitCnt;
// static uint8_t tmpbuf[64];
static uint8_t fb_ok = 0;
static uint8_t fb = 0;

// const uint8_t BitCount_Tab[16]={0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4};
const uint8_t ID_Tab[8]={0x34,0x75,0xC5,0x8C,0xC7,0x33,0x45,0xE7};   //ID code

/**************************************************
 * 函数功能：GPIO中断
 * 参数：
 *      输入参数：无
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // uint32_t gpio_num = (uint32_t) arg;
    // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(rf_state == R_S_TX)
    {
        xSemaphoreGiveFromISR( rf_tx_done_semaphore, &xHigherPriorityTaskWoken );
    }
    else
    {
        xSemaphoreGiveFromISR( rf_received_semaphore, &xHigherPriorityTaskWoken );
    }
}

/**************************************************
 * 函数功能：a7129引脚初始化
 * 参数：
 *      输入参数：无
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void drv_a7129_pin_init(void)
{
    /*选择IO*/
    gpio_pad_select_gpio(RF_SCS);
    gpio_pad_select_gpio(RF_SCK);
    gpio_pad_select_gpio(RF_SDIO);
    gpio_pad_select_gpio(RF_GIO2);

    /*设置为输出*/
    gpio_set_direction(RF_SCS,GPIO_MODE_OUTPUT);
    gpio_set_direction(RF_SCK,GPIO_MODE_OUTPUT);
    gpio_set_direction(RF_SDIO,GPIO_MODE_OUTPUT);

    /*设置为输入*/
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_HILEVEL;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 1ULL << RF_GIO2;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    // gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RF_GIO2, gpio_isr_handler, (void*) RF_GIO2);
}

/*a7129驱动*/
/*********************************************************************
** Strobe Command
*********************************************************************/
static void StrobeCMD(uint8_t cmd)
{
    uint8_t i = 0;

    SCS_L();
    for (i = 0; i < 8; i++)
    {
        if (cmd & 0x80)
            SDIO_H();
        else
            SDIO_L();

        __asm__ __volatile__("NOP");
        SCK_H();
        __asm__ __volatile__("NOP");
        SCK_L();
        cmd <<= 1;
    }
    SCS_H();
}

/************************************************************************
**  ByteSend
************************************************************************/
static void ByteSend(uint8_t src)
{
    uint8_t i = 0;

    for (i = 0; i < 8; i++)
    {
        if (src & 0x80)
            SDIO_H();
        else
            SDIO_L();

        __asm__ __volatile__("NOP");
        SCK_H();
        __asm__ __volatile__("NOP");
        SCK_L();
        src <<= 1;
    }
}

/************************************************************************
**  ByteRead
************************************************************************/
static uint8_t ByteRead(void)
{
    uint8_t i = 0, tmp = 0;

    gpio_set_direction(RF_SDIO,GPIO_MODE_INPUT);
    gpio_set_pull_mode(RF_SDIO,GPIO_PULLUP_ONLY);

    //read data code
    for (i = 0; i < 8; i++)
    {
        if (gpio_get_level(RF_SDIO))
            tmp = (tmp << 1) | 0x01;
        else
            tmp = tmp << 1;

        SCK_H();
        __asm__ __volatile__("NOP");
        SCK_L();
        __asm__ __volatile__("NOP");
    }

    gpio_set_direction(RF_SDIO,GPIO_MODE_OUTPUT);

    return tmp;
}

/************************************************************************
**  A7129_WriteReg
************************************************************************/
static void A7129_WriteReg(uint8_t address, uint16_t dataWord)
{
    uint8_t i = 0;

    SCS_L();
    address |= CMD_Reg_W;
    for (i = 0; i < 8; i++)
    {
        if (address & 0x80)
            SDIO_H();
        else
            SDIO_L();

        __asm__ __volatile__("NOP");
        SCK_H();
        __asm__ __volatile__("NOP");
        SCK_L();
        address <<= 1;
    }
    __asm__ __volatile__("NOP");

    //send data word
    for (i = 0; i < 16; i++)
    {
        if (dataWord & 0x8000)
            SDIO_H();
        else
            SDIO_L();

        __asm__ __volatile__("NOP");
        SCK_H();
        __asm__ __volatile__("NOP");
        SCK_L();
        dataWord <<= 1;
    }
    SCS_H();
}

/************************************************************************
**  A7129_ReadReg
************************************************************************/
static uint16_t A7129_ReadReg(uint8_t address)
{
    uint8_t i = 0;
    uint16_t tmp = 0;

    SCS_L();
    address |= CMD_Reg_R;
    for (i = 0; i < 8; i++)
    {
        if (address & 0x80)
            SDIO_H();
        else
            SDIO_L();

        __asm__ __volatile__("NOP");
        SCK_H();
        __asm__ __volatile__("NOP");
        SCK_L();
        address <<= 1;
    }
    __asm__ __volatile__("NOP");

    //read data code
    gpio_set_direction(RF_SDIO,GPIO_MODE_INPUT);
    gpio_set_pull_mode(RF_SDIO,GPIO_PULLUP_ONLY);

    for (i = 0; i < 16; i++)
    {
        if (gpio_get_level(RF_SDIO))
            tmp = (tmp << 1) | 0x01;
        else
            tmp = tmp << 1;

        SCK_H();
        __asm__ __volatile__("NOP");
        SCK_L();
        __asm__ __volatile__("NOP");
    }
    SCS_H();
    return tmp;
}

/************************************************************************
**  A7129_WritePageA
************************************************************************/
static void A7129_WritePageA(uint8_t address, uint16_t dataWord)
{
    uint16_t tmp = 0;

    tmp = address;
    tmp = ((tmp << 12) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);
    A7129_WriteReg(PAGEA_REG, dataWord);
}

/************************************************************************
**  A7129_ReadPageA
************************************************************************/
static uint16_t A7129_ReadPageA(uint8_t address)
{
    uint16_t tmp = 0;

    tmp = address;
    tmp = ((tmp << 12) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);
    tmp = A7129_ReadReg(PAGEA_REG);
    return tmp;
}

/************************************************************************
**  A7129_WritePageB
************************************************************************/
static void A7129_WritePageB(uint8_t address, uint16_t dataWord)
{
    uint16_t tmp = 0;

    tmp = address;
    tmp = ((tmp << 7) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);
    A7129_WriteReg(PAGEB_REG, dataWord);
}

#if 0
/************************************************************************
**  A7129_ReadPageB
************************************************************************/
static uint16_t A7129_ReadPageB(uint8_t address)
{
    uint16_t tmp = 0;

    tmp = address;
    tmp = ((tmp << 7) | A7129Config[CRYSTAL_REG]);
    A7129_WriteReg(CRYSTAL_REG, tmp);
    tmp = A7129_ReadReg(PAGEB_REG);
    return tmp;
}
#endif

/************************************************************************
**  WriteID
************************************************************************/
static uint8_t A7129_WriteID(void)
{
    uint8_t i = 0;
    uint8_t d1 = 0, d2 = 0, d3 = 0, d4 = 0;

    SCS_L();
    ByteSend(CMD_ID_W);
    for (i = 0; i < 4; i++)
        ByteSend(ID_Tab[i]);
    SCS_H();

    SCS_L();
    ByteSend(CMD_ID_R);
    d1 = ByteRead();
    d2 = ByteRead();
    d3 = ByteRead();
    d4 = ByteRead();
    SCS_H();

    if ((d1 != ID_Tab[0]) || (d2 != ID_Tab[1]) || (d3 != ID_Tab[2]) || (d4 != ID_Tab[3]))
    {
        return 1;
    }

    return 0;
}

/*********************************************************************
** entry_deep_sleep_mode
*********************************************************************/
static void entry_deep_sleep_mode(void)
{
    StrobeCMD(CMD_RF_RST);                                            //RF reset
    A7129_WriteReg(PIN_REG, A7129Config[PIN_REG] | 0x0800);           //SCMDS=1
    A7129_WritePageA(PM_PAGEA, A7129Config_PageA[PM_PAGEA] | 0x0010); //QDS=1
    StrobeCMD(CMD_SLEEP);                                             //entry sleep mode
    vTaskDelay(1 / portTICK_PERIOD_MS);                                                     //delay 600us for VDD_A shutdown, C load=0.1uF
    StrobeCMD(CMD_DEEP_SLEEP);                                        //entry deep sleep mode
    vTaskDelay(1 / portTICK_PERIOD_MS);                                                    //delay 200us for VDD_D shutdown, C load=0.1uF
}

/*********************************************************************
** wake_up_from_deep_sleep_mode
*********************************************************************/
static void wake_up_from_deep_sleep_mode(void)
{
    StrobeCMD(CMD_STBY); //wake up
    vTaskDelay(2 / portTICK_PERIOD_MS);         //delay 2ms for VDD_D stabilized
    //InitRF();
}

/*********************************************************************
** A7129_POR
*********************************************************************/
static void A7129_POR(void)
{
    //power on only
    vTaskDelay(10 / portTICK_PERIOD_MS);  //for regulator settling time (power on only)

    StrobeCMD(CMD_RF_RST);  //reset A7129 chip
    while (A7129_WriteID()) //check SPI
    {
        StrobeCMD(CMD_RF_RST); //reset A7129 chip
    }
    A7129_WritePageA(PM_PAGEA, A7129Config_PageA[PM_PAGEA] | 0x1000); //STS=1
    vTaskDelay(2 / portTICK_PERIOD_MS); ;

    entry_deep_sleep_mode(); //deep sleep
    vTaskDelay(2 / portTICK_PERIOD_MS); ;
    wake_up_from_deep_sleep_mode(); //wake up

    StrobeCMD(CMD_RF_RST);  //reset A7129 chip
    while (A7129_WriteID()) //check SPI
    {
        StrobeCMD(CMD_RF_RST); //reset A7129 chip
    }
    A7129_WritePageA(PM_PAGEA, A7129Config_PageA[PM_PAGEA] | 0x1000); //STS=1
    vTaskDelay(2 / portTICK_PERIOD_MS); ;
}

/*********************************************************************
** A7129_Config
*********************************************************************/
static uint8_t A7129_Config(void)
{
    uint8_t i = 0;
    uint16_t tmp = 0;

    for (i = 0; i < 8; i++)
        A7129_WriteReg(i, A7129Config[i]);

    for (i = 10; i < 16; i++)
    {
        if ((i == 14) && (fb_ok == 1))
            A7129_WriteReg(i, A7129Config[i] | (1 << 4)); //MIFS=1(Manual)
        else
            A7129_WriteReg(i, A7129Config[i]);
    }

    for (i = 0; i < 16; i++)
        A7129_WritePageA(i, A7129Config_PageA[i]);

    for (i = 0; i < 5; i++)
        A7129_WritePageB(i, A7129Config_PageB[i]);

    //for check
    tmp = A7129_ReadReg(SYSTEMCLOCK_REG);
    if (tmp != A7129Config[SYSTEMCLOCK_REG])
    {
        return 1;
    }

    return 0;
}



/*********************************************************************
** A7129_Cal
*********************************************************************/
static uint8_t A7129_Cal(void)
{
    uint8_t i = 0;
    uint8_t fb_old = 0,  fbcf = 0; //IF Filter  fcd = 0,
    uint8_t  vbcf = 0;          //VCO Current  vb = 0,
    uint8_t  vccf = 0;         //VCO Band  vcb = 0,
    uint16_t tmp = 0;
    uint8_t fb_fail = 0;

    StrobeCMD(CMD_STBY);

    //IF calibration procedure @STB state
    if (fb_ok == 1)
    {
        A7129_WriteReg(MODE_REG, A7129Config[MODE_REG] | 0x0800); //VCO Current Calibration
        do
        {
            tmp = A7129_ReadReg(MODE_REG);
        } while (tmp & 0x0800);
        tmp = (A7129Config[CALIBRATION_REG] & 0xFFE0);
        tmp = tmp | fb | (1 << 4);
        A7129_WriteReg(CALIBRATION_REG, tmp);
    }
    else
    {
        fb_fail = 0;

        for (i = 0; i < 3; i++)
        {
            A7129_WriteReg(MODE_REG, A7129Config[MODE_REG] | 0x0802); //IF Filter & VCO Current Calibration
            do
            {
                tmp = A7129_ReadReg(MODE_REG);
            } while (tmp & 0x0802);

            //for check(IF Filter)
            tmp = A7129_ReadReg(CALIBRATION_REG);
            fb = tmp & 0x0F;
            // fcd = (tmp >> 11) & 0x1F;
            fbcf = (tmp >> 4) & 0x01;

            if ((fb < 4) || (fb > 8))
                fb_fail = 1;
            else
            {
                if (i == 0)
                    fb_old = fb;
                else
                {
                    if (fb != fb_old)
                        fb_fail = 1;
                }
            }

            if ((fbcf) || (fb_fail))
            {
                return 1;
            }
        }
    }

    //for check(VCO Current)
    tmp = A7129_ReadPageA(VCB_PAGEA);
    // vcb = tmp & 0x0F;
    vccf = (tmp >> 4) & 0x01;
    if (vccf)
    {
        return 1;
    }

    //RSSI Calibration procedure @STB state
    A7129_WriteReg(ADC_REG, 0x4C00);                          //set ADC average=64
    A7129_WriteReg(MODE_REG, A7129Config[MODE_REG] | 0x1000); //RSSI Calibration
    do
    {
        tmp = A7129_ReadReg(MODE_REG);
    } while (tmp & 0x1000);
    A7129_WriteReg(ADC_REG, A7129Config[ADC_REG]);

    //VCO calibration procedure @STB state
    A7129_WriteReg(PLL1_REG, A7129Config[PLL1_REG]);
    A7129_WriteReg(PLL2_REG, A7129Config[PLL2_REG]);
    A7129_WriteReg(MODE_REG, A7129Config[MODE_REG] | 0x0004); //VCO Band Calibration
    do
    {
        tmp = A7129_ReadReg(MODE_REG);
    } while (tmp & 0x0004);

    //for check(VCO Band)
    tmp = A7129_ReadReg(CALIBRATION_REG);
    // vb = (tmp >> 5) & 0x07;
    vbcf = (tmp >> 8) & 0x01;
    if (vbcf)
    {
        return 1;
    }

    fb_ok = 1;
    return 0;
}

/*********************************************************************
** InitRF
*********************************************************************/
static uint8_t InitRF(void)
{
    //initial pin
    SCS_H();
    SCK_L();
    SDIO_H();

    vTaskDelay(1 / portTICK_PERIOD_MS);          //delay 1ms for regulator stabilized
    StrobeCMD(CMD_RF_RST); //reset A7129 chip
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (A7129_Config()) //config A7129 chip
        return 1;

    vTaskDelay(1 / portTICK_PERIOD_MS);; //delay 800us for crystal stabilized

    if (A7129_WriteID()) //write ID code
        return 1;

    if (A7129_Cal()) //IF and VCO Calibration
        return 1;

    return 0;
}

/**************************************************
 * 函数功能：RF发送数据入FIFO
 * 参数：
 *      输入参数：f_data 数据，len 长度
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void drv_rf_wrtie_data_to_fifo(uint8_t *f_data,uint8_t len)
{
    uint8_t i = 0;

    StrobeCMD(CMD_TFR); //TX FIFO address pointer reset

    SCS_L();

    ByteSend(CMD_FIFO_W); //TX FIFO write command

    for (i = 0; i < len; i++)
        ByteSend(f_data[i]);

    SCS_H();
}
/*a7129驱动*/

/**************************************************
 * 函数功能：RF数据接收
 * 参数：
 *      输入参数：r_data 接收到的数据
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void drv_rf_get_data_from_fifo(uint8_t *r_data)
{
    StrobeCMD(CMD_RFR); //RX FIFO address pointer reset

    SCS_L();

    ByteSend(CMD_FIFO_R); //RX FIFO read command
    for (uint8_t i = 0; i < RF_BUF_MAX_LEN; i++)
    {
        r_data[i] = ByteRead();
    }

    SCS_H();
}

/**************************************************
 * 函数功能：设置RF状态
 * 参数：
 *      输入参数：state 发送/接收
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void drv_set_rf_state(rf_state_t state)
{
    if(state == R_S_TX) //发送
    {
        rf_state = R_S_TX; //发送模式

        StrobeCMD(CMD_TX);
    }
    else    //接收
    {
        rf_state = R_S_RX; 

        StrobeCMD(CMD_RX);
    }
}

/**************************************************
 * 函数功能：RF发送数据入队
 * 参数：
 *      输入参数：q_data 数据，len 长度
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
void rf_send_data_enqueue(void *q_data,uint8_t len)
{
    if(len == 0)
    {
        RF_LOG_E("rf enqueue data len must != 0");
        return;
    }

    xSemaphoreTake((SemaphoreHandle_t)rf_send_mutex_semaphore,portMAX_DELAY);   //获取信号量

    rf_buf_t temp_data;
    memset((void *)&temp_data,0,sizeof(rf_buf_t));
    if(len > RF_BUF_MAX_LEN)
    {
        RF_LOG_E("rf enqueue data len too long:%d",len);
    }
    else
    {
        memcpy((void *)temp_data.r_b_buf,q_data,len);
        if(len != RF_BUF_MAX_LEN)
        {
            memset((void *)&temp_data.r_b_buf[len],0xff,RF_BUF_MAX_LEN - len);  //不足64字节，补0xff
        }
        temp_data.r_b_len = RF_BUF_MAX_LEN;

        if(xQueueSend(rf_send_queue,(void *)&temp_data,0) != pdTRUE)  //失败
        {
            RF_LOG_E("rf send data enqueue fail");
        }
    }

    xSemaphoreGive((SemaphoreHandle_t)rf_send_mutex_semaphore); //释放信号量
}

/**************************************************
 * 函数功能：射频433发送任务
 * 参数：
 *      输入参数：无
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void rf_send_task(void *args)
{
    rf_tx_done_semaphore = xSemaphoreCreateBinary();
    if(rf_tx_done_semaphore == NULL)
    {
        RF_LOG_E("rf tx semaphore create fail");
        return;
    }

    rf_send_mutex_semaphore = xSemaphoreCreateMutex();
    if(rf_send_mutex_semaphore == NULL)
    {
        RF_LOG_E("rf send semaphore create fail");
        return;
    }

    rf_send_queue = xQueueCreate(QUEUE_MAX_AMOUNT,sizeof(rf_buf_t));   //发送数据队列
    if(rf_send_queue == NULL)
    {
        RF_LOG_E("rf send queue create fail");
        return;
    }

    rf_buf_t *rf_send_buf = NULL;

    rf_send_buf = malloc(sizeof(rf_buf_t));
	if(NULL == rf_send_buf)
	{
		RF_LOG_E("rf send buf malloc fail");
        return;
	}

    while(1)
    {
        memset((void *)rf_send_buf,0,sizeof(rf_buf_t));

        xQueueReceive(rf_send_queue,(void *)rf_send_buf,portMAX_DELAY);    //等待接收

        if(rf_send_buf->r_b_len > 0)
        {
            log_ascii("rf send data",(void *)rf_send_buf->r_b_buf, rf_send_buf->r_b_len);

            drv_rf_wrtie_data_to_fifo(rf_send_buf->r_b_buf, rf_send_buf->r_b_len);    //先加入FIFO

            drv_set_rf_state(R_S_TX);   //发送

            if( xSemaphoreTake( rf_tx_done_semaphore, ( TickType_t ) 100 ) != pdTRUE )  //等待发送完成
            {
                RF_LOG_E("rf send data ,wait send done timeout!");
            }

            drv_set_rf_state(R_S_RX);   //接收

            RF_LOG_I("rf send success!!!!");
        }
        else
        {
            RF_LOG_E("uart rx wait parse buf enqueue,but empty");
        }        
    }

    vTaskDelete(NULL);
}

/**************************************************
 * 函数功能：射频433接收任务
 * 参数：
 *      输入参数：无
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
static void rf_receive_task(void *args)
{
    uint8_t rf_init_suc = 0;

    RF_LOG_I("a7129 init start");

    A7129_POR(); //power on only

    while (1)
    {
        if (InitRF()) //init RF
        {
            rf_init_suc = 1;
            // entry_deep_sleep_mode();
            // Delay1ms(2);
            // wake_up_from_deep_sleep_mode();
            if (pdPASS != xTaskCreate(rf_send_task,
                                      "rf send task",
                                      2048, //configMINIMAL_STACK_SIZE
                                      NULL,
                                      RF_TASK_PRIORITY,
                                      NULL))
            {
                RF_LOG_E("create rf send task fail");
            }
        }
        else
        {
            break;
        }
    }

    if (rf_init_suc == 0)
    {
        RF_LOG_E("a7129 init fail!!!!");

        vTaskDelete(NULL);

        return;
    }

    drv_set_rf_state(R_S_RX);   //接收

    rf_received_semaphore = xSemaphoreCreateBinary();
    if(rf_received_semaphore == NULL)
    {
        RF_LOG_E("rf received semaphore create fail");
        return;
    }

    rf_buf_t *rf_received_buf = NULL;

    rf_received_buf = malloc(sizeof(rf_buf_t));
	if(NULL == rf_received_buf)
	{
		RF_LOG_E("rf received buf malloc fail");
        return;
	}

    while (1)
    {
        if( xSemaphoreTake( rf_received_semaphore, portMAX_DELAY ) == pdTRUE )
        {
            drv_rf_get_data_from_fifo(rf_received_buf->r_b_buf);
            rf_received_buf->r_b_len = RF_BUF_MAX_LEN;

            /*数据解析*/
            app_rf_data_parse(rf_received_buf->r_b_buf,rf_received_buf->r_b_len);
        }
    }

    vTaskDelete(NULL);
}

/**************************************************
 * 函数功能：a7129初始化
 * 参数：
 *      输入参数：无
 *      输出参数：无
 * 结果输出：无
 * 编辑者：Guest
**************************************************/
void drv_a7129_init(void)
{
    drv_a7129_pin_init();

    if (pdPASS != xTaskCreate(rf_receive_task,
                              "rf receive task",
                               2048,
                              NULL,
                              RF_TASK_PRIORITY,
                              NULL)) {
        RF_LOG_E("create rf receive task fail");
    }
}
