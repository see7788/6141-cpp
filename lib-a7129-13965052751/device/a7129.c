#include "string.h"
#include "a7129.h"
#include "a7129reg.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"

#define ID_TABLE_MAX_LENGTH         4 
#define RX_PACKAGE_MAX_LENGTH       7  
#define A7129_USE_SOFT_SPI

typedef struct a7129_config_struct
{
    uint8_t                     id_table[ID_TABLE_MAX_LENGTH];
    uint8_t                     id_table_length;
    uint8_t                     rx_packget_length;
    uint8_t                     rx_step_time;
    uint16_t*                   a7129_config;
    uint16_t*                   a7129_config_page_a;
    uint16_t*                   a7129_config_page_b;
    gpio_num_t                  a7129_device_pin_cs;
    gpio_num_t                  a7129_device_pin_clk;
    gpio_num_t                  a7129_device_pin_cko;
    gpio_num_t                  a7129_device_pin_data;
    gpio_num_t                  a7129_device_pin_gpio_1;
    gpio_num_t                  a7129_device_pin_gpio_2;
}a7129_config_t;

typedef struct a7129_info_struct
{
    uint8_t                     a7129_init_flag;
    a7129_config_t*             a7129_config;
    a7129_receive_call_back_t   receive_call_back;
    void*                       call_back_parma;
    uint8_t                     rx_time;
    uint8_t                     rx_buffer[RX_PACKAGE_MAX_LENGTH];
    TimerHandle_t               rx_timer;
    SemaphoreHandle_t           rx_sem;
}a7129_info_t;

const uint16_t A7129Config[]=		//433MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0021, // SYSTEM CLOCK register,
    0x0A21, // PLL1 register,
    0xDA05, // PLL2 register,	433.301MHz
    0x0000, // PLL3 register,
    0x0A20, // PLL4 register,
    0x0024, // PLL5 register,
    0x0000, // PLL6 register,
    0x0011, // CRYSTAL register,
    0x0000, // PAGEA,
    0x0000, // PAGEB,
    0x18D4, // RX1 register, 	IFBW=100KHz, ETH=1
    0x7009, // RX2 register, 	by preamble
    0x4000, // ADC register,
    0x0800, // PIN CONTROL register,		Use Strobe CMD
    0x4C45, // CALIBRATION register,
    0x20C0  // MODE CONTROL register, 	Use FIFO mode
};

const uint16_t   A7129Config_PageA[]=   //433MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0xF706, // TX1 register, 	Fdev = 37.5kHz
    0x0000, // WOR1 register,
    0xF800, // WOR2 register,
    0x1107, // RFI register, 	Enable Tx Ramp up/down
    0x0970, // PM register,		CST=1
    0x0302, // RTH register,
    0x400F, // AGC1 register,
    0x0AC0, // AGC2 register,
    0x0001, // GIO register, 	GIO2=WTR, GIO1=FSYNC
    0xD181, // CKO register
    0x0004, // VCB register,
    0x0A21, // CHG1 register, 	430MHz
    0x0022, // CHG2 register, 	435MHz
    0x0013, // FIFO register, 	FEP=63+1=64bytes
    0x1507, // CODE register, 	Preamble=4bytes, ID=4bytes
    0x0000  // WCAL register,
};

const uint16_t   A7129Config_PageB[]=   //433MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0337, // TX2 register,
    0x8400, // IF1 register, 	Enable Auto-IF, IF=200KHz
    0x0000, // IF2 register,
    0x0000, // ACK register,
    0x0000  // ART register,
};

const a7129_config_t a7129_config = 
{
    .id_table                               = {0x54, 0x21, 0xA4, 0x23},
    .id_table_length                        = 4,
    .rx_packget_length                      = 7,
    .rx_step_time                           = 5,
    .a7129_config                           = A7129Config,
    .a7129_config_page_a                    = A7129Config_PageA,
    .a7129_config_page_b                    = A7129Config_PageB,
    .a7129_device_pin_clk                   = GPIO_NUM_33,
    .a7129_device_pin_data                  = GPIO_NUM_32,
    .a7129_device_pin_cs                    = GPIO_NUM_14,
    // .a7129_device_pin_cko                   = GPIO_NUM_36,
    .a7129_device_pin_gpio_1                = GPIO_NUM_34,
    .a7129_device_pin_gpio_2                = GPIO_NUM_39,
};

#if defined(A7129_USE_SOFT_SPI) && defined(A7129_USE_HARD_SPI)
    #error "you can only chose one spi function"
#elif (!defined(A7129_USE_SOFT_SPI)) && (!defined(A7129_USE_HARD_SPI))
    #error "you shold chose one of spi function"
#elif defined(A7129_USE_SOFT_SPI)
    #define IO_CLR(PIN) gpio_set_level(PIN,0)
    #define IO_SET(PIN) gpio_set_level(PIN,1)
    #define IO_GET(PIN) gpio_get_level(PIN)
#elif defined(A7129_USE_HARD_SPI)
#endif

#ifdef A7129_MODULE_DEBUG_OUTPUT_ON
    static const char *TAG = "[A7129_DRIVER]";
#else
    #define ESP_LOGI( tag, format, ... )      
#endif

a7129_info_t a7129_info;

void RxPacket(a7129_info_t* this);
void a7129_error_handle(void);
static void a7129_change_command(a7129_info_t* this,uint8_t command);

#ifndef THIS_IS_OTHER_FILE_IN_OTHER_MODULE
    static __inline void delay_clock(int ts)
    {
        uint32_t start, curr;
        __asm__ __volatile__("rsr %0, ccount" : "=r"(start));
        do
        {
            __asm__ __volatile__("rsr %0, ccount" : "=r"(curr));
            
        }while (curr - start <= ts);
    }
    void clk_delay(void)
    {
        delay_clock(15);
    }
#endif

void a7129_receive_timer( TimerHandle_t xTimer )
{
    a7129_info_t* this = (a7129_info_t*)pvTimerGetTimerID(xTimer);;
    this->rx_time += 1;
    if(this->rx_time > this->a7129_config->rx_step_time)
    {
        ESP_LOGI("A7129", "[ENTER GPIO ISR]%d",this->a7129_config->id_table_length);
        a7129_change_command(this,CMD_RX);
        RxPacket(this);
        this->rx_time = 0;
        xTimerStop(this->rx_timer,0);
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
            this->receive_call_back(this->call_back_parma,this->rx_buffer,this->a7129_config->rx_packget_length);
        }
    }
}

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    a7129_info_t* this = (a7129_info_t*)arg;
    if(this->a7129_init_flag)
    {
        this->rx_time = 0;
        BaseType_t err;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        err = xTimerStartFromISR(this -> rx_timer,&xHigherPriorityTaskWoken);
        if (err != pdTRUE)//
        {
        }
        portYIELD_FROM_ISR();   
    }
}

static inline void soft_spi_write_byte(a7129_info_t* this,uint32_t data,uint8_t data_length)
{
    uint32_t mask = 0x01 << (data_length - 1);
    for(int i=0; i<data_length; i++)
    {
        if(data & mask)
        {
            IO_SET(this->a7129_config->a7129_device_pin_data);
        }
        else
        {
            IO_CLR(this->a7129_config->a7129_device_pin_data);
        }    
        clk_delay();
        IO_SET(this->a7129_config->a7129_device_pin_clk);
        clk_delay();
        IO_CLR(this->a7129_config->a7129_device_pin_clk);
        data<<=1;
    }
}

static inline uint32_t soft_spi_read_byte(a7129_info_t* this,uint8_t data_length)
{
    uint8_t result = 0;
    IO_SET(this->a7129_config->a7129_device_pin_data);
    gpio_set_direction(this->a7129_config->a7129_device_pin_data,GPIO_MODE_INPUT);
    for(int i=0; i<data_length; i++)
    {
        if(IO_GET(this->a7129_config->a7129_device_pin_data))
        {
            result = (result << 1) | 0x01;
        }
        else
        {
            result = result << 1;
        }    
        IO_SET(this->a7129_config->a7129_device_pin_clk);
        clk_delay();
        IO_CLR(this->a7129_config->a7129_device_pin_clk);
    }
    gpio_set_direction(this->a7129_config->a7129_device_pin_data,GPIO_MODE_OUTPUT);
    return result;
}

static inline void spi_write_byte(a7129_info_t* this,uint32_t data,uint8_t data_length)
{
    soft_spi_write_byte(this,data,data_length);
}

static inline uint32_t spi_read_byte(a7129_info_t* this,uint8_t data_length)
{
    return soft_spi_read_byte(this,data_length);
}

static void a7129_change_command(a7129_info_t* this,uint8_t command)
{
    IO_CLR(this->a7129_config->a7129_device_pin_cs);
    spi_write_byte(this,command,8);
	IO_SET(this->a7129_config->a7129_device_pin_cs);
}

static void A7129_WriteReg(a7129_info_t* this,uint8_t address, uint16_t reg)
{
    IO_CLR(this->a7129_config->a7129_device_pin_cs);
    address |= CMD_Reg_W;
    spi_write_byte(this,address,8);
    clk_delay();
    spi_write_byte(this,reg,16);
    IO_SET(this->a7129_config->a7129_device_pin_cs);
    uint8_t i;
}

static uint16_t A7129_ReadReg(a7129_info_t* this,uint8_t address)
{
    uint16_t result;
    IO_CLR(this->a7129_config->a7129_device_pin_cs);
    address |= CMD_Reg_R;
    spi_write_byte(this,address,8);
    clk_delay();
    result = spi_read_byte(this,16);
    IO_SET(this->a7129_config->a7129_device_pin_cs);
    return result;
}

static void A7129_WritePageA(a7129_info_t* this,uint8_t address, uint16_t dataWord)
{
    A7129_WriteReg(this,CRYSTAL_REG, ((address << 12) | A7129Config[CRYSTAL_REG]));
    A7129_WriteReg(this,PAGEA_REG, dataWord);
}

static uint16_t A7129_ReadPageA(a7129_info_t* this,uint8_t address)
{
    A7129_WriteReg(this,CRYSTAL_REG, ((address << 12) | A7129Config[CRYSTAL_REG]));
    return A7129_ReadReg(this,PAGEA_REG);
}

static void A7129_WritePageB(a7129_info_t* this,uint8_t address, uint16_t dataWord)
{
    A7129_WriteReg(this,CRYSTAL_REG, ((address << 7) | A7129Config[CRYSTAL_REG]));
    A7129_WriteReg(this,PAGEB_REG, dataWord);
}

static uint16_t A7129_ReadPageB(a7129_info_t* this,uint8_t address)
{
    A7129_WriteReg(this,CRYSTAL_REG, ((address << 7) | A7129Config[CRYSTAL_REG]));
    return A7129_ReadReg(this,PAGEB_REG);
}

static uint32_t a7129_write_config(a7129_info_t* this)
{
    for(int i = 0;i < 8;i++)
    {
        A7129_WriteReg(this,i, A7129Config[i]);
    }
	for(int i = 10;i < 16;i++)
    {
        A7129_WriteReg(this,i, A7129Config[i]);
    }
    for(int i = 0;i < 16;i++)
    {
        A7129_WritePageA(this,i, A7129Config_PageA[i]);
    }
	for(int i=0;i < 5;i++)
    {
        A7129_WritePageB(this,i, A7129Config_PageB[i]);
    }
	if(A7129_ReadReg(this,SYSTEMCLOCK_REG) != A7129Config[SYSTEMCLOCK_REG])
	{
		a7129_error_handle();	
	}
    return 0;
}

static uint32_t a7129_write_id(a7129_info_t* this)
{
    IO_CLR(this->a7129_config->a7129_device_pin_cs);
    spi_write_byte(this,CMD_ID_W,8);
    for(int i=0;i<this->a7129_config->id_table_length;i++)
    {
        spi_write_byte(this,this->a7129_config->id_table[i],8);
    }
	IO_SET(this->a7129_config->a7129_device_pin_cs);
    clk_delay();
	IO_CLR(this->a7129_config->a7129_device_pin_cs);
	spi_write_byte(this,CMD_ID_R,8);
    for(int i=0;i<this->a7129_config->id_table_length;i++)
    {
        if(this->a7129_config->id_table[i] != spi_read_byte(this,8))
        {
            IO_SET(this->a7129_config->a7129_device_pin_cs);
            a7129_error_handle();
        }
    }
	IO_SET(this->a7129_config->a7129_device_pin_cs);
    return 0;
}

static uint32_t a7129_set_if_vco(a7129_info_t* this)
{
	A7129_WriteReg(this,MODE_REG, A7129Config[MODE_REG] | 0x0802);			//IF Filter & VCO Current Calibration
    while(A7129_ReadReg(this,MODE_REG) & 0x0802)
    {
        vTaskDelay(1);
    }
	
    //for check(IF Filter)
    if((A7129_ReadReg(this,CALIBRATION_REG)>>4) & 0x01)
    {
        a7129_error_handle();
    }

	//for check(VCO Current)
	if((A7129_ReadPageA(this,VCB_PAGEA)>>4) & 0x01)
	{
        a7129_error_handle();
    }
     
    //RSSI Calibration procedure @STB state
	A7129_WriteReg(this,ADC_REG, 0x4C00);									    //set ADC average=64
    A7129_WritePageA(this,WOR2_PAGEA, 0xF800);								    //set RSSC_D=40us and RS_DLY=80us
	A7129_WritePageA(this,TX1_PAGEA, A7129Config_PageA[TX1_PAGEA] | 0xE000);	//set RC_DLY=1.5ms
    A7129_WriteReg(this,MODE_REG, A7129Config[MODE_REG] | 0x1000);			    //RSSI Calibration
    while(A7129_ReadReg(this,MODE_REG) & 0x1000)
    {
        vTaskDelay(1);
    }
	A7129_WriteReg(this,ADC_REG, A7129Config[ADC_REG]);
    A7129_WritePageA(this,WOR2_PAGEA, A7129Config_PageA[WOR2_PAGEA]);
	A7129_WritePageA(this,TX1_PAGEA, A7129Config_PageA[TX1_PAGEA]);

    //VCO calibration procedure @STB state
	A7129_WriteReg(this,PLL1_REG, A7129Config[PLL1_REG]);
	A7129_WriteReg(this,PLL2_REG, A7129Config[PLL2_REG]);
	A7129_WriteReg(this,MODE_REG, A7129Config[MODE_REG] | 0x0004);		//VCO Band Calibration
	while(A7129_ReadReg(this,MODE_REG) & 0x0004);
    {
        vTaskDelay(1);
    }
	
	//for check(VCO Band)
	if((A7129_ReadReg(this,CALIBRATION_REG) >>8) & 0x01)
	{
		a7129_error_handle();
	}
    return 0;
}

static void a7129_gpio_init(a7129_info_t* this)
{
    gpio_config_t io_conf; 

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_DEF_OUTPUT;
    io_conf.pin_bit_mask = 1ULL<<this->a7129_config->a7129_device_pin_data; //data
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    
    // io_conf.pin_bit_mask = 1ULL<<this->a7129_config->a7129_device_pin_cko; //clk
    // gpio_config(&io_conf);                      

    io_conf.pin_bit_mask = 1ULL<<this->a7129_config->a7129_device_pin_clk; //clk
    gpio_config(&io_conf);                      

    io_conf.pin_bit_mask = 1ULL<<this->a7129_config->a7129_device_pin_cs; //cs
    gpio_config(&io_conf);                      

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_DEF_INPUT;
    io_conf.pin_bit_mask = 1ULL<<this->a7129_config->a7129_device_pin_gpio_1;
    gpio_config(&io_conf);                      

    io_conf.pin_bit_mask = 1ULL<<this->a7129_config->a7129_device_pin_gpio_2; 
    gpio_config(&io_conf);                     

    gpio_set_level(this->a7129_config->a7129_device_pin_data,1);
    //gpio_set_level(this->a7129_config->a7129_device_pin_cko,1);
    gpio_set_level(this->a7129_config->a7129_device_pin_clk,0);
    gpio_set_level(this->a7129_config->a7129_device_pin_cs,1);

    gpio_install_isr_service(1);
    gpio_isr_handler_add(this->a7129_config->a7129_device_pin_gpio_1, gpio_isr_handler,(void *)this);
    //gpio_isr_handler_add(this->a7129_config->a7129_device_pin_gpio_2, gpio_isr_handler,(void *)this);
}

void a7129_init(void)
{
    memset(&a7129_info,0,sizeof(a7129_info));
    a7129_info.a7129_config = &a7129_config;
    a7129_info_t* this      = &a7129_info;
    a7129_gpio_init(this);
    a7129_change_command(this,CMD_RF_RST);	    //reset A7129 chip
    a7129_write_config(this);	                //config A7129 chip
    vTaskDelay(1);
    a7129_write_id(this);		                //write ID  
    a7129_set_if_vco(this);			            //IF and VCO calibration
    a7129_change_command(this,CMD_STBY);
    a7129_change_command(this,CMD_PLL);
    a7129_change_command(this,CMD_RX);
    this -> rx_timer = xTimerCreate("receive_timer",1,pdTRUE,this,a7129_receive_timer);
    this->a7129_init_flag = 1;
}

void RxPacket(a7129_info_t* this)
{
	a7129_change_command(this,CMD_RFR);		//RX FIFO address pointer reset
    //printf("receive ");
    IO_CLR(this->a7129_config->a7129_device_pin_cs);
    spi_write_byte(this,CMD_FIFO_R,8);
    for(int i=0; i <this->a7129_config->rx_packget_length; i++)
	{
		this->rx_buffer[i] = spi_read_byte(this,8);
        //printf("%d ",this->rx_buffer[i]);
	}
	IO_SET(this->a7129_config->a7129_device_pin_cs);
    //printf("\r\n");
}

void a7129_error_handle(void)
{
    while(1)
    {
        vTaskDelay(1000);
        //error handle
    };
}

void a7129_set_receive_config(SemaphoreHandle_t xSemaphore,a7129_receive_call_back_t receive_call_back,void* call_back_parama)
{
    a7129_info_t* this      = &a7129_info;
    this->rx_sem            = xSemaphore;
    this->receive_call_back = receive_call_back;
    this->call_back_parma   = call_back_parama;
}

uint32_t a7129_send_data_block(uint8_t* data,uint8_t size)
{
    a7129_info_t* this      = &a7129_info;
    a7129_change_command(this,CMD_TFR);
    IO_CLR(this->a7129_config->a7129_device_pin_cs);
    spi_write_byte(this,CMD_FIFO_W,8);
    for(int i=0;i<size;i++)
    {
        spi_write_byte(this,data[i],8);
    }
	IO_SET(this->a7129_config->a7129_device_pin_cs);
    return 0;
}