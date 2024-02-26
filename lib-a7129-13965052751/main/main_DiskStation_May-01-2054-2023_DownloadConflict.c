#include "string.h"
#include "main.h"
#include "uart_bus.h"
#include "a7129.h"

uint8_t CRC16_High, CRC16_Low;
uint8_t CRC16_LookupHigh[16] = {
0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70,
0x81, 0x91, 0xA1, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1
};
uint8_t CRC16_LookupLow[16] = {
0x00, 0x21, 0x42, 0x63, 0x84, 0xA5, 0xC6, 0xE7,
0x08, 0x29, 0x4A, 0x6B, 0x8C, 0xAD, 0xCE, 0xEF
};
void CRC16_Init( void )
{
    CRC16_High = 0x1D;
    CRC16_Low = 0x0F;
}
void CRC16_Update4Bits( uint8_t val )
{
    uint8_t t;
    // Step one, extract the Most significant 4 bits of the CRC register
    t = CRC16_High >> 4;
    // XOR in the Message Data into the extracted bits
    t = t ^ val;
    // Shift the CRC Register left 4 bits
    CRC16_High = (CRC16_High << 4) | (CRC16_Low >> 4);
    CRC16_Low = CRC16_Low << 4;
    // Do the table lookups and XOR the result into the CRC Tables
    CRC16_High = CRC16_High ^ CRC16_LookupHigh[t];
    CRC16_Low = CRC16_Low ^ CRC16_LookupLow[t];
}
void CRC16_Update( uint8_t val )
{
    CRC16_Update4Bits( val >> 4 ); // High nibble first
    CRC16_Update4Bits( val & 0x0F ); // Low nibble
}

uint8_t Payload[7];
uint8_t DataBuf[6];
#define TRUE 1
#define FALSE 0

void CRC_test(uint8_t length)
{
    uint8_t i;
    CRC16_Init();
    for(i=0;i<length;i++) //length--> the payload length
    CRC16_Update(Payload[i]); //Payload--> the data you send
}
uint8_t Data_Output() //数据输出
{
    CRC_test(3);

    if((CRC16_High == Payload[3]) && (CRC16_Low == Payload[4]))
    {
        CRC_test(6);
        if(CRC16_Low == Payload[6])
        {
            DataBuf[0] = Payload[0];
            DataBuf[1] = Payload[1];
            DataBuf[2] = Payload[5]&0x0F;
            DataBuf[3] = ((Payload[5]&0x3F)>>4)*2;
            DataBuf[4] = (Payload[5]>>6) + 1;
            DataBuf[5] = Payload[2];
            return(TRUE); //接收正确
        }
        else
        {
            return(FALSE); //接收错误
        }
    }
    else
    {
        return(FALSE); //接收错误
    }
}

void a7129_receive_call_back(void* user_parama,uint8_t* buffer,uint8_t size)
{
    memcpy(&Payload,buffer,7);
    Data_Output();
    printf("data is : ");
    for(int i=0;i<6;i++)
    {
        printf("%d ",DataBuf[i]);
    }
    printf("\r\n");
}


void app_main(void)
{
    show_chip_info();
    a7129_init();
    memset(&Payload,0,7);
    memset(&DataBuf,0,6);
    a7129_set_receive_config(NULL,a7129_receive_call_back,NULL);
    while(1)
    {
        vTaskDelay(10);
    }
}

