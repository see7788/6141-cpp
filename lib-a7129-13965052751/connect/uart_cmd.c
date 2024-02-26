#include "uart_cmd.h"
#include "stdio.h"

void cmd_decode(unsigned char * buff)
{
    printf("[%d]%s",sizeof(buff),buff);
}