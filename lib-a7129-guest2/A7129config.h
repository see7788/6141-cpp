 /********************************************************************
*   A7129CONFIG.h
*   RF Chip-A7129 Configure Definitions
*
*   This file provides the constants associated with the
*   AMICCOM A7129 device.
*
********************************************************************/
#ifndef _A7129CONFIG_h_
#define _A7129CONFIG_h_

#include <stdio.h>

//#define DR_9p6Kbps_96KIFBW       //433MHz, 9.6kbps (IFBW = 96Hz, Fdev = 37.5KHz), Crystal=12.8MHz
#define DR_10Kbps_50KIFBW       //433MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
//#define DR_10Kbps_100KIFBW      //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
//#define DR_50Kbps_50KIFBW       //433MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
//#define DR_100Kbps_100KIFBW     //433MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
//#define DR_150Kbps_150KIFBW     //433MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
//#define DR_250Kbps_250KIFBW     //433MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz

#ifdef DR_9p6Kbps_96KIFBW

const uint16_t  A7129Config[]=        //433MHz, 9p6kbps (IFBW = 96KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x12C7,             //[00]System Clock
    0x0A21,             //[01]PLL1 
    0xDA05,             //[02]PLL2
    0x0000,             //[03]PLL3
    0x0E20,             //[04]PLL4  
    0x0024,             //[05]PLL5  
    0x0000,             //[06]PLL6 
    0x0013,             //[07]Crystal 
    0x0127,             //[08]PageA
    0x000E,             //[09]PageB
    0x1894,             //[0A]RX1
    0x7009,             //[0B]RX2
    0x4000,             //[0C]ADC
    0x0800,             //[0D]Pin Control 
    0x4C45,             //[0E]Calibration 
    0x20C0,             //[0F]Mode Control   
};

const uint16_t  A7129Config_PageA[]=   //433MHz, 9p6kbps (IFBW = 96KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0xF706,             //[PageA-00]TX1
    0x0000,             //[PageA-01]WOR1
    0xF800,             //[PageA-02]WOR2
    0x1907,             //[PageA-03]RFI
    0x1B70,             //[PageA-04]PM
    0x0302,             //[PageA-05]RTH
    0x400F,             //[PageA-06]AGC
    0x0AC0,             //[PageA-07]AGC2
    0x2045,             //[PageA-08]GPIO
    0xD181,             //[PageA-09]CKO
    0x0004,             //[PageA-0A]VCB
    0x0A21,             //[PageA-0B]CHG1
    0x0022,             //[PageA-0C]CHG2
    0x003F,             //[PageA-0D]FIFO
    0x1507,             //[PageA-0E]CODE
    0x0000,             //[PageA-0F]WCAL
};

const uint16_t  A7129Config_PageB[]=   //433MHz, 9p6kbps (IFBW = 96KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0337,             //[PageB-00]TX2
    0x83D7,             //[PageB-01]IF1
    0x0000,             //[PageB-02]IF2
    0x0000,             //[PageB-03]ACK
    0x0000,             //[PageB-04]ART
};


#endif

#ifdef DR_10Kbps_50KIFBW

const uint16_t  A7129Config[]=        //433MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0823,     //SYSTEM CLOCK register,
    0x0A21,     //PLL1 register,
    0xDA05,     //PLL2 register,    433.301MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D0,     //RX1 register,     IFBW=50KHz  
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const uint16_t  A7129Config_PageA[]=   //433MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0xF606,     //TX1 register,     Fdev = 18.75kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x9B70,     //PM register,      CST=1
    0x0302,     //RTH register,
    0x400F,     //AGC1 register,    
    0x0AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0A21,     //CHG1 register,    430MHz
    0x0022,     //CHG2 register,    435MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const uint16_t  A7129Config_PageB[]=   //433MHz, 10kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8200,     //IF1 register,     Enable Auto-IF, IF=100KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

#endif


#ifdef DR_10Kbps_100KIFBW

const uint16_t  A7129Config[]=        //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x1221,     //SYSTEM CLOCK register,
    0x0A21,     //PLL1 register,
    0xDA05,     //PLL2 register,    433.301MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D4,     //RX1 register,     IFBW=100KHz 
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const uint16_t  A7129Config_PageA[]=   //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0xF706,     //TX1 register,     Fdev = 37.5kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x9B70,     //PM register,      CST=1
    0x0302,     //RTH register,
    0x400F,     //AGC1 register,    
    0x0AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0A21,     //CHG1 register,    430MHz
    0x0022,     //CHG2 register,    435MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const uint16_t  A7129Config_PageB[]=   //433MHz, 10kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8400,     //IF1 register,     Enable Auto-IF, IF=200KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

#endif


#ifdef DR_50Kbps_50KIFBW

const uint16_t  A7129Config[]=        //433MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0023,     //SYSTEM CLOCK register,
    0x0A21,     //PLL1 register,
    0xDA05,     //PLL2 register,    433.301MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D0,     //RX1 register,     IFBW=50KHz  
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const uint16_t  A7129Config_PageA[]=   //433MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0xF606,     //TX1 register,     Fdev = 18.75kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,      CST=1
    0x0302,     //RTH register,
    0x400F,     //AGC1 register,    
    0x0AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0A21,     //CHG1 register,    430MHz
    0x0022,     //CHG2 register,    435MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const uint16_t  A7129Config_PageB[]=   //433MHz, 50kbps (IFBW = 50KHz, Fdev = 18.75KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8200,     //IF1 register,     Enable Auto-IF, IF=100KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

#endif


#ifdef DR_100Kbps_100KIFBW

const uint16_t  A7129Config[]=        //433MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0021,     //SYSTEM CLOCK register,
    0x0A21,     //PLL1 register,
    0xDA05,     //PLL2 register,    433.301MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D4,     //RX1 register,     IFBW=100KHz
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const uint16_t  A7129Config_PageA[]=   //433MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0xF706,     //TX1 register,     Fdev = 37.5kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,
    0x0302,     //RTH register,
    0x400F,     //AGC1 register,    
    0x0AC0,     //AGC2 register, 
    0x0045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0A21,     //CHG1 register,    430MHz
    0x0022,     //CHG2 register,    435MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const uint16_t  A7129Config_PageB[]=   //433MHz, 100kbps (IFBW = 100KHz, Fdev = 37.5KHz), Crystal=12.8MHz
{
    0x0337,     //TX2 register,
    0x8400,     //IF1 register,     Enable Auto-IF, IF=200KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

#endif



#ifdef DR_150Kbps_150KIFBW

const uint16_t  A7129Config[]=        //433MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
{
    0x0021,     //SYSTEM CLOCK register,
    0x0A16,     //PLL1 register,
    0x9158,     //PLL2 register,    433.301MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18D8,     //RX1 register,     IFBW=150KHz 
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const uint16_t  A7129Config_PageA[]=   //433MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
{
    0xF706,     //TX1 register,     Fdev = 56.25kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,
    0x0302,     //RTH register,
    0x400F,     //AGC1 register,    
    0x0AC0,     //AGC2 register, 
    0x4045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0616,     //CHG1 register,    430MHz
    0x0B16,     //CHG2 register,    435MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const uint16_t  A7129Config_PageB[]=   //433MHz, 150kbps (IFBW = 150KHz, Fdev = 56.25KHz), Crystal=19.2MHz
{
    0x8337,     //TX2 register,
    0x8400,     //IF1 register,     Enable Auto-IF, IF=300KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

#endif


#ifdef DR_250Kbps_250KIFBW

const uint16_t  A7129Config[]=        //433MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz
{
    0x0020,     //SYSTEM CLOCK register,
    0x0A1B,     //PLL1 register,
    0x14D0,     //PLL2 register,    433.301MHz
    0x0000,     //PLL3 register,
    0x0E20,     //PLL4 register,
    0x0024,     //PLL5 register,
    0x0000,     //PLL6 register,
    0x0011,     //CRYSTAL register,
    0x0000,     //PAGEA,
    0x0000,     //PAGEB,
    0x18DC,     //RX1 register,     IFBW=250KHz 
    0x7009,     //RX2 register,     by preamble
    0x4000,     //ADC register,     
    0x0800,     //PIN CONTROL register,     Use Strobe CMD
    0x4C45,     //CALIBRATION register,
    0x20C0      //MODE CONTROL register,    Use FIFO mode
};

const uint16_t  A7129Config_PageA[]=   //433MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz
{
    0xF530,     //TX1 register,     Fdev = 93.75kHz
    0x0000,     //WOR1 register,
    0xF800,     //WOR2 register,
    0x1907,     //RFI register,     Enable Tx Ramp up/down  
    0x1B70,     //PM register,
    0x0302,     //RTH register,
    0x400F,     //AGC1 register,    
    0x0AC0,     //AGC2 register, 
    0x8045,     //GIO register,     GIO2=WTR, GIO1=FSYNC
    0xD181,     //CKO register
    0x0004,     //VCB register,
    0x0E1A,     //CHG1 register,    430MHz
    0x031B,     //CHG2 register,    435MHz
    0x003F,     //FIFO register,    FEP=63+1=64bytes
    0x1507,     //CODE register,    Preamble=4bytes, ID=4bytes
    0x0000      //WCAL register,
};

const uint16_t  A7129Config_PageB[]=   //433MHz, 250kbps (IFBW = 250KHz, Fdev = 93.75KHz), Crystal=16MHz
{
    0x0337,     //TX2 register,
    0x8800,     //IF1 register,     Enable Auto-IF, IF=500KHz
    0x0000,     //IF2 register,
    0x0000,     //ACK register,
    0x0000      //ART register,
};

#endif


#endif

