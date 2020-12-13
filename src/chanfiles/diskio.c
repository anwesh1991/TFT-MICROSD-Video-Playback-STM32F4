/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include <math.h>
#include <libopencm3/stm32/usart.h>
#include "../videoplayer.h"
#include <libopencm3/stm32/dma.h>

int cdv;





int cmd58() 
{
    CS_ON_CLR();
    int arg = 0;
    int response;

    // send a command
    spi_xfer(SPI2, 0x40 | 58);
    spi_xfer(SPI2, arg >> 24);
    spi_xfer(SPI2, arg >> 16);
    spi_xfer(SPI2, arg >> 8);
    spi_xfer(SPI2, arg >> 0);
    spi_xfer(SPI2, 0x95);

    // wait for the repsonse (response[7] == 0)
    for (int i = 0; i < SD_COMMAND_TIMEOUT; i++) 
    {
        response = spi_xfer(SPI2, 0xFF);
        if (!(response & 0x80)) {
            int ocr = spi_xfer(SPI2, 0xFF) << 24;
            ocr |= spi_xfer(SPI2, 0xFF)  << 16;
            ocr |= spi_xfer(SPI2, 0xFF)  << 8;
            ocr |= spi_xfer(SPI2, 0xFF)  << 0;
            CS_OFF_SET();
            spi_xfer(SPI2, 0xFF);
            return response;
        }
    }
    CS_OFF_SET();
    spi_xfer(SPI2, 0xFF);
    return 5; // timeout
}




int initialise_card_v2() 
{
    for (int i = 0; i < SD_COMMAND_TIMEOUT; i++) 
    {
        msdelay(50);
        if(cmd58()==5)
        {
            usart_print_text("cmd58 fail");
        }
        if(cmd(55, 0)==5)
        {
            usart_print_text("cmd with 55 fail");
        }
        if (cmd(41, 0x40000000) == 0) 
        {
            cmd58();
            cdv = 1;
            return SDCARD_V2;
        }
    }

    usart_print_text("Timeout waiting for v2.x card\n");
    return SDCARD_FAIL;
}



int cmd8() 
{
    CS_ON_CLR();
    char response[5]; 

    spi_xfer(SPI2, 0x40 | 8);   // CMD8
    spi_xfer(SPI2, 0x00);       // reserved
    spi_xfer(SPI2, 0x00);       // reserved
    spi_xfer(SPI2, 0x01);       // 3.3v
    spi_xfer(SPI2, 0xAA);       // check pattern
    spi_xfer(SPI2, 0x87);       // crc

    // wait for the repsonse (response[7] == 0)
    for (int i = 0; i < SD_COMMAND_TIMEOUT * 1000; i++) 
    {
        response[0] = spi_xfer(SPI2, 0xFF);
        if (!(response[0] & 0x80)) 
        {
            for (int j = 1; j < 5; j++) 
            {
                response[i] = spi_xfer(SPI2, 0xFF);
            }
            CS_OFF_SET();
            spi_xfer(SPI2, 0xFF);
            return response[0];
        }
    }
    CS_OFF_SET();
    spi_xfer(SPI2, 0xFF);
    return 5; // timeout
}



int cmd(int cmd, int arg) 
{
    int response;
    CS_ON_CLR();
    
  
    spi_xfer(SPI2, 0x40 | cmd);
    spi_xfer(SPI2, arg >> 24);
    spi_xfer(SPI2, arg >> 16);
    spi_xfer(SPI2, arg >> 8);
    spi_xfer(SPI2, arg >> 0);
    spi_xfer(SPI2, 0x95);
    

    // wait for the repsonse (response[7] == 0)
    for (int i = 0; i < SD_COMMAND_TIMEOUT; i++) 
    {

        response = spi_xfer(SPI2, 0xFF);       
        if (!(response & 0x80)) 
        {
            CS_OFF_SET();
            spi_xfer(SPI2, 0xFF);
            return response;
        }
    }
    CS_OFF_SET();
    spi_xfer(SPI2, 0xFF);
    return 5; // timeout
}



int initialise_card() 
{
    //_spi.frequency(_init_sck);
    CS_OFF_SET();
    

    spi_xfer(SPI2, 0xFF);

    for (int i = 0; i < 16; i++) 
        spi_xfer(SPI2, 0xFF);
    

    // send CMD0, should return with all zeros except IDLE STATE set (bit 0)
    if (cmd(0, 0) != R1_IDLE_STATE) 
    {
        usart_print_text("Idle state fail\n");
        //usart_print_text("ic ret value: ");
        //usart_print_num2(response);
        return SDCARD_FAIL;
    }
    else
    {
        //usart_print_text("Idle state success\n");
    }

    // send CMD8 to determine whther it is ver 2.x
    int r = cmd8();
    if (r == R1_IDLE_STATE) 
    {
        //usart_print_text("cmd8 success v2\n");
        if(initialise_card_v2()==SDCARD_FAIL)
        {
            usart_print_text("init card v2 fail\n");
            return 0;
        }
        else
        {
           // usart_print_text("init card v2 success\n");
            return 1;
        }
    } 
    else if (r == (R1_IDLE_STATE | R1_ILLEGAL_COMMAND)) 
    {
       // usart_print_text("cmd8 success v1\n");
    } 
    else 
    {
        usart_print_text("not in idle state after sending cmd8\n");
        return SDCARD_FAIL;
    }
    
}

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv        /* Physical drive nmuber to identify the drive */
)
{

    if(initialise_card() == 0)
    {
        usart_print_text("disk initialization failure");
        return 1;
    }
    else
    {
        usart_print_text("disk initialization success");
    }

     // Set block length to 512 (CMD16)
    if (cmd(16, 512) != 0) 
    {
        usart_print_text("Set 512-byte block timed out\n");
        return 1;
    }
    else
    {
        //usart_print_text("cmd 16 512 pass\n");
    }


    return 0;
}





DSTATUS disk_status (
    BYTE pdrv        /* Physical drive nmuber to identify the drive */
)
{
    return 'x';
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,       /* Physical drive nmuber to identify the drive */
    BYTE* buff,      /* Data buffer to store read data */
    DWORD sector,    /* Sector address in LBA */
    UINT count       /* Number of sectors to read */
)
{
   // usart_print_text("SDFS disk read\n");

    if (pdrv || !count) return RES_PARERR;
    
    for (uint32_t b = sector; b < sector + count; b++) 
    {
        // set read address for single block (CMD17)
        if (cmd(17, b * cdv) != 0) 
        {
            usart_print_text("cmd-17-block-fail\n");
            return 1;
        }
        
        // receive the data
        read(buff, 512);
        buff += 512;
    }

    return 0;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/


DRESULT disk_write (
    BYTE pdrv,           /* Physical drive nmuber to identify the drive */
    const BYTE* buff,    /* Data to be written */
    DWORD sector,        /* Sector address in LBA */
    UINT count           /* Number of sectors to write */
)
{
   // usart_print_text("SDFS disk write\n");
    if (pdrv || !count) return RES_PARERR;
    
    for (uint32_t b = sector; b < sector + count; b++) {
        // set write address for single block (CMD24)
        if (cmd(24, b * cdv) != 0) 
        {
            usart_print_text("cmd-24-block-fail\n");
            return 1;
        }
        
        // send the data block
        write(buff, 512);
        buff += 512;
    }
    
    return 0;
}


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE pdrv,        /* Physical drive nmuber (0..) */
    BYTE cmd,         /* Control code */
    void* buff        /* Buffer to send/receive control data */
)
{
    return RES_OK;
}



void dma_transfer(uint16_t bytes, uint8_t *buff)
{

    CS_OFF_SET();

    dma_set_number_of_data(DMA1,DMA_STREAM3,bytes);
    dma_set_memory_address(DMA1,DMA_STREAM3,(DWORD)buff);


   // usart_print_text("enabling dma and spi-rx-dma");
    dma_enable_stream(DMA1,DMA_STREAM3);


    CS_ON_CLR();

    spi_enable_rx_dma(SPI2);
    
    //msdelay(100);
}



int read(uint8_t *buffer, uint32_t length) 
{
    //usart_print_text("SDFS read\n");
    CS_ON_CLR();

    // read until start byte (0xFF)
    while (spi_xfer(SPI2, 0xFF) != 0xFE);
    dma_transfer(length, buffer);

    // read data
    for (uint32_t i = 0; i < length; i++) 
    {
        //buffer[i] = spi_xfer(SPI2, 0xFF);
        spi_xfer(SPI2, 0xFF);
    }
    //spi_xfer(SPI2, 0xFF); // checksum
    //spi_xfer(SPI2, 0xFF);

    CS_OFF_SET();
    //spi_xfer(SPI2, 0xFF);
    //usart_print_text("SDFS read finish\n");
    return 0;
}

int write(const uint8_t *buffer, uint32_t length) 
{
    //usart_print_text("SDFS write\n");
    CS_ON_CLR();

    // indicate start of block
    spi_xfer(SPI2, 0xFE);

    // write the data
    for (uint32_t i = 0; i < length; i++) 
    {
        spi_xfer(SPI2, buffer[i]);
    }

    // write the checksum
    //spi_xfer(SPI2, 0xFF);
    //spi_xfer(SPI2, 0xFF);

    // check the response token
    if ( (spi_xfer(SPI2, 0xFF) & 0x1F) != 0x05) 
    {
        usart_print_text("SDFS write fail\n");
        CS_OFF_SET();
        spi_xfer(SPI2, 0xFF);
        return 1;
    }

    // wait for write to finish
    while (spi_xfer(SPI2, 0xFF) == 0);

    CS_OFF_SET();
   // spi_xfer(SPI2, 0xFF);
   // usart_print_text("SDFS write finish\n");

    return 0;
}
