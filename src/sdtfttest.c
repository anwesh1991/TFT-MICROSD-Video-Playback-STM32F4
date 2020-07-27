#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "chanfiles/ff.h"
#include "chanfiles/diskio.h"
#include "sdtfttest.h"

int Paint(FIL);


//#define DEBUG 1

static void gpio_setup(void)
{

	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_180MHZ]);		//PLL multiplier for increasing the system clock/oscillator (?) speed
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART2);
	
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);  //Default inbuilt led
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);	  //USART setup
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);							  //USART setup
}


static void dma_init(void)
{
	
	CS_OFF_SET();
	//rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA1EN);
    rcc_periph_clock_enable(RCC_DMA1);
    
    dma_stream_reset(DMA1, DMA_STREAM3);
    dma_disable_stream(DMA1,DMA_STREAM3);

	
    dma_set_transfer_mode(DMA1, DMA_STREAM3,DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_disable_peripheral_increment_mode(DMA1,DMA_STREAM3);
    dma_set_priority(DMA1,DMA_STREAM3,DMA_SxCR_PL_VERY_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1,DMA_STREAM3);
    dma_enable_memory_increment_mode(DMA1,DMA_STREAM3);
    dma_enable_direct_mode(DMA1,DMA_STREAM3);
    //dma_enable_circular_mode(DMA1,DMA_STREAM3);
    //dma_enable_fifo_mode(DMA1,DMA_STREAM3);
    dma_enable_transfer_error_interrupt(DMA1,DMA_STREAM3);
    dma_set_peripheral_size(DMA1,DMA_STREAM3,DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1,DMA_STREAM3,DMA_SxCR_MSIZE_16BIT);	//8bit?
    dma_set_peripheral_address(DMA1,DMA_STREAM3,(DWORD) &SPI_DR(SPI2));
    dma_channel_select(DMA1, DMA_STREAM3, DMA_SxCR_CHSEL_0);
    nvic_set_priority(NVIC_DMA1_STREAM3_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM3_IRQ);
    CS_ON_CLR();
    
}

void dma1_stream3_isr()
{
    
    if(dma_get_interrupt_flag(DMA1,DMA_STREAM3,DMA_TCIF))
    {
    	dma_clear_interrupt_flags(DMA1, DMA_STREAM3, DMA_TCIF);
    	CS_OFF_SET();
    }
    
   // usart_print_text("Transfer complete");
    
}

static void tftlcdsetup(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); //Pin rd -  B0
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1); //Pin wr -  B1
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5); //Pin rs -  B5
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6); //Pin cs -  B6
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7); //Pin rst - B7

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);  //Pin 0 - C0
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);  //Pin 1 - C1
	
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);  //Pin 2 - C2
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);  //Pin 3 - C3
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);  //Pin 4 - C4
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);  //Pin 5 - C5

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6); //Pin 6 - C6
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);  //Pin 7 - C7
}


static void sd_card_setup(void)
{

	//Pin sck -   B13
	//Pin miso -  B14
	//Pin mosi -  B15
	//Pin CS -  A10


	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);

	//rcc_periph_clock_enable(RCC_GPIOB | RCC_GPIOC);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN,
			GPIO13 | GPIO14 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO14 | GPIO15);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
				GPIO13 | GPIO15);

	/* Chip select line */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);
	//CS_OFF_SET();
	

	rcc_periph_clock_enable(RCC_SPI2);
	spi_reset(SPI2);


	spi_set_bidirectional_mode(SPI2);
	spi_set_full_duplex_mode(SPI2);
	spi_enable_software_slave_management(SPI2);
	spi_set_nss_high(SPI2);
	//spi_enable_rx_dma(SPI2);

	spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);


	spi_enable(SPI2);
	CS_ON_CLR();


}

static void usart_setup(void)
{
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}




void msdelay(int x)
{
	int j;
	const int z=30000*x;	//technically 45000

	for (j = 0; j < z ; j++)	
		__asm__("nop");
	
}


void usart_print_num(unsigned int x)
{
	
	int dig;
	int len = floor(log10(x));
	
	while(len>=0)
	{
		dig=x/((int)pow(10,len));
		usart_send_blocking(USART2, dig + '0');
		x=x- dig*(int)pow(10,len);
		len--;

	}
	usart_send_blocking(USART2, '\t');
}


void usart_print_num2(unsigned int num)
{
	int cnt=0, numc=num;
	if(num==0)
		cnt=1;
	else
	{
		while(numc>0)
		{
			cnt++;
			numc/=10;
		}
	}
	char *arr;
	arr=(char *)malloc(sizeof(char)* (cnt+1) );
	*(arr + cnt)='\0';
	while(cnt>0)
	{
		*(arr + cnt-1)=48+num%10;
		num/=10;
		cnt--;
	}
	usart_print_text(arr);
}


void usart_print_bin(int arr[])
{
	
	int c=7;
	int val;
	while(c>=0)
	{
		val=arr[c];
		usart_send_blocking(USART2, val + '0');
		c--;
	}

	usart_send_blocking(USART2, '\t');
	
}

void usart_print_text(char *arr)
{
	
	int len=strlen(arr);
	int c=0;
	char ch;
	while(c<len)
	{
		ch=arr[c];
		usart_send_blocking(USART2, ch);
		c++;
	}

	usart_send_blocking(USART2, '\t');
	
}

void hex_to_arr(unsigned int x)
{
	int c=0;
	int arr[8];
	while(c<8)
	{
		arr[c]=x&1;
		x=x>>1;
		c++;
	}
	send_data(arr);
}
		


void Write_Command(unsigned int c)
{
	cs_on;
    rs_off;
    rd_off;
    wr_off;

    #ifdef DEBUG
    usart_send_blocking(USART2, 'c');
	usart_send_blocking(USART2, ' ');
	usart_print_num(c);
	#endif

	hex_to_arr(c);
	
	wr_on;
	wr_off;
	cs_off;
	
}

void Write_Data_Word(unsigned int c)
{
	cs_on;
    rs_on;
    rd_off;
    wr_off;

    #ifdef DEBUG
    usart_send_blocking(USART2, 'd');
	usart_send_blocking(USART2, '1');
	usart_send_blocking(USART2, ' ');
	usart_print_num(c>>8);
	#endif

	hex_to_arr(c>>8);
	wr_on;
	wr_off;	


	#ifdef DEBUG
	usart_send_blocking(USART2, 'd');
	usart_send_blocking(USART2, '2');
	usart_send_blocking(USART2, ' ');
	usart_print_num(c);	
	#endif

	hex_to_arr(c);
	wr_on;
	wr_off;
	cs_off;
	
}

void Write_Data_Byte(unsigned int c)
{
	cs_on;
    rs_on;
    rd_off;
    wr_off;

	#ifdef DEBUG
	usart_send_blocking(USART2, 'd');
	usart_send_blocking(USART2, '2');
	usart_send_blocking(USART2, ' ');
	usart_print_num(c);	
	#endif

	hex_to_arr(c);
	wr_on;
	wr_off;
	cs_off;
	
}

void Write_Command_Data(unsigned int cmd, unsigned int dat)
{
	Write_Command(cmd);
	Write_Data_Word(dat);
}

void Lcd_Init()
{
	rst_on;
    msdelay(20);	
	rst_off;
	msdelay(150);
	//rst_on;
	//msdelay(15);

	Write_Command(0x3A);
	Write_Data_Byte(0x55);
	Write_Command(0x11);
	Write_Command(0x29);
	Write_Command(0x36);
	Write_Data_Byte(0x28);


}


void SetXY(unsigned int x0,unsigned int x1,unsigned int y0,unsigned int y1)
{

	Write_Command(0x2A);
	Write_Data_Word(x0);
	Write_Data_Word(x1);
	Write_Command(0x2B);
	Write_Data_Word(y0);
	Write_Data_Word(y1);
	Write_Command(0x2C); 
}



void LCD_clear()
{
    unsigned int i,j;
	SetXY(0,319,0,239);
	#if 1
	for(i=0;i<320;i++)
	{
	    for(j=0;j<240;j++)
		{    
          	Write_Data_Word(0x0000);
		}
	}
	#endif
}

int Paint(FIL file_h)
{
	int i,j,k=1,m;
	SetXY(0,319,0,239);
	unsigned char ptr[2];
	ptr[1]='\0';
	unsigned char *ptr2;
	ptr2=(unsigned char *)0x20000;
	uint8_t x, y;
	unsigned int bytes_ret;
	unsigned char memtest[640];


	//for(i=0;i<70;i++)
	//f_read(&file_h, ptr2, 1, &bytes_ret);

	#if 0	//Read 1 byte at a time

    for(i=0;i<240;i++)
	{
		for (j=0;j<320;j++)
	   	{
		    f_read(&file_h, ptr, 1, &bytes_ret);
		    x=(int)ptr[0];
		    //usart_print_num2(x);
		    f_read(&file_h, ptr, 1, &bytes_ret);
		    y=(int)ptr[0];
	        Write_Data_Word( (uint16_t)((y * 0x100) + x)    );
		   	
	    }
	 }	
	 #endif

#if 1	//Read 241 frames, 1 whole line (640 bytes=320 pixels at a time)
for(m=0;m<241;m++)
{
	for(i=0;i<240;i++)
	{
		f_read(&file_h, memtest, 640, &bytes_ret);
		*(memtest + bytes_ret)='\0';
		k=0;
		for (j=0;j<320;j++)
	   	{
		    //x=(int)memtest[k];
		    //y=(int)memtest[k+1] ;
		    Write_Data_Word( (uint16_t)(( ( (int)memtest[k+1] ) * 0x100) + ( (int)memtest[k] ) )    );
		    k+=2;
	    }
	 }	
}
#endif



#if 0	//Copy image to flash memory and then read - not working


f_read(&file_h, ptr2, 153600, &bytes_ret);
*(ptr2 + bytes_ret)='\0';

for(i=0;i<240;i++)
{
	//k=0;
	for (j=0;j<320;j++)
	{
		//x=(int)memtest[k];
		//y=(int)memtest[k+1] ;
		Write_Data_Word( (uint16_t)(( ( (int)ptr2[k+1] ) * 0x100) + ( (int)ptr2[k] ) )    );
		k+=2;
	}
}	

#endif




	
}


void send_data(int val[])
{

	#ifdef DEBUG
	usart_send_blocking(USART2, 'b');
	usart_print_bin(val);
	#endif

	if(val[0])
	gpio_set(GPIOC, GPIO0);
	else
	gpio_clear(GPIOC, GPIO0);


	if(val[1])
	gpio_set(GPIOC, GPIO1);
	else
	gpio_clear(GPIOC, GPIO1);


	if(val[2])
	gpio_set(GPIOC, GPIO2);
	else
	gpio_clear(GPIOC, GPIO2);


	if(val[3])
	gpio_set(GPIOC, GPIO3);
	else
	gpio_clear(GPIOC, GPIO3);


	if(val[4])
	gpio_set(GPIOC, GPIO4);
	else
	gpio_clear(GPIOC, GPIO4);


	if(val[5])
	gpio_set(GPIOC, GPIO5);
	else
	gpio_clear(GPIOC, GPIO5);


	if(val[6])
	gpio_set(GPIOC, GPIO6);
	else
	gpio_clear(GPIOC, GPIO6);


	if(val[7])
	gpio_set(GPIOC, GPIO7);
	else
	gpio_clear(GPIOC, GPIO7);

}





int main(void)
{

	gpio_setup();
	usart_setup();

	usart_print_text("Begin");


	tftlcdsetup();
	sd_card_setup();
	dma_init();
	Lcd_Init();
	
	msdelay(100);



#if 1

	FATFS fstest;      
	FIL file_h; 
	FRESULT fileStatus;                        
    char fsid[2];


    fsid[0] = '0';
    fsid[1] = '\0';
    f_mount(&fstest, fsid, 0);


	//f_mkdir("/sdp/anwesh");
	f_mkdir("mars");		//max character length = 8?
								//max limit one directory at a time; a new sub-directory of length 8 can only be created if the parent directory exists.

	fileStatus=f_open(&file_h, "testvid.rgb", FA_OPEN_EXISTING | FA_READ);
	if(fileStatus==FR_OK)
	{
		usart_print_text("file opened successfully");
	}
	else
	{
		usart_print_text("file open failed");
	}

	Paint(file_h);


#endif

	while(1)
	{
		gpio_set(GPIOA, GPIO5);
		msdelay(200);
		gpio_clear(GPIOA, GPIO5);
		msdelay(200);
	}

	return 0;
}


