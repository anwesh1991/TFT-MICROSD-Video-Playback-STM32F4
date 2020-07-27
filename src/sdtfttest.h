#include <libopencm3/stm32/gpio.h>





void msdelay(const int);
void usart_print_num(unsigned int);
void usart_print_bin(int[]);
void usart_print_text(char[]);
void usart_print_num2(unsigned int);
void hex_to_arr(unsigned int);
void Write_Command(unsigned int);
void Write_Data_Byte(unsigned int);
void Write_Data_Word(unsigned int);
void Write_Command_Data(unsigned int, unsigned int);
void Lcd_Init(void);
void LCD_clear(void);
void SetXY(unsigned int, unsigned int, unsigned int, unsigned int);

void send_data(int val[]);



#define CS_ON_CLR()       gpio_clear(GPIOA, GPIO10)    /* MMC CS = L */	//For setting CS on, send low bit
#define CS_OFF_SET()      gpio_set(GPIOA, GPIO10)      /* MMC CS = H */	//For setting CS off, send high bit
#define rd_off 		gpio_set(GPIOB, GPIO0)
#define rd_on 		gpio_clear(GPIOB, GPIO0)
#define wr_off 		gpio_set(GPIOB, GPIO1)
#define wr_on 		gpio_clear(GPIOB, GPIO1)
#define rs_on 		gpio_set(GPIOB, GPIO5)
#define rs_off 		gpio_clear(GPIOB, GPIO5)
#define cs_off 		gpio_set(GPIOB, GPIO6)
#define cs_on 		gpio_clear(GPIOB, GPIO6)
#define rst_off 	gpio_set(GPIOB, GPIO7)
#define rst_on 		gpio_clear(GPIOB, GPIO7)


#define X_CONST 240
#define Y_CONST 320
