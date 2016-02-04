//uart.c
//p42,23i/o
#include <stdint.h>
#include "stdio.h"
#include "compiler_abstraction.h"
#include "nrf51.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "simple_uart.h"
#include "app_error.h"
#include "uart.h"

#define uart_pin 23
#define cricket_uart_baudrate UART_BAUDRATE_BAUDRATE_Baud4800

unsigned char uart_flag;

void cricket_uart_tx_init(uint8_t pin_number)
{
	nrf_gpio_cfg_output(pin_number);
    

  NRF_UART0->PSELTXD = pin_number;
  

  
  NRF_UART0->BAUDRATE         = (cricket_uart_baudrate << UART_BAUDRATE_BAUDRATE_Pos);
  NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
  NRF_UART0->TASKS_STARTTX    = 1;
  NRF_UART0->TASKS_STARTRX    = 0;
  NRF_UART0->EVENTS_RXDRDY    = 0;
	
	uart_flag=0;
}

void cricket_uart_rx_init(uint8_t pin_number)
{
	
  nrf_gpio_cfg_input(pin_number, NRF_GPIO_PIN_PULLUP);  

  
  NRF_UART0->PSELRXD = pin_number;

 

  NRF_UART0->BAUDRATE         = (cricket_uart_baudrate << UART_BAUDRATE_BAUDRATE_Pos);
  NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
  NRF_UART0->TASKS_STARTTX    = 1;
  NRF_UART0->TASKS_STARTRX    = 1;
  NRF_UART0->EVENTS_RXDRDY    = 0;
  NRF_UART0->CONFIG=0;
	
	uart_flag=1;
}


void uart_init(void)
{
	cricket_uart_tx_init(uart_pin);
	//simple_uart_put(0xaa);
}



unsigned char uart_send_string_data(unsigned char *p, unsigned int datalong)
{
	//simple_uart_put(0xaa);
}

unsigned char uart_send_single_data(unsigned char send_data)
{//cricket_uart_tx_init(uart_pin);
	simple_uart_put(send_data);
	//cricket_uart_rx_init( uart_pin);
}
/*
int fputc(int ch,FILE *f)
{
	simple_uart_put(ch);
	//while (0);
	return (ch);
}


int GetKey(void)
{
	
}*/


unsigned char receive_flag(void)
{
	if (NRF_UART0->EVENTS_RXDRDY == 1)
		{
			NRF_UART0->EVENTS_RXDRDY = 0;
      return 1;
		}
	return 0;
}

unsigned char get_uart_receive_data(void)
{
	return NRF_UART0->RXD;
}

unsigned char usart_send_chardata_to_hexascii(uint32_t send_data)
{
  unsigned char temp;
  if (send_data>>28)
  	{
  temp=(send_data>>28)&0x0f;
	if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			}
		if (send_data>>24)
			{
			temp=(send_data>>24)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			}
		
		if (send_data>>20)
			{
				temp=(send_data>>20)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			}
			
			if (send_data>>16)
			{
			temp=(send_data>>16)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			}
			
			if (send_data>>12)
			{
				temp=(send_data>>12)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			}
			
			if (send_data>>8)
			{
			temp=(send_data>>8)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			}
			
			//if (send_data>>24)
			{
	    temp=(send_data>>4)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			}
			temp=(send_data)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
}

unsigned char usart_send_chardata_to_decascii(uint32_t send_data)
{
    unsigned char temp;
    /*if (send_data>9999)
		{
			temp=(send_data/10000)%10;
			uart_send_single_data(temp+'0');
		}
		if (send_data>9999)
		{
			temp=(send_data/10000)%10;
			uart_send_single_data(temp+'0');
		}
		if (send_data>9999)
		{
			temp=(send_data/10000)%10;
			uart_send_single_data(temp+'0');
		}*/
		if (send_data>9999)
		{
			temp=(send_data/10000)%10;
			uart_send_single_data(temp+'0');
		}
	if (send_data>999)
		{
			temp=(send_data/1000)%10;
			uart_send_single_data(temp+'0');
		}
	if (send_data>99)
		{
			temp=(send_data/100)%10;
			uart_send_single_data(temp+'0');
		}
	
	if (send_data>9)
		{
			temp=(send_data/10)%10;
			uart_send_single_data(temp+'0');
		}
	temp=send_data%10;
	uart_send_single_data((temp)+'0');
			
}