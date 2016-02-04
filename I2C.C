//I2C.C

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
#include "I2C.h"
#define sda_pin 17
#define scl_pin 18

#define SCL_read      nrf_gpio_pin_read(scl_pin)
#define SDA_read      nrf_gpio_pin_read(sda_pin)

#define I2C_SDA_input_config()  nrf_gpio_cfg_input(sda_pin,NRF_GPIO_PIN_PULLUP)
#define I2C_SDA_output_config() nrf_gpio_cfg_output(sda_pin)


#define I2C_SCL_set()   nrf_gpio_pin_set(scl_pin)//GPIOB->BSRR = GPIO_Pin_6
#define I2C_SCL_reset() nrf_gpio_pin_clear(scl_pin)//GPIOB->BRR  = GPIO_Pin_6
#define I2C_SDA_set()   nrf_gpio_pin_set(sda_pin)//GPIOB->BSRR = GPIO_Pin_7
#define I2C_SDA_reset() nrf_gpio_pin_clear(sda_pin)//GPIOB->BRR  = GPIO_Pin_7

#define I2C_DELAY_TEMP 200

unsigned char i2c_use_flag;//01: 正在使用I2C，
//unsigned char sda_pin=17;
//unsigned char scl_pin=18;
//this is the 24c16
void I2C_init(void)
{
	//sda_pin=17;
	//scl_pin=18;
    nrf_gpio_cfg_output(scl_pin);
	  nrf_gpio_cfg_output(sda_pin);
	/*
	NRF_GPIO->PIN_CNF[16] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);   

    NRF_GPIO->PIN_CNF[17] =      \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);
	*/
    I2C_SCL_set();
    I2C_SDA_set();
    
   
}

void i2c_change_init(void)
{
	//sda_pin=18;
	//scl_pin=17;
	nrf_gpio_cfg_output(scl_pin);
	nrf_gpio_cfg_output(sda_pin);
	I2C_SCL_set();
  I2C_SDA_set();
}

unsigned char i2c_use_staus()
{
	return (i2c_use_flag);
}


void i2c_start(void)
{
	unsigned char I2C_delay;
	I2C_SDA_output_config();
	I2C_SCL_set();
	I2C_SDA_set();
	I2C_delay=120;
	while (I2C_delay--);
  I2C_SDA_reset();
}

void i2c_stop(void)
{
	unsigned char  I2C_delay;
	I2C_SDA_output_config();
	I2C_SDA_reset();
	I2C_delay=40;
	while (I2C_delay--);
	I2C_SCL_set();
	I2C_delay=80;
	//while (I2C_delay--);
	//I2C_SDA_reset();
	//I2C_delay=80;
	while (I2C_delay--);
	I2C_SDA_set();
	I2C_delay=250;
	while (I2C_delay--);
}

unsigned char i2c_send_data(unsigned char i2c_send_data_reg)
{
	unsigned char  i2c_send_data_i;
	volatile unsigned char  I2C_delay;
	I2C_SDA_output_config();
	for (i2c_send_data_i=0;i2c_send_data_i<8;i2c_send_data_i++)
	{
		I2C_delay=100;
	  while (I2C_delay--);
		I2C_SCL_reset();
		
		I2C_delay=100;
	  while (I2C_delay--);
		
		if (i2c_send_data_reg&0x80)
			{
				I2C_SDA_set();
			}
		else
			{
				I2C_SDA_reset();
			}
		
		I2C_delay=100;
	  while (I2C_delay--);
		I2C_SCL_set();
		
		i2c_send_data_reg=i2c_send_data_reg<<1;
	}
	I2C_delay=200;
	while (I2C_delay--);
	I2C_SCL_reset();
	I2C_SDA_set();
	return 0;
}

unsigned char  i2c_receive_data(void)
{
	unsigned char  i2c_send_data_i;
	volatile unsigned char  I2C_delay;
	unsigned char i2c_receive_data_reg;
	
    I2C_SDA_set();
	
	  I2C_SDA_input_config();
    for (i2c_send_data_i=0;i2c_send_data_i<8;i2c_send_data_i++)
	{
		i2c_receive_data_reg=i2c_receive_data_reg<<1;
		I2C_delay=120;
	  while (I2C_delay--);
		I2C_SCL_reset();
		
			
		
		I2C_delay=120;
	  while (I2C_delay--);
		I2C_SCL_set();
		if (SDA_read)
			{
				i2c_receive_data_reg|=0x01;
			}
		
	}
	I2C_SCL_reset();
	return (i2c_receive_data_reg);
}

unsigned char wait_ack(void)
{
	unsigned char  wait_ack_delay_limit=0;
	unsigned char  I2C_delay;
	unsigned char temp=0;
    I2C_SDA_set();
	  I2C_SDA_input_config();
    I2C_SCL_reset();
   // I2C_SDA_set();
    I2C_delay=120;
    while (I2C_delay--);
    
	  I2C_SCL_set();
	  I2C_delay=5;
    while (I2C_delay--);
    //I2C_SDA_set();
    while (SDA_read)//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9))
    {
    	wait_ack_delay_limit++;
    	if (wait_ack_delay_limit>200)
    		{
    			temp=1;
    			break;
    		}
    }
    I2C_delay=80;
	  while (I2C_delay--);
    I2C_SCL_reset();
    I2C_delay=80;
	  while (I2C_delay--);
	  
   return (temp);
}
unsigned char send_ack(void)
{
	unsigned char  I2C_delay;
    I2C_SDA_output_config();
    I2C_SCL_reset();
    I2C_delay=20;
	  while (I2C_delay--);
    I2C_SDA_reset();
    I2C_delay=80;
	  while (I2C_delay--);
    I2C_SCL_set();
    I2C_delay=80;
	  while (I2C_delay--);
	  I2C_SCL_reset();
	  I2C_SDA_set();
	  I2C_delay=80;
	  while (I2C_delay--);
	  
	  return 0;
}

/*
void i2c_stop(void)
{
	unsigned int I2C_delay;
	scl_pin_out();
	sda_pin_out();
	I2C_SDA_reset();
	I2C_delay=I2C_DELAY_TEMP;
	while (I2C_delay--);
	I2C_SCL_set();
	
	I2C_delay=I2C_DELAY_TEMP;
	while (I2C_delay--);
	I2C_SDA_set();
	I2C_delay=I2C_DELAY_TEMP;
	while (I2C_delay--);
}


void i2c_start(void)
{
	
	
	unsigned int I2C_delay;
	
	I2C_delay=I2C_DELAY_TEMP*3;
	while (I2C_delay--);
	scl_pin_out();
	sda_pin_out();
	
	
	I2C_SCL_set();
	I2C_SDA_set();
	I2C_delay=I2C_DELAY_TEMP*3;
	while (I2C_delay--);
  I2C_SDA_reset();
  I2C_delay=I2C_DELAY_TEMP;
	while (I2C_delay--);
	I2C_SCL_reset();
	sda_pin_in();
	  I2C_SDA_set();
}



unsigned char i2c_send_data(unsigned char i2c_send_data_reg)
{
	uint8_t i2c_send_data_i;
	unsigned int I2C_delay;
	scl_pin_out();
	sda_pin_out();
	
	for (i2c_send_data_i=0;i2c_send_data_i<8;i2c_send_data_i++)
	{
		I2C_delay=I2C_DELAY_TEMP;
	  while (I2C_delay--);
		I2C_SCL_reset();
		
		I2C_delay=I2C_DELAY_TEMP/2;
	  while (I2C_delay--);
		if (i2c_send_data_reg&0x80)
			{
				I2C_SDA_set();
			}
		else
			{
				I2C_SDA_reset();
			}
		
		I2C_delay=I2C_DELAY_TEMP/2;
	  while (I2C_delay--);
		I2C_SCL_set();
		
		i2c_send_data_reg=i2c_send_data_reg<<1;
	}
	I2C_delay=I2C_DELAY_TEMP;
	while (I2C_delay--);
	I2C_SCL_reset();
	sda_pin_in();
	  I2C_SDA_set();
	
	return 0;
}

uint8_t i2c_receive_data(void)
{
	
	uint8_t i2c_send_data_i;
	unsigned int I2C_delay;

	unsigned char i2c_receive_data_reg;
	
    
    scl_pin_out();
	  sda_pin_in();
    I2C_SDA_set();
    for (i2c_send_data_i=0;i2c_send_data_i<8;i2c_send_data_i++)
	{
		i2c_receive_data_reg=i2c_receive_data_reg<<1;
		I2C_delay=I2C_DELAY_TEMP/2;
	  while (I2C_delay--);
		I2C_SCL_reset();
		
		
		
			
		
		I2C_delay=I2C_DELAY_TEMP/2;
	  while (I2C_delay--);
		I2C_SCL_set();
		if (SDA_read)
			{
				i2c_receive_data_reg|=0x01;
			}
		
	}
	I2C_delay=I2C_DELAY_TEMP;
	  while (I2C_delay--);
	I2C_SCL_reset();
	sda_pin_in();
	  I2C_SDA_set();
	return (i2c_receive_data_reg);
}

unsigned char wait_ack(void)
{
	unsigned int wait_ack_delay_limit=0;
	unsigned int I2C_delay;
	unsigned char temp;
	
    
	  scl_pin_out();
	  I2C_SCL_reset();
	  sda_pin_in();
	  I2C_SDA_set();
	  
    
    I2C_delay=I2C_DELAY_TEMP/2;
    while (I2C_delay--);
    I2C_SCL_reset();
	  I2C_delay=I2C_DELAY_TEMP;
    while (I2C_delay--);
	  
    
    while (SDA_read)
    {
    	wait_ack_delay_limit++;
    	if (wait_ack_delay_limit>600)
    		{
    	
	  i2c_stop();
	  return 0;//1;
    			break;
    		}
    }
    
    I2C_SCL_set();
    I2C_delay=I2C_DELAY_TEMP;
	  while (I2C_delay--);
	  I2C_SCL_reset();
	  sda_pin_in();
	  I2C_SDA_set();
	  
   return 0;
}

unsigned char send_ack(void)
{
	unsigned int I2C_delay;
	
    scl_pin_out();
	sda_pin_out();
    I2C_SCL_reset();
    I2C_delay=I2C_DELAY_TEMP;
	  while (I2C_delay--);
    I2C_SDA_reset();
    I2C_delay=I2C_DELAY_TEMP;
	  while (I2C_delay--);
    I2C_SCL_set();
    I2C_delay=I2C_DELAY_TEMP;
	  while (I2C_delay--);
	  I2C_SCL_reset();
	  sda_pin_in();
	  I2C_SDA_set();
	  I2C_delay=I2C_DELAY_TEMP;
	  
	  while (I2C_delay--);
	  
	  return 0;
}
*/


