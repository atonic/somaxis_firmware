//motor.c

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


#define motor_en 16
#define motor_pwm 19

#define motor_address 0xb4

void motor_pwm_enable(void)
{
	nrf_gpio_pin_set(motor_pwm);
}

void motor_pwm_disable(void)
{
	nrf_gpio_pin_clear(motor_pwm);
}

void motor_en_enable(void)
{
	nrf_gpio_pin_set(motor_en);
}

void motor_en_disable(void)
{
	nrf_gpio_pin_clear(motor_en);
}

void motor_init(void)
{
   nrf_gpio_cfg_output(motor_en);
	 nrf_gpio_cfg_output(motor_pwm);
	 motor_en_enable();
}




//0x16:  0x53,89,b6,
void motor_driver_LRA_init(void)
{
	motor_en_enable();
	i2c_change_init();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_send_data(0x00);
	wait_ack();
	i2c_stop();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x03);
	wait_ack();
	i2c_send_data(0x06);
	wait_ack();
	i2c_stop();
	
	/*
	
	//0x16:0x53,0x17:0x89
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x16);
	wait_ack();
	i2c_send_data(0x53);
	wait_ack();
	i2c_send_data(0x89);
	wait_ack();
	i2c_stop();
	*/
	//0x1a:0xb6
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x1a);
	wait_ack();
	i2c_send_data(0xb6);
	wait_ack();
	i2c_send_data(0x13);//1b
	wait_ack();
	i2c_send_data(0xf5);//1c
	wait_ack();
	i2c_send_data(0x80);//1d
	wait_ack();
	i2c_stop();
	
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x03);
	wait_ack();
	i2c_send_data(0x06);
	wait_ack();
	i2c_stop();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_send_data(0x00);
	wait_ack();
	i2c_stop();
	
	/*
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x1e);
	wait_ack();
	i2c_send_data(0x20);
	wait_ack();
	i2c_stop();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x0c);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_stop();
	*/
	
}

unsigned char motor_staus_read(void)
{
	unsigned char temp;
	motor_en_enable();
	i2c_change_init();
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x00);
	wait_ack();
	i2c_start();
	i2c_send_data(motor_address|i2c_read);
	wait_ack();
	temp=i2c_receive_data();
	i2c_stop();
	return temp;
}

unsigned char motor_start_up(void)
{
	
}



unsigned char motor_test(void)
{
	motor_en_enable();
	i2c_change_init();
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_send_data(0x00);
	i2c_stop();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x04);
	wait_ack();
	i2c_send_data(0x04);//4
	wait_ack();
	i2c_send_data(0x07);//5
	wait_ack();
	i2c_send_data(0x05);//6
	wait_ack();
	i2c_send_data(0x00);//7
	/*
	wait_ack();
	i2c_send_data(0xff);//8
	wait_ack();
	i2c_send_data(0xff);//9
	wait_ack();
	i2c_send_data(0xff);//a
	wait_ack();
	i2c_send_data(0xff);//b
	*/
	i2c_stop();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x0c);
	wait_ack();
	i2c_send_data(0x01);
	i2c_stop();
	
	
}

unsigned char motor_test_2(void)
{
	motor_pwm_disable();
	motor_en_enable();
	i2c_change_init();
	/*
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_send_data(0x00);
	i2c_stop();
	*/
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x16);
	wait_ack();
	i2c_send_data(0x53);//16
	wait_ack();
	i2c_send_data(0xa0);//17
	i2c_stop();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x1a);
	wait_ack();
	i2c_send_data(0xb6);//1a
	wait_ack();
	i2c_send_data(0x93);//1b
	wait_ack();
	i2c_send_data(0xf5);//1c
	wait_ack();
	i2c_send_data(0x80);//1d
	
	i2c_stop();
	
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_send_data(0x07);
	i2c_stop();
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x1e);
	wait_ack();
	i2c_send_data(0x20);
	i2c_stop();
	
	
	i2c_start();
	i2c_send_data(motor_address|i2c_write);
	wait_ack();
	i2c_send_data(0x0c);
	wait_ack();
	i2c_send_data(0x01);
	i2c_stop();
	
	motor_pwm_enable();
}