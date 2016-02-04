//battery.c
//need i2c to read the ltc2942
#include "nrf.h"
#include "battery_motor.h"
#include "twi_master.h"
#include "twi_master_config.h"
#include "nrf_assert.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "I2C.h"
#include "boards.h"
#define ltc2942_address 0xc8
#define usb_plug_detect_io 19
#define charge_staus_io 16
#define battery_quality_full 37647//mah  200/42.5*q_lsb
#define q_lsb 42//uah
unsigned char ltc2942_staus;

unsigned char battery_voltage;
unsigned char battery_charge_flag;
unsigned int battery_quality;
unsigned char battery_quality_percent;
//01:charging
//02:usb plug

void battery_charge_finish_set(void);

void battery_charge_io_init(void)
{
	nrf_gpio_cfg_input(usb_plug_detect_io,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(charge_staus_io,NRF_GPIO_PIN_PULLUP);
}

unsigned char battery_charge_flag_deal(void)
{
	if (nrf_gpio_pin_read(usb_plug_detect_io))
		{
			battery_charge_flag|=0x01;//charging
			if (nrf_gpio_pin_read(charge_staus_io))
				{
					if (battery_charge_flag&0x02)
						{
							battery_charge_finish_set();
						}
					battery_charge_flag|=0x02;
				}
			else
				{
					battery_charge_flag&=0xfd;
				}
		}
	else
		{
			battery_charge_flag&=0xfc;
		}
	return 0;
}
unsigned char battery_init(void)
{
	//unsigned char temp;
	//twi_master_init();
	battery_charge_io_init();
	I2C_init();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_write);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_read);
	wait_ack();
	ltc2942_staus=i2c_receive_data();
	i2c_stop();
	battery_itc2942_set();
	return ltc2942_staus;
}

unsigned char battery_alert_init(void)
{
	I2C_init();
	i2c_start();
	i2c_send_data(0x19);
	wait_ack();
	ltc2942_staus=i2c_receive_data();
	i2c_stop();
	return ltc2942_staus;
}

unsigned int battery_voltage_get(void)
{
	unsigned int temp;
	I2C_init();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_write);
	wait_ack();
	i2c_send_data(0x08);
	wait_ack();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_read);
	wait_ack();
	temp=i2c_receive_data();
	send_ack();
	temp=temp<<8;
	temp+=i2c_receive_data();
	i2c_stop();
	temp=temp/1092;
	battery_voltage=temp;
	return temp;
}


unsigned int battery_quality_get(void)
{
	unsigned int temp;
	I2C_init();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_write);
	wait_ack();
	i2c_send_data(0x02);
	wait_ack();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_read);
	wait_ack();
	temp=i2c_receive_data();
	send_ack();
	temp=temp<<8;
	temp+=i2c_receive_data();
	i2c_stop();
	if (temp>(0xffff-battery_quality_full))
		{
			battery_quality=temp-(0xffff-battery_quality_full);
		}
	else
		{
			battery_quality=0;
		}
	if (battery_quality<=battery_quality_full)
		{
			temp=battery_quality*100/battery_quality_full;
		}
	else
		{
			temp=100;
		}
	battery_quality_percent=temp;
	return temp;
}

unsigned char battery_staus_get(void)
{
	unsigned char temp;
	I2C_init();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_write);
	wait_ack();
	i2c_send_data(0);
	wait_ack();
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_read);
	wait_ack();
	temp=i2c_receive_data();
	//send_ack();
	i2c_stop();
	
	return temp;
}

void battery_itc2942_set(void)
{
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_write);
	wait_ack();
	i2c_send_data(0x01);
	wait_ack();
	i2c_send_data(0xa2);//just voltage adc,M=16,CHARGE compelte signal,q_lsb=0.0053125,max=37647
	wait_ack();
	i2c_stop();
	
	return ;
	i2c_send_data(q_lsb>>8);
	wait_ack();
	i2c_send_data(q_lsb);
	wait_ack();
	i2c_stop();
}

//run in every 1 second
void battery_deal(void)
{
	static unsigned char run_count=60;
	if (run_count++>=20)
		{
			run_count=0;
			battery_voltage_get();
			battery_quality_get();
		}
	battery_charge_flag_deal();
}

void battery_charge_finish_set(void)
{
	return ;
	/*
	i2c_start();
	i2c_send_data(ltc2942_address|i2c_write);
	wait_ack();
	i2c_send_data(0x04);
	wait_ack();
	i2c_send_data(battery_quality_full>>8);
	wait_ack();
	i2c_send_data(battery_quality_full);
	wait_ack();
	i2c_stop();
	*/
}
unsigned char get_battery_flag(void)
{
	return battery_charge_flag;
}

unsigned char get_battery_voltage(void)
{
	return battery_voltage;
}

unsigned int get_battery_q(void)
{
	return battery_quality;
}

unsigned char get_battery_quality_percent(void)
{
	return battery_quality_percent;
}


#define battery_ad_pin 0
void battery_voltage_AD_init(void)
{
	NRF_ADC->CONFIG = battery_ad_pin;
}

#define PWM_OUTPUT_PIN_NUMBER 30
void motor_pwm_init(void)
{
	
	nrf_gpio_cfg_output(PWM_OUTPUT_PIN_NUMBER);
return;
    NRF_GPIO->OUT = 0x00000000UL;

    // Configure GPIOTE channel 0 to toggle the PWM pin state
	// @note Only one GPIOTE task can be connected to an output pin.
    nrf_gpiote_task_config(0, PWM_OUTPUT_PIN_NUMBER, \
                           NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
	// Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[0] match.
    NRF_PPI->CH[0].EEP = (uint32_t)(&NRF_TIMER1->EVENTS_COMPARE[0]);
    NRF_PPI->CH[0].TEP = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[0]);
    
    nrf_gpio_cfg_output(PWM_OUTPUT_PIN_NUMBER);
    
    NRF_PPI->CH[0].EEP = (uint32_t)(&NRF_TIMER2->EVENTS_COMPARE[0]);
    NRF_PPI->CH[0].TEP = (uint32_t)(&NRF_TIMER2->TASKS_STOP);
	
	  // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[1] match.
    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[1];
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[1];
    
    // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[2] match.
    NRF_PPI->CH[2].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[2];
    NRF_PPI->CH[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
    
    // Enable PPI channels 0-2.
    NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                    | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
                    //| (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos);
                 
	
	
	nrf_gpiote_task_config(0, PWM_OUTPUT_PIN_NUMBER, \
                           NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
	
	
                    
   
   NRF_TIMER1->MODE        = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE     = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER1->PRESCALER   = 6;//TIMER_PRESCALERS;

    // Clears the timer, sets it to 0.
    NRF_TIMER1->TASKS_CLEAR = 1;

    // Load the initial values to TIMER2 CC registers.
    NRF_TIMER1->CC[0] = 20;//MAX_SAMPLE_LEVELS + next_sample_get();
    NRF_TIMER1->CC[1] = 30;//MAX_SAMPLE_LEVELS;

    
                    
    // CC2 will be set on the first CC1 interrupt.
    //NRF_TIMER1->CC[2] = 0;
    //NRF_TIMER1->
    NRF_TIMER1->SHORTS         =2;
    
    NRF_TIMER1->TASKS_START    = 1;                    // Start timer.
    // Interrupt setup.
    //NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
    
    
    
    

    
}

unsigned char timer1_counter,timer_counter_data;
unsigned char timer1_event_deal(void)
{
	if ((NRF_TIMER2->EVENTS_COMPARE[1]) != 0)
		{
			timer1_counter++;
		}
	return timer1_counter;
}

unsigned char clear_timer1_count(void)
{
	timer_counter_data=timer1_counter;
	timer1_counter=0;
	return 0;
}
unsigned char get_timer_count_data(void)
{
	return timer_counter_data;
}


//******************************************************************************************************************************
//motor
unsigned char motor_enable_count;
unsigned char motor_run_type,motor_run_power;
void motor_start(unsigned char temp)
{
	motor_enable_count=temp;
}

void motor_enable (void)
{
	nrf_gpio_pin_set(PWM_OUTPUT_PIN_NUMBER);
}

void motor_disable(void)
{
	nrf_gpio_pin_clear(PWM_OUTPUT_PIN_NUMBER);
}


void motor_deal(void)
{
	if (motor_enable_count)
		{
			motor_enable_count--;
			motor_enable();
		}
	else
		{
			motor_disable();
		}
}


