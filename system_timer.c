//this is for the system timer
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "system_timer.h"
#define system_timer TIMER2
#define system_timer_interval 100

unsigned int system_timer_flag;//01:future,02:ms,04:10ms,08:100ms,10:1second,20:mintues,40:hour
unsigned int system_timer_ms,system_timer_year;
unsigned char system_timer_second,system_timer_mintues,system_timer_hours,system_timer_day,system_timer_month;
unsigned char system_timer_data[10];
typedef enum
{
  TIMER0 = 0,  /**< Timer 0 module, base address at 0x40008000. */
  TIMER1,      /**< Timer 1 module, base address at 0x40009000. */
  TIMER2       /**< Timer 2 module, base address at 0x4000A000. */
} timer_t;
static volatile NRF_TIMER_Type * timer_init(timer_t timer)
{
    volatile NRF_TIMER_Type * p_timer;
    //timer_t timer;
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }

    switch (timer)
    {
        case TIMER0:
            p_timer = NRF_TIMER0;
            break;

        case TIMER1:
            p_timer = NRF_TIMER1;
            break;

        case TIMER2:
            p_timer = NRF_TIMER2;
            break;

        default:
            p_timer = 0;
            break;
    }
    return p_timer;
}

unsigned char system_timer_start(void)//(timer_t timer, uint_fast16_t volatile number_of_ms)
{
	/*volatile NRF_TIMER_Type * p_timer = timer_init(TIMER2);

    if (p_timer == 0) 
    {
        while(true) 
        {
            // Do nothing.
        }
    }*/
    
	NRF_TIMER2->TASKS_STOP         = 1;                // Stop timer.
	NRF_TIMER2->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
	
	NRF_TIMER2->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    NRF_TIMER2->PRESCALER      = 4;                            // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    NRF_TIMER2->BITMODE        = TIMER_BITMODE_BITMODE_16Bit;  // 16 bit mode.
    NRF_TIMER2->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    
    // With 32 us ticks, we need to multiply by 31.25 to get milliseconds.
    NRF_TIMER2->CC[0]          = 1000;//system_timer_interval * 31;
    //NRF_TIMER2->CC[0]         += system_timer_interval / 4; 
    NRF_TIMER2->SHORTS         =1;
    NRF_TIMER2->TASKS_START    = 1;                    // Start timer.
    NRF_TIMER2->INTENSET    = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER2->SHORTS      = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NVIC_EnableIRQ(TIMER2_IRQn);
    /*
while (NRF_TIMER2->EVENTS_COMPARE[0] == 0)
		{
			//return   0;
		}
		NRF_TIMER2->EVENTS_COMPARE[0]  = 0;
    NRF_TIMER2->TASKS_STOP         = 1;                // Stop timer.
    */
}

void system_timer_stop(void)
{
	volatile NRF_TIMER_Type * p_timer = timer_init(system_timer);
	p_timer->EVENTS_COMPARE[0]  = 0;
	
	p_timer->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    p_timer->PRESCALER      = 0;                            // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    p_timer->BITMODE        = TIMER_BITMODE_BITMODE_16Bit;  // 16 bit mode.
    p_timer->TASKS_STOP         = 1;                // Stop timer.
    p_timer->TASKS_STOP         = 1;                // Stop timer.
}

unsigned char system_timer_flow_event(void)
{
	//volatile NRF_TIMER_Type * p_timer = timer_init(system_timer);
	unsigned char feed_code;
	if ((NRF_TIMER2->EVENTS_COMPARE[0]) == 0)
		{
			feed_code =  0;
		}
		else
			{
	//NRF_TIMER2->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    //NRF_TIMER2->PRESCALER      = 6;                            // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    //NRF_TIMER2->BITMODE        = TIMER_BITMODE_BITMODE_08Bit;  // 16 bit mode.
	NRF_TIMER2->EVENTS_COMPARE[0]  = 0;
	
		//NRF_TIMER2->TASKS_START    = 1;                    // Start timer.
		feed_code = 1;
	}
			return feed_code;
}



void system_timer_deal(void)
{
	if (system_timer_flow_event())
		{
			system_timer_flag|=0x02;
			system_timer_ms++;
			//uart_send_single_data(test_char);
			if ((system_timer_ms%10)==0)
				{
					system_timer_flag|=0x04;
				}
			if ((system_timer_ms%100)==0)
				{
					system_timer_flag|=0x08;
					motor_deal();
				}
			if (system_timer_ms>=1000)
			{
				system_timer_ms=0;
				system_timer_flag|=0x10;
				system_timer_second++;
				if (system_timer_second>=60)
					{
						system_timer_second=0;
						system_timer_mintues++;
						system_timer_flag|=0x20;
						if (system_timer_mintues>=60)
							{
								system_timer_mintues=0;
								system_timer_flag|=0x40;
								system_timer_hours++;
								if (system_timer_hours>=24)
									{
										system_timer_hours=0;
										system_timer_flag|=0x80;
									}
							}
					}
			}
		}
}


/** @brief Function for handling the Timer 2 interrupt.
 */
void TIMER2_IRQHandler(void)
{
	system_timer_deal();
}
unsigned char get_system_timer_ms_flag(void)
{
	if (system_timer_flag&0x02)
		{
			system_timer_flag^=0x02;
			return 1;
		}
	return 	 0;
}

unsigned char get_system_timer_10ms_flag(void)
{
	if (system_timer_flag&0x04)
		{
			system_timer_flag^=0x04;
			return 1;
		}
	return 	 0;
}

unsigned char get_system_timer_100ms_flag(void)
{
	if (system_timer_flag&0x08)
		{
			system_timer_flag^=0x08;
			return 1;
		}
	return 	 0;
}

unsigned char get_system_timer_ms(void)
{
	return system_timer_ms;
}


unsigned char get_system_timer_second_flag(void)
{
	if (system_timer_flag&0x10)
		{
			system_timer_flag^=0x10;
			return 1;
		}
	return 	 0;
}

unsigned char get_system_timer_second(void)
{
	return system_timer_second;
}

unsigned char system_timer_set(unsigned char *p)
{
	system_timer_year=p[0];
	system_timer_year=system_timer_year<<8;
	system_timer_year+=p[1];
	
	system_timer_month=p[2];
	
	system_timer_day=p[3];
	
	system_timer_hours=p[4];
	
	system_timer_mintues=p[5];
	
	system_timer_second=p[6];
	
	return 0;
}

unsigned char system_timer_get(unsigned char *p)
{
	p[0]=system_timer_year>>8;
	p[1]=system_timer_year;
	p[2]=system_timer_month;
	p[3]=system_timer_day;
	p[4]=system_timer_hours;
	p[5]=system_timer_mintues;
	p[6]=system_timer_second;
	//p=system_timer_data;
	//return system_timer_data;
} 

unsigned char *system_timer_character_get(void)
{
	unsigned char temp_data[20];
	temp_data[0]=0;
}