//w25q64.c
//****************************************************************************************
//采用W25Q128做为FLASH最大存储器，此IC为128Mbit,16MB的FLASH,所以，它分为几个区，前面的4MB
//用来做为出厂的音乐保存，这个基本上不会动的。
//因为这个片子最小单位擦除为4KB，所以把最前面的4KB做为信息存储用。
//接下来的的4MB里面的剩下的为出厂音乐保存用。
//0~3fffff:音乐存储1区。
//400000~7fffff:音乐存储2区。
//f00000~ffffff:系统参数保存区。
//
//
//
//
//
//
//*********************************************************************************************

//#include "includes.h"
//#include "platform.h"
//#include "spi.h"
#include "25q128.h"
#include "nrf_gpio.h"
#include "spi_master_config.h"
#include "spi_master.h"

#define flash_cs_pin 29
#define flash_CLK    22
#define flash_MISO   28
#define flash_MOSI   24
#define flash_RST    25

#define w25q64_cs_enable() nrf_gpio_pin_clear(flash_cs_pin)
#define w25q64_cs_disable() nrf_gpio_pin_set(flash_cs_pin)



uint32_t *flash_spi;

static void w25q64_delay(void)
{
	unsigned int temp=0x0ff;
	while (temp--);
}

unsigned char system_extflash_data_init(void)
{
//	unsigned char buf[5];
//	w25q64_read(system_data_address,buf,1);
//	system_music_location=buf[0];
}

unsigned char spi_1_communication(unsigned char transfer_data)
{
	unsigned char temp;
	u32 counter=0;
	NRF_SPI_Type *spi_base = (NRF_SPI_Type *)NRF_SPI1;
	spi_base->TXD = transfer_data;

        /* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
        while ((spi_base->EVENTS_READY == 0U) && (counter < TIMEOUT_COUNTER))
        {
            counter++;
        }
        
        
spi_base->EVENTS_READY = 0U;
        temp = (uint8_t)spi_base->RXD;
				
		return temp;
}

unsigned char spi_1_communication_1(unsigned char transfer_data)
{
	unsigned char i=8;
	unsigned char temp=0;
	nrf_gpio_pin_clear(flash_CLK);
	while(i--)
	{
		temp=temp<<1;
		nrf_gpio_pin_clear(flash_CLK);
		
		if (nrf_gpio_pin_read(flash_MISO))
			{
				temp|=1;
			}
		nrf_gpio_pin_set(flash_CLK);
		if (transfer_data&0x80)
			{
				nrf_gpio_pin_set(flash_MOSI);
			}
		else
			{
				nrf_gpio_pin_clear(flash_MOSI);
			}
	}
	nrf_gpio_pin_clear(flash_CLK);
	return temp;
}
uint32_t* flash_spi_master_init(void)
{
    uint32_t config_mode;
SPIMode mode=3;
    NRF_SPI_Type *spi_base_address =(NRF_SPI_Type *)NRF_SPI1;

    
    {
        /* Configure GPIO pins used for pselsck, pselmosi, pselmiso and pselss for SPI1*/
        nrf_gpio_cfg_output(flash_CLK);
        nrf_gpio_cfg_output(flash_MOSI);
        nrf_gpio_cfg_input(flash_MISO, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_output(flash_cs_pin);

        /* Configure pins, frequency and mode */
        spi_base_address->PSELSCK  = flash_CLK;
        spi_base_address->PSELMOSI = flash_MOSI;
        spi_base_address->PSELMISO = flash_MISO;
        nrf_gpio_pin_set(flash_cs_pin);         /* disable Set slave select (inactive high) */
    }

    spi_base_address->FREQUENCY = 0x80000000UL << (uint32_t)Freq_125Kbps;

    switch (mode )
    {
        /*lint -e845 -save // A zero has been given as right argument to operator '!'" */
        case SPI_MODE0:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case SPI_MODE1:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case SPI_MODE2:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        case SPI_MODE3:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        default:
            config_mode = 0;
            break;
        /*lint -restore */
    }
    //if (lsb_first)
    {
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
     //   spi_base_address->CONFIG = (config_mode | (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos));
    }
    //else
    {
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        spi_base_address->CONFIG = (config_mode | (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos));
    }

    spi_base_address->EVENTS_READY = 0U;

    /* Enable */
    spi_base_address->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

    return (uint32_t *)spi_base_address;
}

void flash_spi_master_simulate_init(void)
{
	nrf_gpio_cfg_output(flash_CLK);
        nrf_gpio_cfg_output(flash_MOSI);
        nrf_gpio_cfg_input(flash_MISO, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_output(flash_cs_pin);
}

void w25q64_init(void)
{
	u32 temp;
	
	nrf_gpio_cfg_output(flash_RST);//rstn
	nrf_gpio_pin_set(flash_RST);
	
	flash_spi = flash_spi_master_init();
	//flash_spi_master_simulate_init();
	//spi_master_init(SPI1, SPI_MODE3, false);
}

unsigned char w25q_read_busy_staus(void)
{
	unsigned char temp;
	w25q64_cs_enable();
	spi_1_communication(0x05);//fast read
	temp=spi_1_communication(0x00);
	w25q64_cs_disable();
	return (temp&0x01);
}

unsigned char w25q_get_sector(u32 start_address,u32 datalong)
{
	   return 0;
}
unsigned char * w25q64_read(u32 write_address,unsigned char *p,u32 datalong)
{
	unsigned int i;
	flash_spi_master_init();
	while (w25q_read_busy_staus())
	{
		w25q64_delay();
	}
	w25q64_cs_enable();
	spi_1_communication(0x0b);//fast read
	spi_1_communication(write_address>>16);
	spi_1_communication(write_address>>8);
	spi_1_communication(write_address);
	spi_1_communication(0xff);
	for (i=0;i<datalong;i++)
	{
		*(p+i)=spi_1_communication(0x00);
	}
	w25q64_cs_disable();
	return (p);
}

unsigned char w25q64_page_write(u32 write_address,unsigned char *p,u32 datalong)
{
	unsigned int i;
	//w25q64_cs_enable();
	while (w25q_read_busy_staus())
	{
		w25q64_delay();
	}
	w25q64_write_enable();
	//w25q_get_sector(write_address,datalong);
	w25q64_cs_enable();
	spi_1_communication(0x02);//02:page program
	spi_1_communication(write_address>>16);
	spi_1_communication(write_address>>8);
	spi_1_communication(write_address);
	//spi_1_communication(0x00);
	for (i=0;i<datalong;i++)
	{
		spi_1_communication(*(p+i));
	}
	w25q64_cs_disable();
	//while (w25q_read_busy_staus())
	//w25q64_write_disable();
	return 0;
}

unsigned char * w25q64_read_unique_id(unsigned char *p)
{
	unsigned int i;
	w25q64_cs_enable();
	spi_1_communication(0x9f);//fast read
	
	for (i=0;i<4;i++)
	{
		*(p+i)=spi_1_communication(0xff);
	}
	w25q64_cs_disable();
	return (p);
}

void w25q64_write_enable(void)
{
	w25q64_cs_enable();
	spi_1_communication(0x06);//enable
	w25q64_cs_disable();
}

void w25q64_write_disable(void)
{
	w25q64_cs_enable();
	spi_1_communication(0x04);//disable
	w25q64_cs_disable();
}

void w25q64_erase_chip(void)
{
	while (w25q_read_busy_staus());
	w25q64_write_enable();
	w25q64_cs_enable();
	spi_1_communication(0xc7);//fast read
	//temp=spi_1_communication(0x00);
	w25q64_cs_disable();
}

unsigned char w25q64_long_program(u32 write_address,unsigned char *p,u32 datalong)//in one sector
{
//	u32 address_page_end;
	u32 page_temp;
	u32 datalong_temp;
	if ((write_address/256)==((write_address+datalong)/256))//检查PAGE的位置，如果在同一个PAGE里面
		                                                     //则直接写入，不然的话就需要分PAGE
		{
			w25q64_page_write(write_address,p,datalong);
		}
	else//需要分区
		{
			while(datalong)
			{
			  page_temp=write_address/256;
			  if (datalong>=256)//not in a page
			  	{
			  		 datalong_temp=(page_temp+1)*256-write_address;
			  	}
			  else
			  	{
			  		datalong_temp=datalong;
			  	}
			  w25q64_page_write(write_address,p,datalong_temp);
			  datalong-=datalong_temp;
			  write_address+=datalong_temp;
			  p+=datalong_temp;
		  }
		}
	return 0;
}

unsigned char sector_erase(u32 erase_address)//erase 4KB
{
	while (w25q_read_busy_staus())
	{
		w25q64_delay();
	}
	while (w25q_read_busy_staus());
	w25q64_write_enable();
	while (w25q_read_busy_staus());
	w25q64_cs_enable();
	spi_1_communication(0x20);//sector erase
	spi_1_communication(erase_address>>16);
	spi_1_communication(erase_address>>8);
	spi_1_communication(erase_address);
	w25q64_cs_disable();
	while (w25q_read_busy_staus())
	{
		w25q64_delay();
	}
	w25q64_write_disable();
	while (w25q_read_busy_staus())
	{
		w25q64_delay();
	}
}

unsigned char w25q64_write(u32 write_address,unsigned char *p,u32 datalong)
{
	//w25q64_write_enable();
	u32 temp;
	flash_spi_master_init();
	while(datalong)
	{
		if ((write_address&0xfff)==0)//起始地址在sector 节点上，那么需要sector erase
			{
				//sector_erase(write_address);//1st erase
			}
		if (((write_address+datalong)>>12)>(write_address>>12))//跨越了多个sector
			{
				temp=0x1000-(write_address&0xfff);
				w25q64_long_program(write_address,p,temp);
				datalong-=temp;//0x1000-(write_address&0xfff);
				write_address=(write_address&0xfffff000)+0x1000;
				p+=temp;
			}
		else//page write,1st should be erase,and then write in
			{
				w25q64_long_program(write_address,p,datalong);
				datalong=0;
			}
	}
	//w25q64_write_disable();
}

unsigned char w25q64_erase_whole_chip(void)
{
	while (w25q_read_busy_staus())
	{
		w25q64_delay();
	}
	w25q64_write_enable();
	//w25q_get_sector(write_address,datalong);
	w25q64_cs_enable();
	spi_1_communication(0xc7);//02:page program
	//spi_1_communication(write_address>>16);
	//spi_1_communication(write_address>>8);
	//spi_1_communication(write_address);
	//spi_1_communication(0x00);
	
	w25q64_cs_disable();
}

//******************************************************************************************



u32 music_sum_datalong;
u32 music_start_write_address,music_write_end_address;


unsigned char w25q64_music_write_factory_ready(u32 datalong,u32 offset,u32 end_offset)
{
	u32 i,j;
	music_sum_datalong=datalong;
	music_start_write_address=offset;
	music_write_end_address=end_offset;
	//need clear the flash data
	/*
  for ()
  {
  	sector_erase(write_address);//1st erase
  }
  */
	return 0;
}



unsigned char w25q64_music_writing_factory(unsigned char *p,u32 datalong)//judge over or not
{
	if ((music_start_write_address+datalong)<=music_write_end_address)//not over the factory address
		{
			w25q64_write(music_start_write_address,p,datalong);
			music_start_write_address+=datalong;
			if (music_sum_datalong<datalong)
				{
					return -1;
				}
			music_sum_datalong-=datalong;
			if (music_sum_datalong==0)
				{
					return 0;
				}
			return 1;
		}
	else
		{
			return -1;
		}
}

unsigned char w25q64_music_read_test(u32 read_address,unsigned char *p,u32 datalong)
{
//	w25q64_read(music_factory1_start_address,p,datalong);
	return 0;
}


unsigned char w25q64_test(void)
{
	unsigned char temp,i;
	unsigned char flash_test_data[10];
	//w25q64_read_unique_id(flash_test_data);
	//uart_send_single_data(flash_test_data[0]);
	//return 0;
	w25q64_read(0x00,flash_test_data,10);
	uart_send_single_data(flash_test_data[0]);
	//if (flash_test_data[0]==0xff)
	{
		for (temp=0;temp<10;temp++)
		{
			flash_test_data[temp]+=temp+1;
		}
		w25q64_write(0x00,flash_test_data,10);
	}
}


unsigned char get_in_deep_powerdown_mode(void)
{
	w25q64_init();
	
}

unsigned char release_deep_powerdown_mode(void)
{
	unsigned int delay_count;
	
	
}

//write the special data in the special ram in the first time.
//other time, just read the special ram and verify the data,if it is the same, return ok,if it not ok,return fault.
unsigned char w25q64_user_init(void)
{
	unsigned char user_character[]={"cricket"};
	unsigned char i,j,temp;
	unsigned char a_test[10];
	w25q64_init();
	w25q64_read(system_id_address_start,a_test,10);
	temp=sizeof(user_character);
	for (i=0;i<temp;i++)
	{
		if (a_test[i]!=user_character[i])
			{
				j=1;
				break;
			}
	}
  if (j)
  	{
  		w25q64_write(system_id_address_start,user_character,temp);
  		w25q64_read(system_id_address_start,a_test,10);
	    temp=sizeof(user_character);
	    for (i=0;i<temp;i++)
	    {
		    if (a_test[i]!=user_character[i])
			    {
				    return 1;
			    }
	    }
  	}
  return 0;
}

//save the main paraments
//1:acc rate,full_scale,acc_bandwith_filter:reserve 2byte
//2:gyro rate,full_scale,reserve 3byte
//3:exg


unsigned char system_user_data_init(void)
{
	unsigned char temp_arr[32];
	w25q64_read(system_user_data_adress_start,temp_arr,16);
	acc_parament_set(temp_arr);
	/*
	acc_rate_set(temp_arr[0]);
	acc_full_scale_set(temp_arr[1]);
	acc_bandwith_filter_set(temp_arr[2]);
	//
	//
	gyro_rate_set(temp_arr[5]);
	gyro_full_scale_set(temp_arr[6]);
	//
	//
	//
	*/
	afe_parament_set(&temp_arr[10]);
	/*
	afe_rate_set(temp_arr[0x0a]);
	afe_gain_set(temp_arr[0x0b]);
	afe_decimation_set(temp_arr[0x0c]);
	afe_databit_set(temp_arr[0x0d]);
	afe_mux_set(temp_arr[0x0e]);*/
}

unsigned char system_user_data_save(void)
{
	unsigned char temp_arr[32];
	w25q64_read(system_user_data_adress_start,temp_arr,16);
	temp_arr[0]=acc_rate_get();
	temp_arr[1]=acc_full_scale_get();
	temp_arr[2]=acc_bandwith_filter_get();
	//
	//
	temp_arr[5]=gyro_rate_get();
	temp_arr[6]=gyro_full_scale_get();
	//
	//
	//
	temp_arr[0x0a]=afe_rate_get();
	temp_arr[0x0b]=afe_gain_get();
	temp_arr[0x0c]=afe_decimation_get();	
	temp_arr[0x0d]=afe_databit_get();
	temp_arr[0x0e]=afe_mux_get();
	
	
	w25q64_write(system_user_data_adress_start,temp_arr,16);
}

unsigned char flash_type_get(void)
{
	return 0;
}

unsigned char flash_remain_percent(void)
{
	return 0;
}

