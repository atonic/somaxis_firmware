//system_flash_save

//
#include "25q128.h"

#define system_data_start_address 0x2000
#define system_data_end_adress 0xffffff

u32 system_flash_sum,system_flash_save_count;
unsigned char system_flash_percent;

//return :0:normal,1:flow and save part of it.2:flow, didn`t save any
unsigned char device_flash_dave(unsigned char *p,unsigned int datalong)
{
  if ((system_flash_save_count+system_data_start_address)>=system_data_end_adress)
  	{
  		system_flash_percent=100;
  		return 2;
  	}
  if ((system_flash_save_count+system_data_start_address+datalong)>system_data_end_adress)
  	{
  		w25q64_write(system_flash_save_count+system_data_start_address,p,system_data_end_adress-system_flash_save_count-system_data_start_address);
  		system_flash_save_count+=system_data_end_adress-system_flash_save_count-system_data_start_address;
  		system_flash_percent=100;
  		return 1;
  	}
  else
  	{
  		w25q64_write(system_flash_save_count+system_data_start_address,p,datalong);
  		system_flash_save_count+=datalong;
  		system_flash_percent=system_flash_save_count*100/system_flash_sum;
  		return 0;
  	}
}

void device_flash_init(void)
{
	system_flash_sum=system_data_end_adress-system_data_start_address;
	
}

unsigned char system_flash_percent_get(void)
{
	return system_flash_percent;
}

u32 system_flash_data_count_get(void)
{
	return system_flash_save_count;
}