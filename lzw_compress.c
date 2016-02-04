//lzw compress 
//write by xhgao  2015/12/23
#include "stdint.h"
#include <stdio.h>
#include <stdlib.h>

#define lzw_compress_clear 256
#define lzw_compress_end   257
#define compress_table_max_limit 20//3839
#define compressed_max_data_count 100
typedef uint32_t u32;

unsigned int compress_source_data_count;
unsigned int compressed_data_sum_count,compressed_data_sendout_count;

//lzw_compress_table_type *lzw_compress_table ;//_at_ 0x20004000;
uint16_t compressed_data[compressed_max_data_count+20];


struct 
{
	u32 compress_pre;
	unsigned int compress_bak;
	unsigned int compress_output;
	unsigned int source_data_count;
	unsigned int source_data_multi_count;
	unsigned int compressed_data_count;
	unsigned int compressed_data_multi_count;
	unsigned int compressed_frame_data_count;//保存在缓存的数据
}lzw_compress_parament;


//the default 0~255 is in the table anytime
struct 
{
  uint16_t string_table[compress_table_max_limit][2];
  u32 table_num_max;
  unsigned int string_table_real_data;
} lzw_compress_table __attribute__((at(0X20004000)));// __at (0x20004000);//@0x20004000;

//unsigned char t __attribute__((at(0X20003000)));




//compress 
void lzw_compress_init(void)
{
	//lzw_compress_table=(lzw_compress_table_type *)malloc (sizeof(lzw_compress_table_type));
	
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	simple_uart_putstring("lzw compress malloc data size:");
	lzw_compress_parament.source_data_count=0;
	lzw_compress_parament.source_data_multi_count=0;
	lzw_compress_parament.compressed_data_count=0;
	lzw_compress_parament.compressed_data_multi_count=0;
	lzw_compress_parament.compressed_frame_data_count=0;
	lzw_compress_table.table_num_max=0;
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	
}
unsigned int lzw_compress_output_deal(void)
{
	//lzw_compress_parament.compressed_data_count++;
	lzw_compress_parament.compress_output=lzw_compress_parament.compress_pre;
	return (lzw_compress_parament.compress_pre);
}


unsigned char lzw_compress_clear_deal(void)
{
	
  compressed_data[lzw_compress_parament.compressed_frame_data_count]=lzw_compress_parament.compress_pre;
							lzw_compress_parament.compressed_data_count++;
							lzw_compress_parament.compressed_data_multi_count++;
							lzw_compress_parament.compressed_frame_data_count++;
				  		compressed_data[lzw_compress_parament.compressed_frame_data_count]=lzw_compress_clear;
							lzw_compress_parament.compressed_data_count++;
							lzw_compress_parament.compressed_data_multi_count++;
							lzw_compress_parament.compressed_frame_data_count++;
	compress_source_data_count=lzw_compress_parament.source_data_count;
	compressed_data_sum_count=lzw_compress_parament.compressed_data_count;
	lzw_compress_parament.source_data_count=0;						
	lzw_compress_parament.compressed_data_count=0;
	lzw_compress_table.table_num_max=0;
  return 0;
}

u32 lzw_compress_end_deal(void)
{
	lzw_compress_parament.compressed_data_count+=2;
	lzw_compress_parament.compressed_data_multi_count+=2;
	return lzw_compress_parament.compress_pre;
}

u32 lzw_compress_long_end_deal(void)
{
	compressed_data[lzw_compress_parament.compressed_data_count++]=lzw_compress_parament.compress_pre;
	lzw_compress_parament.compressed_data_multi_count++;
	compressed_data[lzw_compress_parament.compressed_data_count++]=lzw_compress_end;
	lzw_compress_parament.compressed_data_multi_count++;
	compressed_data_sum_count=lzw_compress_parament.compressed_data_multi_count;
	lzw_compress_parament.compressed_data_multi_count=0;
	lzw_compress_parament.compressed_data_count=0;
	
	compress_source_data_count=lzw_compress_parament.source_data_multi_count;
	lzw_compress_parament.source_data_count=0;
	lzw_compress_parament.source_data_multi_count=0;
	return 0;
}
// judge the pre+bak is in the table
//output 0:not output any data
//       1:output one data
//       2:output one data and clear
//in this function, just update the lzw_compress_table,
//                  can`t change any lzw_compress_parament
unsigned char lzw_compress_judge(void)
{
	unsigned int i;
	u32 temp=0;
	
	for (i=0;i<lzw_compress_table.table_num_max;i++)
	{
		if (lzw_compress_parament.compress_pre==lzw_compress_table.string_table[i][0])
			{
				if (lzw_compress_parament.compress_bak==lzw_compress_table.string_table[i][1])
					{
						temp=1;
						break;
					}
			}
	}
	if (temp==0)//new table and output
		{
			//new table
			lzw_compress_table.string_table[lzw_compress_table.table_num_max][0]=lzw_compress_parament.compress_pre;
			lzw_compress_table.string_table[lzw_compress_table.table_num_max][1]=lzw_compress_parament.compress_bak;
			lzw_compress_table.table_num_max++;
			//output
			lzw_compress_output_deal();
			if (lzw_compress_table.table_num_max>=compress_table_max_limit)//need clear
				{
					//lzw_compress_clear_deal();
					return 2;
				}
			return 1;
		}
	else
		{
			lzw_compress_table.string_table_real_data=i+258;
		}
	return 0;
	
}


//in this function, just update the lzw_compress_parament  ,
//                  can`t change any lzw_compress_table
unsigned int lzw_compress_long(unsigned char *data_temp,uint32_t compress_datalong)
{
	unsigned char temp;
	unsigned char i;
	unsigned char bak_flag=0;
	for (i=0;i<compress_datalong;i++)
	{
	if (lzw_compress_parament.source_data_count==0)//just start
		{
			lzw_compress_parament.compress_pre=data_temp[i];
			lzw_compress_parament.source_data_count=1;
			lzw_compress_parament.source_data_multi_count++;
			lzw_compress_table.table_num_max=0;
			lzw_compress_parament.compressed_data_count=0;
			//lzw_compress_parament.compressed_data_multi_count=0;
			lzw_compress_parament.compressed_frame_data_count++;
			//return 0;
		}
	else
		{
			lzw_compress_parament.source_data_count++;
			lzw_compress_parament.source_data_multi_count++;
			lzw_compress_parament.compress_bak=data_temp[i];
			temp=lzw_compress_judge();
			if (temp)//output
				{
					compressed_data[lzw_compress_parament.compressed_frame_data_count]=lzw_compress_parament.compress_output;
					lzw_compress_parament.compressed_data_count++;
					lzw_compress_parament.compressed_data_multi_count++;
					lzw_compress_parament.compressed_frame_data_count++;
					
					lzw_compress_parament.compress_pre=data_temp[i];
					//return 1;
				  if (temp==2)//clear
				  	{
				  		
							lzw_compress_clear_deal();
							bak_flag=1;
				  	}
				}
			else
				{
					lzw_compress_parament.compress_pre=lzw_compress_table.string_table_real_data;
				}
		}
	}
	
	if (lzw_compress_parament.compressed_frame_data_count>=compressed_max_data_count)
		{
			//lzw_compress_long_end_deal();
			//compressed_data_sendout_count=lzw_compress_parament.compressed_data_multi_count;
			
	    //lzw_compress_parament.compressed_data_count=0;
	    //compressed_data_sum_count+=
	    compressed_data_sendout_count=lzw_compress_parament.compressed_frame_data_count;
	    lzw_compress_parament.compressed_frame_data_count=0;
			bak_flag=2;//need send out
		}
	return bak_flag;
}


//in this function, just update the lzw_compress_parament  ,
//                  can`t change any lzw_compress_table
unsigned int lzw_compress(unsigned char data_temp)
{
	unsigned char temp;
	if (lzw_compress_parament.source_data_count==0)//just start
		{
			lzw_compress_parament.compress_pre=data_temp;
			lzw_compress_parament.source_data_count=1;
			lzw_compress_table.table_num_max=0;
			return 0;
		}
	else
		{
			lzw_compress_parament.source_data_count++;
			lzw_compress_parament.compress_bak=data_temp;
			temp=lzw_compress_judge();
			if (temp)//output
				{
					lzw_compress_parament.compressed_data_count++;
					lzw_compress_parament.compressed_data_multi_count++;
					lzw_compress_parament.compress_pre=data_temp;
					return 1;
				}
			else
				{
					lzw_compress_parament.compress_pre=lzw_compress_table.string_table_real_data;
				}
		}
		
	return 0;
}


unsigned int lzw_compress_end_get(void)
{
	return lzw_compress_end;
}

unsigned char test_lzw_compress(void)
{
	unsigned int temp;
	unsigned char i,j,k;
	unsigned char test_data[]={12,13,14,19,252,79,13,14,65,79,87,98,19,252,79,80,19,252,79,97};
	unsigned int test_compressed_data[30];
	simple_uart_putstring("lzw compress test");
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	simple_uart_putstring("lzw compress oral data:");
	temp=sizeof(test_data);
	for (i=0;i<temp;i++)
	{
		usart_send_chardata_to_decascii(test_data[i]);
		uart_send_single_data(',');
	}
	k=0;
	for (i=0;i<temp;i++)
		{
			j=lzw_compress(test_data[i]);
			if (j)
				{
					test_compressed_data[k++]=lzw_compress_parament.compress_output;
				}
		}
	test_compressed_data[k++]=lzw_compress_end_deal();
	test_compressed_data[k++]=lzw_compress_end_get();
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	simple_uart_putstring("lzw compressed data:");
	for (i=0;i<lzw_compress_parament.compressed_data_multi_count;i++)
	{
		usart_send_chardata_to_decascii(test_compressed_data[i]);
		uart_send_single_data(',');
	}
	
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	simple_uart_putstring("oral datacount:");
	usart_send_chardata_to_decascii(lzw_compress_parament.source_data_count);	
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	
	simple_uart_putstring("compressed datacount:");
	usart_send_chardata_to_decascii(lzw_compress_parament.compressed_data_multi_count);	
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	
	lzw_compress_init();
	lzw_compress_long(test_data,sizeof(test_data));
	lzw_compress_long_end_deal();
	simple_uart_putstring("long compressed datacount:");
	for (i=0;i<compressed_data_sum_count;i++)
	{
		usart_send_chardata_to_decascii(compressed_data[i]);
		uart_send_single_data(',');
	}
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	
  simple_uart_putstring("compressed datacount:");
	usart_send_chardata_to_decascii(compressed_data_sum_count);	
	uart_send_single_data(0x0d);
	uart_send_single_data(10);

	return 0;
}

u32 lzw_compress_source_count_get()
{
	return compress_source_data_count;
}

u32 lzw_compressed_count_get()
{
	return compressed_data_sum_count;
}

u32 lzw_compressed_sendout_datacount_get(void)
{
	return compressed_data_sendout_count;
}
