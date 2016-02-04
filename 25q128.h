//#w25q64.h
typedef unsigned           int u32; 



unsigned char * w25q64_read_unique_id(unsigned char *p);
void w25q64_write_enable(void);
void w25q64_write_disable(void);
unsigned char w25q64_write(u32 write_address,unsigned char *p,u32 datalong);
unsigned char * w25q64_read(u32 write_address,unsigned char *p,u32 datalong);
void w25q64_init(void);
unsigned char w25q64_long_program(u32 write_address,unsigned char *p,u32 datalong);
void w25q64_erase_chip(void);
unsigned char w25q64_music_write_factory_ready(u32 datalong,u32 offset,u32 end_offset);
unsigned char w25q64_music_writing_factory(unsigned char *p,u32 datalong);

#define system_id_address_start 0x00
#define system_id_address_datalong 0x10

#define system_user_data_adress_start 0x1000
#define system_user_data_long 0x20

