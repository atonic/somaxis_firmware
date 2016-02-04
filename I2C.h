//i2c.h


#define i2c_read 1
#define i2c_write 0



void I2C_init(void);
void i2c_start(void);
void i2c_stop(void);
unsigned char send_ack(void);
unsigned char wait_ack(void);
unsigned char i2c_send_data(unsigned char i2c_send_data_reg);
unsigned char i2c_receive_data(void);
//unsigned char I2C_send_address(unsigned char address_temp,unsigned char I2C_Direction_temp);

