//acc.h

void accel_stop();
void accel_gyro_read(void);
void accel_init();
void accel_config();
void accel_start();
int accel_has_data();
unsigned int g_accel_flag_get(void);
void acc_power_off(void);
unsigned char acc_deal(void);
unsigned char *acc_data_get(void);

