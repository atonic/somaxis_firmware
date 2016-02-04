//afe.h
#include "nrf_delay.h"
#include "spi_master_config.h"
#include "spi_master.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
void afe_init();
void afe_reset(void);
void afe_start();
void afe_stop();
void afe_config();
int afe_has_data();
int32_t afe_read();
void afe_power_off(void);
unsigned char afe_deal(void);
unsigned char afe_flag_get(void);
unsigned char *afe_data_get(void);
int32_t afe_bigest_different_data_get(void);
