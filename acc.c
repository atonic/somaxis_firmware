//acc

/***************************************************
Accel code
****************************************************/
#include "acc.h"
#include "nrf_delay.h"
#include "spi_master_config.h"
#include "spi_master.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"

#define ACCEL_PWR   //8
#define ACCEL_CLK  12
#define ACCEL_MISO 10
#define ACCEL_MOSI 11
#define ACCEL_CSN  13
#define ACCEL_INT1  9
#define ACCEL_INT2 14

#define ACCEL_REG_WHO_AM_I  0x0F

#define GYRO_REG_OUT_X_L   0x22 // data rate, low power mode
#define ACCEL_REG_CTRL_REG2   0x21 // high pass, click
#define ACCEL_REG_CTRL_REG3   0x22 // interrupts
#define ACCEL_REG_CTRL_REG4   0x23 // block data, endian, scale
#define ACCEL_REG_CTRL_REG5   0x24
#define ACCEL_REG_STATUS_REG  0x27
#define ACCEL_REG_OUT_X_L     0x28
#define ACCEL_FIFO_CTRL_REG   0x2E

#define ACCEL_READ            0x80
#define ACCEL_WRITE           0x00
#define ACCEL_INCREMENT_ADDR  0x40

#define ACCEL_FIFO_SIZE 1

// CTRL_REG1
#define ACCEL_SET_SPS_OFF     0x00
#define ACCEL_SET_SPS_1       0x10
#define ACCEL_SET_SPS_10      0x20
#define ACCEL_SET_SPS_25      0x30
#define ACCEL_SET_SPS_50      0x40
#define ACCEL_SET_SPS_100     0x50
#define ACCEL_SET_SPS_200     0x60
#define ACCEL_SET_SPS_400     0x70
// Low power 1600  0x80
#define ACCEL_SET_SPS_1250    0x90

#define ACCEL_SET_ALL_AXIS    0x07

// CTRL_REG4
#define ACCEL_SET_SCALE_2G    0x00
#define ACCEL_SET_SCALE_4G    0x10
#define ACCEL_SET_SCALE_8G    0x20
#define ACCEL_SET_SCALE_16G   0x30


//acc and gyro default set
#define acc_rate_default 3//26hz
#define acc_full_scale_default 0//16g
#define acc_bandwith_filter_default 4//auto
#define gyro_rate_default 4//104
#define gyro_full_scale_default 0//125

static uint8_t tx_data[30]; /*!< SPI TX buffer */
static uint8_t rx_data[30]; /*!< SPI RX buffer */


uint32_t *accel_spi;

#define ACCEL_MAX_DATA_LENGTH 5

//uint8_t accel_data[ ACCEL_MAX_DATA_LENGTH * 3 * 2 ];


uint8_t g_accel_sample_rate = ACCEL_SET_SPS_50;
uint8_t g_accel_scale = ACCEL_SET_SCALE_2G;


unsigned int accel_real_data[3],gypo_real_data[3];
unsigned char accel_real_short_data[12];
unsigned int accel_data[ACCEL_MAX_DATA_LENGTH][3],gypo_data[ACCEL_MAX_DATA_LENGTH][3];
int g_accel_started = 0;

unsigned char acc_rate=acc_rate_default;
unsigned char gyro_rate=gyro_rate_default;
unsigned char acc_full_scale=acc_full_scale_default;
unsigned char gyro_full_scale=gyro_full_scale_default;
unsigned char acc_bandwith_filter=acc_bandwith_filter_default;

unsigned char acc_set_flag;

//gyro_full_scale

void acc_spi_delay(unsigned char delay_time)
{
	//delay_time=delay_time*3;
	while (delay_time--);
}
unsigned char acc_spi_tranfit_1_byte(unsigned char tx_data_temp)
{
	unsigned char i=8;
	unsigned char temp=0;
	nrf_gpio_pin_set(ACCEL_CLK);
	while (i--)
	{
	temp=temp<<1;
	//acc_spi_delay(1);
	nop();
	nrf_gpio_pin_clear(ACCEL_CLK);
	if ((tx_data_temp>>i)&0x01)
		{
			nrf_gpio_pin_set(ACCEL_MOSI);
		}
	else
		{
			nrf_gpio_pin_clear(ACCEL_MOSI);
		}
	nop();//acc_spi_delay(1);
	nrf_gpio_pin_set(ACCEL_CLK);
	nop();//acc_spi_delay(1);
	if (nrf_gpio_pin_read(ACCEL_MISO))
		{
			temp=temp|1;
		}
	}
	return temp;
}
uint32_t* acc_spi_master_init()
{
    uint32_t config_mode;
SPIMode mode=3;
//return 0;
    NRF_SPI_Type *spi_base_address =(NRF_SPI_Type *)NRF_SPI1;

    
    {
        /* Configure GPIO pins used for pselsck, pselmosi, pselmiso and pselss for SPI1*/
        nrf_gpio_cfg_output(ACCEL_MOSI);
        nrf_gpio_cfg_output(ACCEL_CLK);
        nrf_gpio_cfg_input(ACCEL_MISO, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_output(ACCEL_CSN);
        nrf_gpio_pin_set(ACCEL_CSN); 
        nrf_gpio_pin_set(ACCEL_MOSI); 
        nrf_gpio_pin_set(ACCEL_CLK); 
//return 0;
        /* Configure pins, frequency and mode */
        spi_base_address->PSELSCK  = ACCEL_CLK;
        spi_base_address->PSELMOSI = ACCEL_MOSI;
        spi_base_address->PSELMISO = ACCEL_MISO;
        nrf_gpio_pin_set(ACCEL_CSN);         /* disable Set slave select (inactive high) */
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


unsigned char acc_spi_transfer_data(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data)
{
	uint32_t counter = 0;
    uint16_t number_of_txd_bytes = 0;
    uint32_t SEL_SS_PINOUT;
    acc_spi_master_init();
    NRF_SPI_Type *spi_base = (NRF_SPI_Type *)NRF_SPI1;
	nrf_gpio_pin_clear(ACCEL_CSN);

    while(number_of_txd_bytes < transfer_size)
    {
        spi_base->TXD = (uint32_t)(tx_data[number_of_txd_bytes]);

        /* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
        while ((spi_base->EVENTS_READY == 0U) && (counter < TIMEOUT_COUNTER))
        {
            counter++;
        }

        if (counter == TIMEOUT_COUNTER)
        {
            /* timed out, disable slave (slave select active low) and return with error */
            nrf_gpio_pin_set(SEL_SS_PINOUT);
            return false;
        }
        else
        {   /* clear the event to be ready to receive next messages */
            spi_base->EVENTS_READY = 0U;
        }

        rx_data[number_of_txd_bytes] = (uint8_t)spi_base->RXD;
        number_of_txd_bytes++;
    };

    /* disable slave (slave select active low) */
    nrf_gpio_pin_set(ACCEL_CSN);
		return 0;
}

unsigned char acc_spi_transfer_data_2(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data)
{
//	uint32_t counter = 0;
    uint16_t number_of_txd_bytes = 0;
//    uint32_t SEL_SS_PINOUT;
    //acc_spi_master_init();
//    NRF_SPI_Type *spi_base = (NRF_SPI_Type *)NRF_SPI0;
	nrf_gpio_pin_clear(ACCEL_CSN);

    while(number_of_txd_bytes < transfer_size)
    {
        rx_data[number_of_txd_bytes]=acc_spi_tranfit_1_byte(tx_data[number_of_txd_bytes]);
        number_of_txd_bytes++;
        
        
    };

    /* disable slave (slave select active low) */
    nrf_gpio_pin_set(ACCEL_CSN);
		return 0;
}
void accel_wreg(uint8_t reg, uint8_t val)
{
	/*
	if (reg <= 0x06 || (reg >= 0x10 && reg <= 0x1E) ||  (reg >= 0x34 && reg <= 0x37) )
	{
		// write to reserved registers, skip
		return;
	}*/
//	_clear_data();
	acc_spi_master_init();
	tx_data[0] = ACCEL_WRITE | reg;
	tx_data[1] = val;
	int result = acc_spi_transfer_data(accel_spi, 2, (const uint8_t *)tx_data, rx_data);
}

uint8_t accel_rreg(uint8_t reg)
{
//	_clear_data();
	acc_spi_master_init();
	tx_data[0] = ACCEL_READ | reg;
	int result = acc_spi_transfer_data(accel_spi, 2, (const uint8_t *)tx_data, rx_data);
	return rx_data[1];
}

void accel_test_start(void)
{
	accel_wreg(0x12,0x01);
	nrf_delay_ms(200);
	accel_wreg(0x18,0x38);
	accel_wreg(0x10,0xa4);
	accel_wreg(0x11,0x8c);
}


uint32_t* accel_io_init(void)
{
	
  return (spi_master_init(SPI1, SPI_MODE3, false));
}
void accel_init()
{
	//nrf_gpio_pin_dir_set(ACCEL_PWR,    NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(ACCEL_INT1,   NRF_GPIO_PIN_DIR_INPUT);
 
  //nrf_gpio_pin_clear(ACCEL_PWR);
  nrf_delay_ms(10);

	//nrf_gpio_pin_set(ACCEL_PWR);
  nrf_delay_ms(10);
  // Use SPI1, mode 3 with msb first
  accel_spi=acc_spi_master_init();
  //accel_test_start();
  if (accel_spi == 0)
  {
    return;
  }

}

void accel_config()
{
	unsigned char temp;
	/*
	//old one
	accel_wreg( ACCEL_REG_CTRL_REG1, g_accel_sample_rate | ACCEL_SET_ALL_AXIS ); // sample rate, normal (not low power), all axis
	accel_wreg( ACCEL_REG_CTRL_REG3, 0x10 ); // INT1 on DRDY1
	accel_wreg( ACCEL_REG_CTRL_REG4, g_accel_scale | 0x04 ); // full scale, high resolution mode 
	*/
	//accel_wreg(0x01,0x00);
	
	
	//accel_wreg(0x08,0x00);
	//accel_wreg(0x0a,0x00);
	
	acc_spi_master_init();
	accel_wreg(0x12,0x01);
	nrf_delay_ms(200);
	accel_wreg(0x12,0x04);
	accel_wreg(0x18,0x38);
	switch (acc_rate)
	{
		case 0:temp=0x00;break;//stop
			case 1:temp=0x10;break;//13
				case 2:temp=0x20;break;//26
					case 3:temp=0x30;break;//52
						case 4:temp=0x40;break;//104
							case 5:temp=0x50;break;//208
								case 6:temp=0x60;break;//416
									case 7:temp=0x70;break;//833
										case 8:temp=0x80;break;//1.66k
											case 9:temp=0x90;break;//3.33k
												case 10:temp=0xa0;break;//6.66k
													//case 9:temp=0x;break;
	}
	switch (acc_full_scale)
	{
		case 0:temp+=0x00;break;//2
			case 1:temp+=8;break;//4
				case 2:temp+=12;break;//8
					case 3:temp+=4;break;//16
						
		default :temp+=0;break;
	}
	switch (acc_bandwith_filter)
	{
		case 0:temp+=0x00;break;//400
			case 1:temp+=1;break;//200
				case 2:temp+=2;break;//100
					case 3:temp+=3;break;//50
						
		default :temp+=0;break;
	}
	accel_wreg(0x10,temp);//0x4c);//enable the acc
	//accel_wreg(0x1a,0x3f);
	temp=0;
	switch (gyro_rate)
	{
		case 0:temp=0x00;break;//stop
			case 1:temp=0x10;break;//13
				case 2:temp=0x20;break;//26
					case 3:temp=0x30;break;//52
						case 4:temp=0x40;break;//104
							case 5:temp=0x50;break;//208
								case 6:temp=0x60;break;//416
									case 7:temp=0x70;break;//833
										case 8:temp=0x80;break;//1.66k
													//case 9:temp=0x;break;
	}
	switch (gyro_full_scale)
	{
		case 0:temp+=0x02;break;//125
			case 1:temp+=0x00;break;//245
				case 2:temp+=0x04;break;//500
					case 3:temp+=0x08;break;//1000
						case 4:temp+=0x0c;break;//2000
	}
	accel_wreg(0x11,temp);//enable the gyroscope
	temp=0;
	if (acc_bandwith_filter==4)
		{
			temp=0x40;
		}
	accel_wreg(0x15,temp);
	if (acc_rate)
		{
			accel_wreg(0x0d,0x01);
		}
	else
		{
			accel_wreg(0x0d,0x00);
		}
	
	//nrf_delay_ms(200);
}

void accel_start()
{
	g_accel_started = 1;

	accel_config();
}

void accel_stop()
{
	g_accel_started = 0;
accel_spi=acc_spi_master_init();
//	accel_wreg( ACCEL_REG_CTRL_REG1, ACCEL_SET_SPS_OFF );
	accel_wreg(0x10,0x00);//enable the acc
	//accel_wreg(0x1a,0x3f);
	accel_wreg(0x11,0x00);//enable the gyroscope
}

void accel_gyro_read(void)
{
//accel_spi=acc_spi_master_init();
	//for (int i = 0; i < ACCEL_FIFO_SIZE; i++)
	//{
	acc_spi_master_init();
		//_clear_data();
		tx_data[0] = ACCEL_READ  | GYRO_REG_OUT_X_L;//| ACCEL_INCREMENT_ADDR
		acc_spi_transfer_data(accel_spi, 13, (const uint8_t *)tx_data, rx_data);
		accel_real_data[0]=(rx_data[8]<<8)+rx_data[7];
		accel_real_data[1]=(rx_data[10]<<8)+rx_data[9];
		accel_real_data[2]=(rx_data[12]<<8)+rx_data[11];
		
		gypo_real_data[0]=(rx_data[2]<<8)+rx_data[1];
		gypo_real_data[1]=(rx_data[4]<<8)+rx_data[3];
		gypo_real_data[2]=(rx_data[6]<<8)+rx_data[5];
		
		
		//memmove( &( accel_data[ accel_idx*6 ] ), &( rx_data[1] ), 6 );
	  //accel_idx ++;
	 // if (accel_idx > ACCEL_MAX_DATA_LENGTH) { accel_idx = 0; }
	//}

}


void accel_self_test(void)
{
}
int accel_has_data()
{
		return nrf_gpio_pin_read(ACCEL_INT1);
}

unsigned int g_accel_flag_get(void)
{
	return g_accel_started ;
}

void acc_power_off(void)
{
	accel_stop();
}

unsigned char accel_idx = 0;
unsigned char acc_deal(void)
{
	unsigned char i;
	if (acc_set_flag&0x01)
		{
			acc_set_flag&=0xfe;
			accel_start();
		}
	else
		{
	if ( g_accel_flag_get() && accel_has_data())
		{
			accel_gyro_read();
			
			
			
			if (accel_idx < ACCEL_MAX_DATA_LENGTH)
				{
					accel_data[accel_idx][0]=accel_real_data[0];
					accel_data[accel_idx][1]=accel_real_data[1];
					accel_data[accel_idx][2]=accel_real_data[2];
			
					gypo_data[accel_idx][0]=gypo_real_data[0];
					gypo_data[accel_idx][1]=gypo_real_data[1];
					gypo_data[accel_idx][2]=gypo_real_data[2];
			    accel_idx++;
				}
			else
				{
					for (i=0;i<(ACCEL_MAX_DATA_LENGTH-1);i++)
					{
						accel_data[i][0]=accel_data[i+1][0];
						accel_data[i][1]=accel_data[i+1][1];
						accel_data[i][2]=accel_data[i+1][2];
			
						gypo_data[i][0]=gypo_data[i+1][0];
						gypo_data[i][1]=gypo_data[i+1][1];
						gypo_data[i][2]=gypo_data[i+1][2];
					}
					accel_data[ACCEL_MAX_DATA_LENGTH-1][0]=accel_real_data[0];
					accel_data[ACCEL_MAX_DATA_LENGTH-1][1]=accel_real_data[1];
					accel_data[ACCEL_MAX_DATA_LENGTH-1][2]=accel_real_data[2];
			
					gypo_data[ACCEL_MAX_DATA_LENGTH-1][0]=gypo_real_data[0];
					gypo_data[ACCEL_MAX_DATA_LENGTH-1][1]=gypo_real_data[1];
					gypo_data[ACCEL_MAX_DATA_LENGTH-1][2]=gypo_real_data[2];
				}
			
			/*
						 			  if (accel_idx >= ACCEL_MAX_DATA_LENGTH)
						 			  {
//						 			  	data[0] = accel_seq;
//						 			  	accel_seq ++;
						 			  	for (int i = 0; i < ACCEL_MAX_DATA_LENGTH; i++)
						 			  	{
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+0] = accel_data[i][0]>>8;
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+1] = accel_data[i][0];
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+2] = accel_data[i][1]>>8;
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+3] = accel_data[i][1];
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+4] = accel_data[i][2]>>8;
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+5] = accel_data[i][2];
						 			  		//tx_data[i*ACCEL_MAX_DATA_LENGTH+3] = (accel_data[i][0] & 0xC0) | ((accel_data[i][1] & 0xC0) >> 2) | ((accel_data[i][2] & 0xC0) >> 4);							
						 			  	}
						 			  	for (int i = 0; i < ACCEL_MAX_DATA_LENGTH; i++)
						 			  	{
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+6] = gypo_data[i][0]>>8;
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+7] = gypo_data[i][0];
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+8] = gypo_data[i][1]>>8;
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+9] = gypo_data[i][1];
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+10] = gypo_data[i][2]>>8;
						 			  		tx_data[i*ACCEL_MAX_DATA_LENGTH+11] = gypo_data[i][2];
						 			  		//tx_data[i*ACCEL_MAX_DATA_LENGTH+9] = (gypo_data[i][0] & 0xC0) | ((gypo_data[i][1] & 0xC0) >> 2) | ((gypo_data[i][2] & 0xC0) >> 4);							
						 			  	}
//						 			  	if (g_is_connected )
						 			  		{
//						 			  			int result = ble_nus_send_accel_data_verison1(&m_nus, data, 17);
						 			  		}
						 			    accel_idx = 0;
						 			    return 1;
						 			  }
						 			  */
			
		}
	if (accel_idx)
		{
			return 1;
		}
		}
  return 0;
}

unsigned char acc_data_turn (void)
{
	unsigned char i;
	accel_idx-=1;
	for (i=0;i<accel_idx;i++)
	{
		accel_data[i][0]=accel_data[i+1][0];
						accel_data[i][1]=accel_data[i+1][1];
						accel_data[i][2]=accel_data[i+1][2];
			
						gypo_data[i][0]=gypo_data[i+1][0];
						gypo_data[i][1]=gypo_data[i+1][1];
						gypo_data[i][2]=gypo_data[i+1][2];
	}
	return accel_idx;
}


unsigned char *acc_data_get(void)
{
	accel_real_short_data[0]=accel_data[0][0]>>8;
		accel_real_short_data[1]=accel_data[0][0];
		accel_real_short_data[2]=accel_data[0][1]>>8;
		accel_real_short_data[3]=accel_data[0][1];
		accel_real_short_data[4]=accel_data[0][2]>>8;
		accel_real_short_data[5]=accel_data[0][2];
		
		accel_real_short_data[6]=gypo_data[0][0]>>8;
		accel_real_short_data[7]=gypo_data[0][0];
		accel_real_short_data[8]=gypo_data[0][1]>>8;
		accel_real_short_data[9]=gypo_data[0][1];
		accel_real_short_data[10]=gypo_data[0][2]>>8;
		accel_real_short_data[11]=gypo_data[0][2];
	return accel_real_short_data;
}

unsigned char get_acc_data(unsigned char *p)
{
	p[0]=accel_real_data[0];
	p[1]=accel_real_data[1];
	p[2]=accel_real_data[2];
	return 0;
}



unsigned char acc_rate_set(unsigned int temp)
{
	switch (temp)
	{
		case 0:acc_rate=0;break;
		case 1:acc_rate=1;break;
			case 10:acc_rate=2;break;
				case 25:acc_rate=3;break;
					case 50:acc_rate=4;break;
						case 100:acc_rate=5;break;
							case 200:acc_rate=6;break;
								case 400:acc_rate=7;break;
									case 1250:acc_rate=8;break;
					default:acc_rate=acc_rate_default;
	}
	//acc_rate=temp;
	/*
	if (acc_rate>10)
		{
			acc_rate=acc_rate_default;
		}*/
	return acc_rate;
}
unsigned char acc_rate_get(void)
{
	return acc_rate;
}

unsigned char acc_full_scale_set(unsigned char temp)
{
	switch (temp)
	{
		case 2:acc_full_scale=0;break;
			case 4:acc_full_scale=1;break;
				case 8:acc_full_scale=2;break;
					case 16:acc_full_scale=3;break;
						default :acc_full_scale=acc_full_scale_default;break;
	}
	/*
	acc_full_scale=temp;
	if (acc_full_scale>3)
		{
			acc_full_scale=acc_full_scale_default;
		}
		*/
	return acc_full_scale;
}
unsigned char acc_full_scale_get(void)
{
	return acc_full_scale;
}

unsigned char acc_bandwith_filter_set(unsigned char temp)
{
	acc_bandwith_filter=temp;
	if (acc_bandwith_filter>4)
		{
			acc_bandwith_filter=acc_bandwith_filter_default;
		}
	return acc_bandwith_filter;
}

unsigned char acc_bandwith_filter_get()
{
	return acc_bandwith_filter;
}
unsigned char gyro_rate_set(unsigned char temp)
{
	gyro_rate=temp;
	if (gyro_rate>8)
		{
			gyro_rate=gyro_rate_default;
		}
	return gyro_rate;
}
unsigned char gyro_rate_get(void)
{
	return gyro_rate;
}

unsigned char gyro_full_scale_set(unsigned char temp)
{
	gyro_full_scale=temp;
	if (gyro_full_scale>4)
		{
			gyro_full_scale=gyro_rate_default;
		}
	return gyro_full_scale;
}
unsigned char gyro_full_scale_get(void)
{
	return gyro_full_scale;
}



void acc_parament_set(unsigned char *temp_arr)
{
	//if (temp_arr[0])
		{
	acc_rate_set(temp_arr[1]*256+temp_arr[2]);
	acc_full_scale_set(temp_arr[3]);
    }
    /*
  else
  	{
  		acc_rate_set(0);
  		gyro_rate_set(0);
  	}*/
	//acc_bandwith_filter_set(temp_arr[2]);
	//
	//
	//gyro_rate_set(temp_arr[5]);
	//gyro_full_scale_set(temp_arr[6]);
	//
	//
	//
	/*
	afe_rate_set(temp_arr[0x0a]);
	afe_gain_set(temp_arr[0x0b]);
	afe_decimation_set(temp_arr[0x0c]);
	afe_databit_set(temp_arr[0x0d]);
	afe_mux_set(temp_arr[0x0e]);*/
	acc_set_flag|=0x01;
	//accel_start();
}

