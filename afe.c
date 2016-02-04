/***************************************************
AFE code
****************************************************/


#include "afe.h"
#include "nrf_delay.h"
#include "spi_master_config.h"
#include "spi_master.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"






#define DELAY_MS               100        /*!< Timer Delay in milli-seconds */

#define AFE_DRDY  2
#define AFE_MISO  3
#define AFE_CLK   4
#define AFE_MOSI  5
#define AFE_CSN   6
#define AFE_START 7
#define AFE_PWDN_RST 8

#define AFE_CMD_WAKEUP  0x02
#define AFE_CMD_STANDBY 0x04
#define AFE_CMD_RESET   0x06
#define AFE_CMD_START   0x08
#define AFE_CMD_STOP    0x0A
#define AFE_CMD_RDATAC  0x10
#define AFE_CMD_SDATAC  0x11
#define AFE_CMD_RDATA   0x12
#define AFE_CMD_RREG0   0x20
#define AFE_CMD_WREG0   0x40

#define AFE_REG_CONFIG1   1
#define AFE_REG_CONFIG2   2
#define AFE_REG_LOFF      3
#define AFE_REG_CH1SET    4
#define AFE_REG_CH2SET    5
#define AFE_REG_RLDSENS   6
#define AFE_REG_LEADOFF_SENSE_SELECTION 7
#define AFE_REG_LEADOFF_STAUS 8
#define AFE_REG_RESP1     0x09
#define AFE_REG_RESP2     0x0A

#define AFE_SET_MUX_IN    0x00 // 0b0000
#define AFE_SET_MUX_SHORT 0x01 // 0b0001
#define AFE_SET_MUX_MVDD  0x03 // 0b0011
#define AFE_SET_MUX_TEMP  0x04 // 0b0100
#define AFE_SET_MUX_TEST  0x05 // 0b0101

#define AFE_SET_GAIN_1    0x10 // 0b00010000
#define AFE_SET_GAIN_2    0x20
#define AFE_SET_GAIN_3    0x30
#define AFE_SET_GAIN_4    0x40
#define AFE_SET_GAIN_6    0x00 // 0b00000000
#define AFE_SET_GAIN_8    0x50 // 0b01010000
#define AFE_SET_GAIN_12   0x60 // 0b01100000

#define AFE_SET_SPS_125   0x00
#define AFE_SET_SPS_250   0x01
#define AFE_SET_SPS_500   0x02
#define AFE_SET_SPS_1000  0x03
#define AFE_SET_SPS_2000  0x04
#define AFE_SET_SPS_4000  0x05
#define AFE_SET_SPS_8000  0x06

#define g_afe_sample_rate_default AFE_SET_SPS_2000
#define g_afe_gain_default 0x04//6
#define g_afe_mux_default 0x00
#define g_afe_decimate_default 0x00

#define g_afe_bits_default 16
#define g_afe_dataout_num_default 9 
#define AFE_MAX_DATA_LENGTH 18

    uint32_t err_code;
    int32_t afe_data[AFE_MAX_DATA_LENGTH];
		
		
		uint8_t data[20];


uint32_t *afe_spi;

uint8_t g_afe_leadoff_command;
uint8_t g_afe_leadoff_flag;
uint8_t g_afe_sample_rate = g_afe_sample_rate_default;
uint8_t g_afe_gain = g_afe_gain_default;
uint8_t g_afe_bits = g_afe_bits_default;
uint8_t g_afe_mux = AFE_SET_MUX_IN;
uint8_t g_afe_decimate = 0;
uint8_t g_afe_median = 0;

uint8_t g_afe_dataout_num = g_afe_dataout_num_default;
unsigned char g_afe_started=0;
//unsigned char afe_data[50];
unsigned char tx_data[20],rx_data[20];
unsigned char AFE_data_buffer[40];
unsigned char afe_set_flag;


int32_t afe_biggest_data=0;
int32_t afe_old_data=0;
int32_t afe_minus_data=0;


int afe_idx = 0;
__asm void nop(void)
{
  NOP
}

//#define afe_spi_cs 6
uint32_t* afe_spi_master_init(void)
{
    uint32_t config_mode;

    NRF_SPI_Type *spi_base_address =(NRF_SPI_Type *)NRF_SPI0;
SPIMode mode=1;
    //return 0;
    {
        /* Configure GPIO pins used for pselsck, pselmosi, pselmiso and pselss for SPI1*/
        nrf_gpio_cfg_output(AFE_MOSI);
        nrf_gpio_cfg_output(AFE_CLK);
        nrf_gpio_cfg_input(AFE_MISO, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_output(AFE_CSN);

        /* Configure pins, frequency and mode */
        spi_base_address->PSELSCK  = AFE_CLK;
        spi_base_address->PSELMOSI = AFE_MOSI;
        spi_base_address->PSELMISO = AFE_MISO;
        nrf_gpio_pin_set(AFE_CSN);         /* disable Set slave select (inactive high) */
    }

    spi_base_address->FREQUENCY = 0x40000000UL << (uint32_t)Freq_125Kbps;

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

uint32_t* afe_io_init(void)
{
	return (spi_master_init(SPI0, SPI_MODE1, false));
}

unsigned char afe_spi_transfer_data(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data)
{
	uint32_t counter = 0;
    uint16_t number_of_txd_bytes = 0;
    uint32_t SEL_SS_PINOUT;
    
    afe_spi_master_init();
    NRF_SPI_Type *spi_base =(NRF_SPI_Type *)NRF_SPI0;
	nrf_gpio_pin_clear(AFE_CSN);

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
            nrf_gpio_pin_set(AFE_CSN);
            simple_uart_putstring(" x,");
            return false;
        }
        else
        {   /* clear the event to be ready to receive next messages */
            spi_base->EVENTS_READY = 0U;
        }

        rx_data[number_of_txd_bytes] = (uint8_t)spi_base->RXD;
        number_of_txd_bytes++;
    };
    /*
    number_of_txd_bytes=255;
    while (number_of_txd_bytes--)
    {
    }
    */
    /* disable slave (slave select active low) */
    nrf_gpio_pin_set(AFE_CSN);
    return true;
}
void afe_init()
{
  nrf_gpio_pin_dir_set(AFE_START,    NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(AFE_PWDN_RST, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_dir_set(AFE_DRDY,     NRF_GPIO_PIN_DIR_INPUT);

	nrf_gpio_pin_clear(AFE_START);
	
	nrf_gpio_pin_set(AFE_PWDN_RST);
  nrf_delay_ms(1000);
	nrf_gpio_pin_clear(AFE_PWDN_RST);
  nrf_delay_ms(10);
	nrf_gpio_pin_set(AFE_PWDN_RST);
	nrf_gpio_pin_set(AFE_PWDN_RST);
	
  // Use SPI0, mode 1, msb first
  afe_spi = afe_spi_master_init();
  afe_config();
  if (afe_spi == 0)
  {
    return;
  }

}
void afe_reset(void)
{
	tx_data[0] = 0x06;
	
	int result = afe_spi_transfer_data (afe_spi, 1, (const uint8_t *)tx_data, rx_data);
	nrf_delay_ms(300);
	
}

void afe_sdatac(void)
{
	tx_data[0] = 0x11;
	
	int result = afe_spi_transfer_data (afe_spi, 1, (const uint8_t *)tx_data, rx_data);
}

void afe_rdatac(void)
{
	tx_data[0] = 0x10;
	
	int result = afe_spi_transfer_data (afe_spi, 1, (const uint8_t *)tx_data, rx_data);
}

int32_t afe_rdata_read(void)
{
	//afe_spi = afe_spi_master_init();
		//_clear_data();

		//while (nrf_gpio_pin_read(AFE_DRDY) == 1) { };
		tx_data[0] = 0x12;
		int result = afe_spi_transfer_data(afe_spi, 7, (const uint8_t *)tx_data, rx_data);
		
		int32_t value = (rx_data[4] << 16) | (rx_data[5] << 8) | rx_data[6];

		return value;
}
void afe_wreg(uint8_t reg, uint8_t val)
{
	
	//_clear_data();
	tx_data[0] = AFE_CMD_WREG0 + reg;
	tx_data[1] = 0;
	tx_data[2] = val;
	int result = afe_spi_transfer_data (afe_spi, 3, (const uint8_t *)tx_data, rx_data);//spi_master_tx_rx(afe_spi, 3, (const uint8_t *)tx_data, rx_data);
}

uint8_t afe_rreg(uint8_t reg)
{
	
	//_clear_data();
	
	tx_data[0] = AFE_CMD_RREG0 + reg;
	tx_data[1] = 0;
	int result = afe_spi_transfer_data(afe_spi, 3, (const uint8_t *)tx_data, rx_data);// spi_master_tx_rx(afe_spi, 3, (const uint8_t *)tx_data, rx_data);
	uint8_t val = rx_data[2];
	return val;
}


void afe_start()
{
	g_afe_started = 1;
	afe_spi = afe_spi_master_init();
	//_clear_data();
	//tx_data[0] = AFE_CMD_RDATAC;
	//int result = afe_spi_transfer_data(afe_spi, 1, (const uint8_t *)tx_data, rx_data);
  afe_sdatac();
	nrf_delay_ms(10);

	nrf_gpio_pin_set(AFE_START);
}

void afe_stop()
{
	g_afe_started = 0;
	afe_spi = afe_spi_master_init();
	nrf_gpio_pin_clear(AFE_START);
	nrf_delay_ms(10);
	
	//_clear_data();
	tx_data[0] = AFE_CMD_SDATAC;
	int result = afe_spi_transfer_data(afe_spi, 1, (const uint8_t *)tx_data, rx_data);
	
	nrf_delay_ms(10);
}

void afe_send_all_regs()
{
	int result;
afe_spi = afe_spi_master_init();
	//_clear_data();
	tx_data[0] = AFE_CMD_RREG0;
	tx_data[1] = 11; // num registers - 1
	result = afe_spi_transfer_data(afe_spi, 14, (const uint8_t *)tx_data, rx_data);
	
	// DEBUG
//	ble_nus_send_debug_data(&m_nus, rx_data, 14);
}

void afe_config(void)
{
	unsigned char temp,temp1;
	afe_spi = afe_spi_master_init();
	//afe_stop();
 // afe_reset();
  switch(g_afe_gain)
  {
  	case 0:temp=0x10;break;
  		case 1:temp=0x20;break;
  			case 2:temp=0x30;break;
  				case 3:temp=0x40;break;
  					case 4:temp=0x00;break;
  						case 5:temp=0x50;break;
  							case 6:temp=0x60;break;
  	default:temp=0x00;break;
  }
  
  switch(g_afe_mux)
  {
  	case 0:temp1=0x00;break;
  		case 1:temp1=0x01;break;
  			case 2:temp1=0x02;break;
  				case 3:temp1=0x03;break;
  					case 4:temp1=0x04;break;
  						case 5:temp1=0x05;break;
  	default:temp1=0x00;break;
  }
	afe_wreg(AFE_REG_CH1SET, temp |temp1  );
	afe_wreg(AFE_REG_CH2SET, 0x81 );
	switch(g_afe_sample_rate)
  {
  	case 0:temp=0x00;break;
  		case 1:temp=0x01;break;
  			case 2:temp=0x02;break;
  				case 3:temp=0x03;break;
  					case 4:temp=0x04;break;
  						case 5:temp=0x05;break;
  							case 6:temp=0x06;break;
  	default:temp=0x00;break;
  }
	afe_wreg(AFE_REG_CONFIG1, temp);
	if (temp1 == AFE_SET_MUX_TEST)
	{
		afe_wreg(AFE_REG_CONFIG2, 0xe3);  // value: 0b10100011 0xA3 VREF_4V=0 PDB_REFBUF=1 INT_TEST-1 TEST_FREQ=1 // 0xA0 to turn off test signal
	}
	else
	{
		afe_wreg(AFE_REG_CONFIG2, 0xe0);
	}
	afe_wreg(AFE_REG_RLDSENS, 0x13);  // value: chop freq default, RLD on, RLD ch1 connected to IN1P and IN1N
	
	afe_wreg(AFE_REG_LEADOFF_SENSE_SELECTION,0X13);
	afe_wreg(AFE_REG_RESP1, 0x02); // value: magic
	afe_wreg(AFE_REG_RESP2, 0x03); // value: enable internal RLD reference, default value
	
	afe_send_all_regs();
}

unsigned char afe_lead_off_read()
{
	g_afe_leadoff_flag=afe_rreg(AFE_REG_LEADOFF_STAUS);
	return g_afe_leadoff_flag;
}

int32_t afe_read()
{
	afe_spi = afe_spi_master_init();
		//_clear_data();

		//while (nrf_gpio_pin_read(AFE_DRDY) == 1) { };
		
		int result = afe_spi_transfer_data(afe_spi, 6, (const uint8_t *)data, data);
		
		int32_t value = (rx_data[3] << 16) | (rx_data[4] << 8) | afe_data[5];

		return value;
}

int afe_has_data()
{
		return (nrf_gpio_pin_read(AFE_DRDY) == 0);
}


void afe_power_off(void)
{
	nrf_gpio_pin_clear(AFE_PWDN_RST);
}

unsigned int afe_data_rate_count;
unsigned char afe_deal(void)
{
	static int32_t afe_value_sum = 0;
	static int8_t afe_value_count = 0;
	static int afe_idx = 0;
	static uint8_t afe_seq = 0xab;
	static int32_t v1, v2, v3;
	if (afe_set_flag)
		{
			afe_config();
	    afe_start();
	    afe_set_flag=0;
	    return 0;
		}
	if (g_afe_started && afe_has_data())
		{
					 			  	int32_t value = afe_rdata_read();//afe_read();
					 			  	// sign extend
							 			  if (value & (1<<23))
							 			  {
						 			  		  value = value - (1<<24);
						 			  	}
						 			  	//value = (value & 0x7FFFFF) | ((value & 0x80000000) >> 8);
						 			  	/*
g_afe_median=1;
						 			  	if (g_afe_median)
						 			  	{
						 			  		v1 = value;
							 			  	value = MIN( MIN( MAX(v2, v1), MAX(v1, v3) ), MAX(v2, v3) );
							 			  	v3 = v2;
						 			  		v2 = v1;
						 			  	}
						 			  	
					 			  	if (g_afe_decimate)
				 			  		{
							 			  // sign extend
							 			  if (value & (1<<23))
							 			  {
						 			  		  value = value - (1<<24);
						 			  	}

						 			  	if (g_afe_median)
						 			  	{
						 			  		v1 = value;
							 			  	value = MIN( MIN( MAX(v2, v1), MAX(v1, v3) ), MAX(v2, v3) );
							 			  	v3 = v2;
						 			  		v2 = v1;
						 			  	}

						 			  	afe_value_sum += value;
						 			  	afe_value_count ++;
						 			  	if (afe_value_count < g_afe_decimate)
						 			  	{
//						 			  		continue;
						 			  	}
						 			  	value = (afe_value_sum / afe_value_count);
						 			  	// opposite of sign-extend
						 			  	value = (value & 0x7FFFFF) | ((value & 0x80000000) >> 8);
						 			  	afe_value_sum = 0;
						 			  	afe_value_count = 0;
						 			  }
						 			  //judge the bigest different data
                    if (afe_old_data>value)
                    	{
                    		afe_minus_data=afe_old_data-value;
                    	}
                    else
                    	{
                    		afe_minus_data=value-afe_old_data;
                    	}
                    if ((afe_minus_data>afe_biggest_data)&&(afe_old_data!=0))
                    	{
                    		afe_biggest_data=afe_minus_data;
                    	}
                    afe_old_data=value;
                    */
                    //end
					 			  	afe_data[afe_idx] = value;
					         if (g_afe_bits==24)
					         	{
					 			  	afe_idx ++;
					 			  		if (afe_idx >= 6)
					 			  	{
					 			  		
					 			  		afe_seq ++;
					 			  		for (int i = 0; i < afe_idx; i++)
						 			  	{
							 			  	value = afe_data[i];
							 			  	AFE_data_buffer[i*3+0] = 0xff & (value >> 16);
							 			  	AFE_data_buffer[i*3+1] = 0xff & (value >> 8);
							 			  	AFE_data_buffer[i*3+2] = 0xff & (value >> 0);
						 			  	}
						 			  	
						 			  	afe_idx = 0;
						 			  	return 1;
					 			  	}
					 			  }
					 			  else//16
					 			  	{
					 			  		afe_idx ++;
					 			  		if (afe_idx >= 9)
					 			  	{
					 			  		
					 			  		afe_seq ++;
					 			  		for (int i = 0; i < afe_idx; i++)
						 			  	{
							 			  	value = afe_data[i];
							 			  	AFE_data_buffer[i*2+0] = 0xff & (value >> 16);
							 			  	AFE_data_buffer[i*2+1] = 0xff & (value >> 8);
							 			  	//AFE_data_buffer[i*3+2] = 0xff & (value >> 0);
						 			  	}
						 			  	
						 			  	afe_idx = 0;
						 			  	return 1;
					 			  	}
					 			  	}
      
		}
	return 0;		        
}

unsigned int afe_data_rate_count_get(void)
{
	unsigned int temp;
	temp=afe_data_rate_count;
	afe_data_rate_count=00;
	return temp;
}
unsigned char afe_data_turn(void)
{
	unsigned char i;
	afe_idx-=g_afe_dataout_num;
	for (i=0;i<afe_idx;i++)
	{
		afe_data[i]=afe_data[i+g_afe_dataout_num];
	}
	return afe_idx;
}
unsigned char *afe_data_get(void)
{
	int32_t value;
	unsigned char i;
	/*
	if (g_afe_dataout_num==6)
		{
	for (int i = 0; i < 6; i++)
						 			  	{
							 			  	value = afe_data[i];
							 			  	AFE_data_buffer[i*3+0] = 0xff & (value >> 16);
							 			  	AFE_data_buffer[i*3+1] = 0xff & (value >> 8);
							 			  	AFE_data_buffer[i*3+2] = 0xff & (value >> 0);
						 			  	}
		}
	if (g_afe_dataout_num==9)
		{
			for (int i = 0; i < 9; i++)
						 			  	{
							 			  	value = afe_data[i];
							 			  	AFE_data_buffer[i*2+0] = 0xff & (value >> 16);
							 			  	AFE_data_buffer[i*2+1] = 0xff & (value >> 8);
							 			  	//AFE_data_buffer[i*3+2] = 0xff & (value >> 0);
						 			  	}
		}
		
		*/
	return AFE_data_buffer;
}
unsigned char afe_flag_get(void)
{
	return g_afe_started;
}
int32_t afe_bigest_different_data_get(void)
{
	int32_t temp;
	temp=afe_biggest_data;
	afe_biggest_data=0;
	return temp;
}

unsigned char afe_rate_set(unsigned int temp)
{
	switch (temp)
	{
		case 125:g_afe_sample_rate=0;break;
			case 250:g_afe_sample_rate=1;break;
				case 500:g_afe_sample_rate=2;break;
					case 1000:g_afe_sample_rate=3;break;
						case 2000:g_afe_sample_rate=4;break;
							case 4000:g_afe_sample_rate=5;break;
								case 8000:g_afe_sample_rate=6;break;
									default :g_afe_sample_rate=g_afe_sample_rate_default;break;
	}
	//g_afe_sample_rate=temp;
	/*
	if (temp>6)
		{
			g_afe_sample_rate=g_afe_sample_rate_default;
		}*/
	return g_afe_sample_rate;
}

unsigned char afe_rate_get(void)
{
	return g_afe_sample_rate;
}

unsigned char afe_gain_set(unsigned char temp)
{
	switch (temp)
	{
		case 1:g_afe_gain=0;break;
			case 2:g_afe_gain=1;break;
				case 3:g_afe_gain=2;break;
					case 4:g_afe_gain=3;break;
						case 6:g_afe_gain=4;break;
							case 8:g_afe_gain=5;break;
								case 12:g_afe_gain=6;break;
					default :g_afe_gain=0;break;
	}
	//g_afe_gain=temp;
	if (g_afe_gain>6)
		{
			g_afe_gain=g_afe_gain_default;
		}
	return g_afe_gain;
}

unsigned char afe_gain_get(void)
{
	return g_afe_gain;
}

unsigned char afe_decimation_set(unsigned char temp)
{
	g_afe_decimate=temp;
	if (g_afe_decimate>100)
		{
			g_afe_decimate=g_afe_decimate_default;
		}
	return g_afe_decimate;
}

unsigned char afe_decimation_get(void)
{
	return g_afe_decimate;
}

unsigned char afe_databit_set(unsigned char temp)
{
	switch (temp)
	{
		case 16:g_afe_bits=16;g_afe_dataout_num=9;break;
			case 24:g_afe_bits=24;g_afe_dataout_num=6;break;
				default :g_afe_bits=g_afe_bits_default;g_afe_dataout_num=6;break;
	}
	return g_afe_bits;	
}

unsigned char afe_databit_get(void)
{
	return g_afe_bits;
}

unsigned char afe_mux_get(void)
{
	return g_afe_mux;
}

unsigned char afe_mux_set(unsigned char temp)
{
	g_afe_mux=temp;
	if (g_afe_mux>5)
		{
			g_afe_mux=g_afe_mux_default;
		}
	return g_afe_mux;
}


void afe_parament_set(unsigned char *temp_arr)
{
	afe_rate_set(temp_arr[1]*256+temp_arr[2]);
	afe_gain_set(temp_arr[3]);
	//afe_decimation_set(temp_arr[2]);
	afe_databit_set(temp_arr[4]);
	afe_set_flag=1;
	
}