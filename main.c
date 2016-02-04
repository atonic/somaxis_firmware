/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup from button, advertise, get a connection
 * restart advertising on disconnect and if no new connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified with 'YOUR_JOB' indicates where
 * and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stdio.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "app_scheduler.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "ble_nus.h"
#include "simple_uart.h"
#include "boards.h"
#include "nrf_ecb.h"

#include "nrf_delay.h"
#include "spi_master_config.h"
#include "spi_master.h"

#include "uart.h"
#include "battery_motor.h"
#include "system_timer.h"
#include "acc.h"
#include "afe.h"


#define debug_set 1

//#define DEVICE_NAME                     "xhgao1 Cricket Chirp"                                /**< Name of device. Will be included in the advertising data. */
#define device_head_name "Cricket"
#define system_soft_verision    "cricket_v1.0.0001.china.160129"
#define WAKEUP_BUTTON_PIN               15                                          /**< Button used to wake up the application. */

// Some pin (re) defines
#undef ADVERTISING_LED_PIN_NO
#undef CONNECTED_LED_PIN_NO
#undef ASSERT_LED_PIN_NO
// LED pins: 20 = green, 21 = yellow, 23 = red
 #define ADVERTISING_LED_PIN_NO  20
#define CONNECTED_LED_PIN_NO    21
// #define ASSERT_LED_PIN_NO       23

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      60                                         /**< The advertising timeout (in units of seconds). */

// YOUR_JOB: Modify these according to requirements.
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               16                                           /**< Minimum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               40                                           /**< Maximum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                400                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint8_t tx_data[TX_RX_MSG_LENGTH]; /*!< SPI TX buffer */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /*!< SPI RX buffer */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus;

unsigned char DEVICE_NAME[20];

//protocol
#define protocol_vesion 1//0:old,from the USA  1:NEW, NOT ENCRYPTION

void afe_compress_deal(void);




/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
//    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}

/***************************************************
State
****************************************************/
int g_is_connected = 0;






void _clear_data()
{
  for(uint8_t i = 0; i < 16; i++)
  {
    tx_data[i] = 0;
    rx_data[i] = 0;
  }
}





/***************************************************
Battery code
****************************************************/

#define BAT_MON_EN 30
	
uint16_t battery_measure()
{
		nrf_gpio_pin_dir_set(BAT_MON_EN, NRF_GPIO_PIN_DIR_OUTPUT);
		nrf_gpio_pin_set(BAT_MON_EN);
		nrf_delay_ms(10);
		
		// NOTE This is a horrible workaround.  
		// The thing we want to measure is going to AREF0 (pin P0.00), which is not an analog input  
		// Therefore, the only way to measure it is to measure a known voltage (1/3 of regulated supply) using it as references
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                          (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                          (ADC_CONFIG_REFSEL_External << ADC_CONFIG_REFSEL_Pos) |
                          (ADC_CONFIG_PSEL_AnalogInput0 << ADC_CONFIG_PSEL_Pos) |
                          (ADC_CONFIG_EXTREFSEL_AnalogReference0 << ADC_CONFIG_EXTREFSEL_Pos);


    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
    NRF_ADC->TASKS_START = 1;
	
		uint32_t counter = 0;
		while (NRF_ADC->EVENTS_END == 0 && counter < 1000000) { counter ++; }

    uint16_t adc_result = 0;

    NRF_ADC->EVENTS_END = 0;
    adc_result = NRF_ADC->RESULT;
		NRF_ADC->TASKS_STOP = 1;
		
		nrf_gpio_pin_clear(BAT_MON_EN);
		
		return adc_result;
}

void all_power_off()
{
	// AFE off
  afe_power_off();
	
	// Accel off
	//nrf_gpio_pin_clear(ACCEL_PWR);
	acc_power_off();
	
	// LED off
	// very strange: this does not normally turn off when sd_power_system_off is called
	// do all set pins stay on?
	nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
	nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
	
	nrf_gpio_pin_clear(BAT_MON_EN);
	
	// Go to system-off mode (this function will not return; wakeup will cause a reset)
  GPIO_WAKEUP_BUTTON_WITH_PULLUP_CONFIG(WAKEUP_BUTTON_PIN);
  sd_power_system_off();
}


/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Service error handler.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the 
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/*
// YOUR_JOB: Uncomment this function and make it handle error situations sent back to your 
//           application by the services it uses.
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
} */


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
//    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    
    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); */
}


/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile) 
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

void data_handler(ble_nus_t * p_nus, int num, uint8_t * data, uint16_t length)
{
	unsigned char i;
	unsigned char temp[30];
	
  if (num==7)
	{ 
		if ((data[0]==0xb0)&&(data[1]==0x04)&&(data[2]==0x01))//moto
			{
				motor_start(data[3]);
			}
		if ((data[0]==0xb0)&&(data[1]==0x00))//time
			{
				
				system_timer_get(temp);
				if ((temp[0]==0)||(temp[6]==0))//year==0 or second==0
					{
						system_timer_set(&data[2]);
					}
					/*
	//simple_uart_putstring("receive:");
				for (i=0;i<7;i++)
      		{
      			usart_send_chardata_to_hexascii(data[i+2]);
      		}
      		//uart_send_single_data(0x0d);
	//uart_send_single_data(10);
	*/
			}
		if ((data[0]==0xb0)&&(data[1]==0x02))//acc
			{simple_uart_putstring("acc ");
				usart_send_chardata_to_decascii(length);
				simple_uart_putstring(":");
				for (i=0;i<length;i++)
				{
					usart_send_chardata_to_decascii(data[i]);
      			simple_uart_putstring(" ,");
				}
				uart_send_single_data(0x0d);
	      uart_send_single_data(10);
				//acc_parament_set(&data[2]);
				//ble_nus_send_acc_parament(&m_nus);
			}
		if ((data[0]==0xb0)&&(data[1]==0x01))//afe
			{
				simple_uart_putstring("exg ");
				usart_send_chardata_to_decascii(length);
				simple_uart_putstring(":");
				for (i=0;i<length;i++)
				{
					usart_send_chardata_to_decascii(data[i]);
      			simple_uart_putstring(" ,");
				}
				uart_send_single_data(0x0d);
	      uart_send_single_data(10);
	      
				//afe_parament_set(&data[2]);
				//ble_nus_send_afe_parament(&m_nus);
			}
		if ((data[0]==0xb0)&&(data[1]==0x03))//system
			{simple_uart_putstring("system");
				usart_send_chardata_to_decascii(length);
				simple_uart_putstring(":");
				for (i=0;i<length;i++)
				{
					usart_send_chardata_to_decascii(data[i]);
      			simple_uart_putstring(" ,");
				}
				uart_send_single_data(0x0d);
	      uart_send_single_data(10);
	      
	      if (data[3]==82)//reset
	      	{
	      		 afe_power_off();
	
	// Accel off
	//nrf_gpio_pin_clear(ACCEL_PWR);
	acc_power_off();
	      		NVIC_SystemReset();
	      	}
	      if (data[3]==79)//sleep
	      	{
	      		all_power_off();
	      	}
			}
		if ((data[0]==0xb0)&&(data[1]==10))//read info command
			{
				switch (data[2])
				{
					case 0:ble_nus_send_device_info(&m_nus);break;//system_info
						case 1:ble_nus_send_afe_parament(&m_nus);break;//exg info
							case 2:ble_nus_send_acc_parament(&m_nus);break;//acc info
								case 3:ble_nus_send_battery_and_flash_parament(&m_nus);break;
									
				}
			}
	}
}


/**@brief Initialize services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    static ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof nus_init);
    nus_init.data_handler = data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start timers.
*/
static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    uint32_t err_code;
    
    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code = NRF_SUCCESS;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            //nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						g_is_connected = 1;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

						// Reset power to default
						sd_ble_gap_tx_power_set(0);
            advertising_start();

            g_is_connected = 0;
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                all_power_off();
            }
            break;

						
				case BLE_GAP_EVT_CONN_PARAM_UPDATE:
					  break;
				
        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // YOUR_JOB: If the MTU size is changed by the application, the MTU_SIZE parameter to
    //           BLE_STACK_HANDLER_INIT() must be changed accordingly.
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, // NRF_CLOCK_LFCLKSRC_XTAL_500_PPM, //
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}

/**@brief Initialize button handler module.
 */
static void buttons_init(void)
{
    nrf_gpio_cfg_input(WAKEUP_BUTTON_PIN,NRF_GPIO_PIN_PULLUP);//GPIO_WAKEUP_BUTTON_WITH_PULLUP_CONFIG(WAKEUP_BUTTON_PIN);
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


void button_short_press_function_deal(void)
{
	
}

void button_long_press_function_deal(void)
{
	if (! g_is_connected)
		{
			nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
	    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
			while (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN)==0);
			nrf_delay_ms(1000);
			all_power_off();
		}
	else
		{
			nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
	    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
			while (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN)==0);
			nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
			nrf_delay_ms(1000);
			all_power_off();
		}
}

void button_long_press_end_function_deal(void)
{
	
}

/**@brief Application main function.
 */
#define button_short_time_limit 20
#define button_long_time_limit 2000 
 //should be run in every 1ms
void button_deal(void)
{
	static unsigned int button_press_time_count;
	if (nrf_gpio_pin_read(WAKEUP_BUTTON_PIN)==0)
		{
			if (button_press_time_count<button_long_time_limit)
				{
					button_press_time_count++;
					if (button_press_time_count==button_long_time_limit)
						{
							button_long_press_function_deal();
						}
				}
		}
	else
		{
			if ((button_press_time_count>=button_short_time_limit)&&(button_press_time_count<button_long_time_limit))
				{
					button_short_press_function_deal();
				}
			if (button_press_time_count>=button_long_time_limit)
				{
					button_long_press_end_function_deal();
				}
			button_press_time_count=0;
			
		}
}
//should be run in every 100ms
void led_run(void)
{

	static unsigned char connected_led_light_count,connected_led_unlight_count;
	static unsigned char ADVERTISING_LED_light_count,ADVERTISING_LED_unlight_count;
	static unsigned char life_led_count;
	//connect
	if (g_is_connected)
		{
			nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
			
			if ((life_led_count++)>50)
				{
					//nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
					if (life_led_count>=52)
						{
							life_led_count=0;
							//uart_send_single_data('l');
						}
					
				}
			else
				{
					//nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
				}
		}
	else
		{
			if ((connected_led_light_count++)<2)
				{
					nrf_gpio_pin_toggle(CONNECTED_LED_PIN_NO);
					//uart_send_single_data(0x3c);
					connected_led_light_count=0;
				}
			else
				{
					if (connected_led_light_count<4)
						{
						}
				}
		}
}


void display_device_id(void)
{
	unsigned char temp;
	uart_send_single_data(0x0d);
	uart_send_single_data(10);

	simple_uart_putstring("device id:");
	
	temp=(NRF_FICR->DEVICEID[1]>>12)&0x0f;
	//uart_send_single_data(temp);
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[1]>>8)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[1]>>4)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[1])&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
				
			temp=(NRF_FICR->DEVICEID[0]>>12)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[0]>>8)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[0]>>4)&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[0])&0x0f;
			if (temp>9)
				{
					uart_send_single_data(temp-10+'A');
				}
			else
				{
					uart_send_single_data(temp+'0');
				}
			uart_send_single_data(0x0d);
			uart_send_single_data(10);
}


void self_inspection_in_start(void)
{
	unsigned int delay_time_count,delay_time_count2;
	unsigned char temp;
	unsigned char temp_arr[20];
	nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	simple_uart_putstring("device_firmware_verison:");
	simple_uart_putstring(system_soft_verision);
	//device id
	
	display_device_id();
	
	
	
	
	simple_uart_putstring("flash :");
  usart_send_chardata_to_decascii(NRF_FICR->CODESIZE);
  simple_uart_putstring("K");
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
	
	simple_uart_putstring("RAM :");
  usart_send_chardata_to_decascii(NRF_FICR->NUMRAMBLOCK*64);
  simple_uart_putstring("K");
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
  //acc test 
  simple_uart_putstring("ACCELEMETER TEST:");//ACCELEMETER 
  usart_send_chardata_to_hexascii(accel_rreg(0x0f));
  uart_send_single_data(',');
  //acc start 
  //accel_config();
	accel_start();
	
	//afe start
	
	afe_send_all_regs();
	afe_config();
	afe_start();
		
	//acc detect	
	nrf_delay_ms(10);
	
	uart_send_single_data(0x0d);
	uart_send_single_data(10);
  simple_uart_putstring("ok,acc data:");
  if (accel_has_data())
  	{
      accel_gyro_read();
      get_acc_data(temp_arr);
  		usart_send_chardata_to_decascii(temp_arr[0]);
  		uart_send_single_data(',');
			usart_send_chardata_to_decascii(temp_arr[1]);
			uart_send_single_data(',');
			usart_send_chardata_to_decascii(temp_arr[2]);
			uart_send_single_data(',');
			usart_send_chardata_to_decascii(temp_arr[3]);
			uart_send_single_data(',');
  	}
  else
  	{
  		simple_uart_putstring("err:fault");
  	}
  //afe_wreg(0x09,0x02);
  //afe_wreg(0x0a,0x03);
  //EKG test
  uart_send_single_data(0x0d);
  uart_send_single_data(10);
  simple_uart_putstring("AFE TEST:");
  afe_sdatac();
  for (temp=0;temp<12;temp++)
  {
  	usart_send_chardata_to_hexascii(temp);
  	simple_uart_putstring(":0x");
  	usart_send_chardata_to_hexascii(afe_rreg(temp));
  	uart_send_single_data(',');
  }
  
  
  if (afe_has_data())
  	{
  		int32_t value = afe_rdata_read();//afe_read();
  		simple_uart_putstring("ok:");
  		usart_send_chardata_to_decascii((value>>16)&0xff);
  		uart_send_single_data(',');
  		usart_send_chardata_to_decascii((value>>8)&0xff);
  		uart_send_single_data(',');
  	}
  else
  	{
  		simple_uart_putstring("err:fault");
  	}
  	//afe_rdatac();
  uart_send_single_data(0x0d);
  uart_send_single_data(10);
  simple_uart_putstring("flash test:");
  temp=w25q64_user_init();
  if (temp)
  {
  	simple_uart_putstring("fault");
  }
  else
  	{
  		simple_uart_putstring("ok");
  	}
  //w25q64_test();
  //system start
  uart_send_single_data(0x0d);
  uart_send_single_data(10);
  
  temp=battery_voltage_get();
  simple_uart_putstring("battery test:");
  usart_send_chardata_to_decascii(temp);
  uart_send_single_data(0x0d);
  uart_send_single_data(10);
  
  simple_uart_putstring("RAM STAUS:");
//  usart_send_chardata_to_decascii(NRF_POWER->RAMSTATUS);
  uart_send_single_data(0x0d);
  uart_send_single_data(10);
  
  
  simple_uart_putstring("SYSTEM START");
  nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
}

void ecb_test(void)
{
	unsigned char i,temp;
	unsigned char aes_key[16]={"0123456789abcdef"};
		unsigned char aes_source_test[16]={"abcdefghijk12345"};
		unsigned char aes_encrypt_data[16];
		
		simple_uart_putstring("ecb test");
		nrf_ecb_set_key(aes_key);
		simple_uart_putstring("ecb key set");
		nrf_ecb_crypt(aes_encrypt_data,aes_source_test);
		simple_uart_putstring("ecb encrypt ok");
		uart_send_single_data(0x0d);
	uart_send_single_data(10);
		for (i=0;i<16;i++)
		{
			usart_send_chardata_to_hexascii(aes_encrypt_data[i]);
			uart_send_single_data(';');
		}
		uart_send_single_data(0x0d);
	uart_send_single_data(10);
}
int main(void)
{
	unsigned int temp;
	
  unsigned char i;
 unsigned char temp_arr[50];
  unsigned int acc_datarate_count,afe_datarate_count,acc_datarate_count_temp,afe_datarate_count_temp;
  unsigned int acc_datarate_count_all,afe_datarate_count_all,acc_datarate_count_all_temp,afe_datarate_count_all_temp;

	NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }
	leds_init();
    nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
    // Initialize
     
 	 {
  temp=sizeof (device_head_name)-1;
  for (i=0;i<temp;i++)
  {
  	DEVICE_NAME[i]=device_head_name[i];
  }
  
 // NRF_FICR->DEVICEID[1]>>12
  
  temp=(NRF_FICR->DEVICEID[1]>>12)&0x0f;
	//uart_send_single_data(temp);
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
	temp=(NRF_FICR->DEVICEID[1]>>8)&0x0f;
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[1]>>4)&0x0f;
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[1])&0x0f;
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
				
			temp=(NRF_FICR->DEVICEID[0]>>12)&0x0f;
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[0]>>8)&0x0f;
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[0]>>4)&0x0f;
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
				temp=(NRF_FICR->DEVICEID[0])&0x0f;
			if (temp>9)
				{
					DEVICE_NAME[i++]=(temp-10+'A');
				}
			else
				{
					DEVICE_NAME[i++]=(temp+'0');
				}
    DEVICE_NAME[i++]=' ';
		NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;
}

w25q64_init();
system_user_data_init();
		
		//NRF_POWER->DCDCEN=1;
	  accel_init();
	  afe_init();
	  

    uart_init();
		uart_send_single_data('0');
    
    simple_uart_putstring("AES start:");
    temp=nrf_ecb_init();
    if (temp)
			{
				simple_uart_putstring("init ok");
			}
		else
			{
				simple_uart_putstring("init failt");
			}
			
		ecb_test();
		simple_uart_putstring("start");
		ecb_test();
    timers_init();
    buttons_init();
    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
    uart_send_single_data('1');
    ble_stack_init();
    uart_send_single_data('2');
    gap_params_init();
    uart_send_single_data('3');
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();
    uart_send_single_data('4');
    nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
    //XHGAO WRITE INIT
	
  battery_init();
  
	
    timers_start();
    advertising_start();
    uart_send_single_data('A');


		
		
		


		
		
		nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
		
		uart_send_single_data('B');
		
		system_timer_start();
		
		afe_send_all_regs();
		afe_start();
		self_inspection_in_start	();
			
			motor_pwm_init();
			
			//motor_enable();
			//nrf_delay_ms(500);
			//motor_disable();
		motor_start(2);
    //lzw_compress_init();
		//test_lzw_compress();
		//lzw_compress_init();
		/*
		w25q64_init();
		unsigned char flash_test_flag;
		unsigned int flash_test_count=0;
		uint32_t old_data,new_data,delta_data;
		w25q64_read(0x6000,temp_arr,1);
		flash_test_flag=temp_arr[0];
		usart_send_chardata_to_decascii(flash_test_flag);
		*/
		
		/*
		if (flash_test_flag==1)
			{
				
				while (flash_test_count<9000)
				{
				w25q64_read(0x1000+flash_test_count,temp_arr,3);
				
				flash_test_count+=3;
				new_data=(temp_arr[0]<<16)+(temp_arr[1]<<8)+temp_arr[2];
				if (new_data<old_data)
					{
						delta_data=old_data-new_data;
					}
				else
					{
						delta_data=new_data-old_data;
					}
				old_data=new_data;
				usart_send_chardata_to_hexascii(delta_data);
				simple_uart_putstring(" ,");
				
	      }
	      lzw_compress_long_end_deal();
	      uart_send_single_data(0x0d);
	  			  uart_send_single_data(10);
	
    			  simple_uart_putstring("test compress source datacount:");
	  			  usart_send_chardata_to_decascii(lzw_compress_source_count_get());	
    				uart_send_single_data(0x0d);
	    			uart_send_single_data(10);
	    
	   			  simple_uart_putstring("test compressed datacount:");
	    			usart_send_chardata_to_decascii(lzw_compressed_count_get());	
	   			  uart_send_single_data(0x0d);
	    			uart_send_single_data(10);
	    		
		}
		flash_test_count=0;
	  //flash_test_flag=2;
		lzw_compress_init();
		*/
		device_flash_init();
		
    __enable_irq();
    while(1)
    {
    	
    	
    		
    	//continue;
    	//deal with the time and other function
    
    	{
    	//system_timer_deal();//for mcu time base
    	timer1_event_deal();
    	if (get_system_timer_ms_flag())//ms
    		{
    			button_deal();
					
    		}
    	if (get_system_timer_100ms_flag())
    		{
    			led_run();
					
    		}
      if (get_system_timer_second_flag())
      	{
      		acc_datarate_count=acc_datarate_count_temp;
      		afe_datarate_count=afe_datarate_count_temp;
      		afe_datarate_count_all=afe_datarate_count_all_temp;
      		acc_datarate_count_temp=0;
      		afe_datarate_count_temp=0;
      		afe_datarate_count_all_temp=0;
      		battery_deal();
      		if (debug_set)
      			{
      				system_timer_get(temp_arr);
      		
      		uart_send_single_data(0x0d);
	        uart_send_single_data(10);
          //battery_deal();
          static unsigned char arr_test[20];
          unsigned char *p;
          p=acc_data_get();
          
          temp=afe_data_rate_count_get();
          arr_test[0]++;
          arr_test[1]=acc_datarate_count>>8;//afe_rate_get();//acc_datarate_count>>8;
          arr_test[2]=acc_datarate_count;//afe_gain_get();//acc_datarate_count;
          arr_test[3]=afe_datarate_count>>8;//afe_decimation_get();//afe_datarate_count>>8;
          arr_test[4]=afe_datarate_count;//afe_databit_get();//afe_datarate_count;
          arr_test[5]=afe_datarate_count_all>>8;//afe_rate_get();//afe_mux_get();
          arr_test[6]=afe_datarate_count_all;//afe_gain_get();//0xaa;
          arr_test[7]=g_is_connected;//afe_databit_get();
          arr_test[8]=0xaa;
           arr_test[9]=system_flash_data_count_get()>>24;
            arr_test[10]=system_flash_data_count_get()>>16;
          arr_test[11]=system_flash_data_count_get()>>8;
          arr_test[12]=system_flash_data_count_get();
          arr_test[13]=system_flash_percent_get();
          arr_test[14]=get_battery_quality_percent();
          arr_test[15]=gyro_rate_get();
           
          arr_test[16]=gyro_full_scale_get();
          
          for (i=0;i<15;i++)
      		{
      			usart_send_chardata_to_decascii(arr_test[i]);
      			simple_uart_putstring(" ,");
      		}
          if (g_is_connected)
          	{
          		ble_nus_send_debug_data(&m_nus, arr_test, 15); // DEBUG
          		
          	}
          	
          	//w25q64_read(0x00,arr_test,10);
          }
      		
      	}
      
      {
      	temp=afe_deal();
      	if (temp)
      		{
      			
      		unsigned char test_afe_data[20];
      		
      			if (g_is_connected)
      				{
      					static unsigned char test_afe_temp;
      					unsigned char test_afe_data2[20];
      					unsigned char *p;
      					
      					p=afe_data_get();
      					for (i=0;i<18;i++)
      					{
      						test_afe_data[i+1]=p[i];
      					}
      					test_afe_data[0]=test_afe_temp;
      					int result = ble_nus_send_afe_data_verison1(&m_nus, &test_afe_data[1], 18 );
      					afe_datarate_count_all_temp++;
      					if (result==0)
      						{
      					    afe_datarate_count_temp++;
      					    test_afe_temp++;
      						}
      					else
      						{
      							//acc_datarate_count_temp++;
      						}
      					
      				
      				}
      			
      			
      			//the follow is the exg to memory test 
      			/*
      			{
      				unsigned char *p;
      				static unsigned char save_datalong,save_data[60];
      					p=afe_data_get();
      					for (i=0;i<18;i++)
      					{
      						save_data[i+save_datalong]=p[i];
      					}
      					save_datalong+=18;
      					if (save_datalong>=54)
      						{
      							afe_datarate_count_temp++;
      							//simple_uart_putstring("save start");
      							//w25q64_write(0x2000,save_data,save_datalong);
      							temp=device_flash_dave(save_data,save_datalong);
      							if (temp==1)
      								{
      									simple_uart_putstring("end");
      								}
      							//simple_uart_putstring("end");
      							save_datalong=0;
      						}
      			}
      			*/
      		}
      }
      	
      {
      	temp=acc_deal();
      	//temp=0;
      	if (temp)
      		{
      			//simple_uart_putstring("acc data come");
      			if (g_is_connected)
      				{
      					static unsigned char test_acc_temp;
      					unsigned char test_acc_data[20];
      					//test_acc_data[0]=test_
      					test_acc_data[0]=test_acc_temp;
      					int result = ble_nus_send_accel_data_verison1(&m_nus, acc_data_get(),12);//get_acc_data(), 12 );//test_acc_data,12);//
      					if (result==0)
      						{
      							acc_datarate_count_temp++;
      							acc_data_turn();
      							test_acc_temp++;
      						}
      				}
      			
      		}
      }
      
      
	    }
			

    }
}

//#define afe_compressed_max_data 4070
void afe_compress_deal(void)
{
	unsigned char temp,i;
	unsigned char test_arr[20];
	unsigned char *p;
	p=afe_data_get();
	/*
	for (i=0;i<6;i++)
				{
					test_arr[i*2]=p[i*3];
					test_arr[i*2+1]=p[i*3+1];
				}*/
	temp=lzw_compress_long(p,18);
	if (temp==1)
		{
			//lzw_compress_long_end_deal();
			uart_send_single_data(0x0d);
	    uart_send_single_data(10);
	
      simple_uart_putstring("compress source datacount:");
	    usart_send_chardata_to_decascii(lzw_compress_source_count_get());	
    	uart_send_single_data(0x0d);
	    uart_send_single_data(10);
	    
	    simple_uart_putstring("compressed datacount:");
	    usart_send_chardata_to_decascii(lzw_compressed_count_get());	
	    uart_send_single_data(0x0d);
	    uart_send_single_data(10);
		}
	if (temp==2)
		{
			uart_send_single_data(0x0d);
	    uart_send_single_data(10);
	
      simple_uart_putstring("send out");
		}
}
/** 
 * @}
 */
