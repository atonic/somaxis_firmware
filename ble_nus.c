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

#include "ble_nus.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


unsigned char senddata[20];
/**@brief Connect event handler.
 *
 * @param[in]   p_nus       Nordic UART Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_nus_t * p_nus, ble_evt_t * p_ble_evt)
{
    p_nus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Disconnect event handler.
 *
 * @param[in]   p_nus       Nordic UART Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_nus_t * p_nus, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_nus->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Write event handler.
 *
 * @param[in]   p_nus       Nordic UART Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_nus_t * p_nus, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if (
        (p_evt_write->handle == p_nus->afe_data_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
    )
    {
        uint16_t value = *(uint16_t *) p_evt_write->data;
        if (value == 0x001)
        {
            p_nus->is_notification_enabled = true;
        }
        else
        {
            p_nus->is_notification_enabled = false;
        }
        
    }
    else if (p_nus->data_handler != NULL)
    {
			  if (p_evt_write->handle == p_nus->afe_command_handles.value_handle)
				{
					p_nus->data_handler(p_nus, 1, p_evt_write->data, p_evt_write->len);
				}
			  if (p_evt_write->handle == p_nus->accel_command_handles.value_handle)
				{
					p_nus->data_handler(p_nus, 2, p_evt_write->data, p_evt_write->len);
				}
				if (p_evt_write->handle == p_nus->verison_data_handles.value_handle)
				{
					p_nus->data_handler(p_nus, 7, p_evt_write->data, p_evt_write->len);
				}

    }
}


void ble_nus_on_ble_evt(ble_nus_t * p_nus, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_nus, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_nus, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_nus, p_ble_evt);
            break;
            
        default:
            break;
    }
}


/**@brief Add RX characteristic.
 *
 * @param[in]   p_nus        Nordic UART Service structure.
 * @param[in]   p_nus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t data_char_add(ble_nus_t * p_nus, const ble_nus_init_t * p_nus_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    // Add Battery Level characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 20;
    
		int result;
		
    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_AFE_DATA_CHARACTERISTIC;
    
    result = sd_ble_gatts_characteristic_add(p_nus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_nus->afe_data_handles);
                                               
		ble_uuid.uuid = BLE_UUID_NUS_ACCEL_DATA_CHARACTERISTIC;
		
    result = sd_ble_gatts_characteristic_add(p_nus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_nus->accel_data_handles);
																							 
		ble_uuid.uuid = BLE_UUID_NUS_DEBUG_DATA_CHARACTERISTIC;
		
    result = sd_ble_gatts_characteristic_add(p_nus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_nus->debug_data_handles);
                                               
    ble_uuid.uuid = BLE_UUID_NUS_DATA_tx;
    
    result = sd_ble_gatts_characteristic_add(p_nus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_nus->tx_data_handles);
		
		return result;

}

/**@brief Add TX characteristic.
 *
 * @param[in]   p_nus        Nordic UART Service structure.
 * @param[in]   p_nus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t command_char_add(ble_nus_t * p_nus, const ble_nus_init_t * p_nus_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 20;
    
		
		int result;
		
    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_AFE_COMMAND_CHARACTERISTIC;
    
    result = sd_ble_gatts_characteristic_add(p_nus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_nus->afe_command_handles);
																							 
		ble_uuid.uuid = BLE_UUID_NUS_ACCEL_COMMAND_CHARACTERISTIC;
    result = sd_ble_gatts_characteristic_add(p_nus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_nus->accel_command_handles);
                                               
    ble_uuid.uuid = BLE_UUID_NUS_DATA_CHARACTERISTIC;
    result = sd_ble_gatts_characteristic_add(p_nus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_nus->verison_data_handles);

		return result;
}

uint32_t ble_nus_init(ble_nus_t * p_nus, const ble_nus_init_t * p_nus_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t nus_base_uuid = {0x53, 0x58, 0x4D, 0x53, 0x5D, 0x83, 0x41, 0xBD, 0xDA, 0x45, 0x26, 0x21, 0x01, 0x00, 0x40, 0xEF};


    // Initialize service structure
    p_nus->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_nus->data_handler = p_nus_init->data_handler;
    p_nus->is_notification_enabled = false;
    

    // Add custom base UUID
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_nus->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add service
    ble_uuid.type = p_nus->uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_nus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
	err_code = data_char_add(p_nus, p_nus_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	err_code = command_char_add(p_nus, p_nus_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
    
	return NRF_SUCCESS;
}


uint32_t ble_nus_send_afe_data(ble_nus_t * p_nus, uint8_t * data, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->afe_data_handles.value_handle;
    hvx_params.p_data = data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

uint32_t ble_nus_send_afe_data_verison1(ble_nus_t * p_nus, uint8_t * data, uint16_t length)
{
	unsigned int i;
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    
    senddata[0]=0xa0;
    senddata[1]=0x01;
    for (i=0;i<length;i++)
    {
    	senddata[2+i]=data[i];
    }
		length+=2;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->tx_data_handles.value_handle;
    hvx_params.p_data = senddata;
    hvx_params.p_len  = &length;//&(length+2);
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

uint32_t ble_nus_send_afe_parament(ble_nus_t * p_nus)
{
	unsigned int i;
	uint16_t length=0;
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    i=0;
    senddata[i++]=0xa0;
    senddata[i++]=0x02;
    senddata[i++]=afe_rate_get();
    senddata[i++]=afe_gain_get();
    senddata[i++]=afe_decimation_get();
    senddata[i++]=afe_databit_get();
    senddata[i++]=afe_mux_get();
    
		length=i;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->tx_data_handles.value_handle;
    hvx_params.p_data = senddata;
    hvx_params.p_len  = &length;//&(length+2);
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

uint32_t ble_nus_send_acc_parament(ble_nus_t * p_nus)
{
	unsigned int i;
	uint16_t length;
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
   
    
    i=0;
    senddata[i++]=0xa0;
    senddata[i++]=0x04;
    senddata[i++]=acc_rate_get();
    senddata[i++]=acc_full_scale_get();
    senddata[i++]=acc_bandwith_filter_get();
    senddata[i++]=0;
    senddata[i++]=0;    
    senddata[i++]=gyro_rate_get();
    senddata[i++]=gyro_full_scale_get();
    senddata[i++]=0;
    senddata[i++]=0;
    senddata[i++]=0;
		length=i;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->tx_data_handles.value_handle;
    hvx_params.p_data = senddata;
    hvx_params.p_len  = &length;//&(length+2);
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}
uint32_t ble_nus_send_battery_and_flash_parament(ble_nus_t * p_nus)
{
	unsigned int i;
	uint16_t length=0;
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    i=0;
    senddata[i++]=0xa0;
    senddata[i++]=0x05;
    senddata[i++]=get_battery_voltage();
    senddata[i++]=get_battery_flag();
    senddata[i++]=get_battery_quality_percent();
    senddata[i++]=0;
    senddata[i++]=0;
    senddata[i++]=0;
    
    senddata[i++]=flash_type_get();
    senddata[i++]=flash_remain_percent();
    senddata[i++]=0;
    senddata[i++]=0;
    senddata[i++]=0;
		length=i;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->tx_data_handles.value_handle;
    hvx_params.p_data = senddata;
    hvx_params.p_len  = &length;//&(length+2);
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

uint32_t ble_nus_send_device_info(ble_nus_t * p_nus)
{
	unsigned int i;
	uint16_t length=0;
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    i=0;
    senddata[i++]=0xa0;
    senddata[i++]=0x00;
    senddata[i++]=get_battery_voltage();
    senddata[i++]=get_battery_flag();
    senddata[i++]=get_battery_q();
    senddata[i++]=0;
    senddata[i++]=0;
    senddata[i++]=0;
    
    senddata[i++]=flash_type_get();
    senddata[i++]=flash_remain_percent();
    senddata[i++]=0;
    senddata[i++]=0;
    senddata[i++]=0;
		length=i;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->tx_data_handles.value_handle;
    hvx_params.p_data = senddata;
    hvx_params.p_len  = &length;//&(length+2);
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

uint32_t ble_nus_send_accel_data(ble_nus_t * p_nus, uint8_t * data, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->accel_data_handles.value_handle;//tx_data_handles//accel_data_handles
    hvx_params.p_data = data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

uint32_t ble_nus_send_accel_data_verison1(ble_nus_t * p_nus, uint8_t * data, uint16_t length)
{
	unsigned char i;
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    senddata[0]=0xa0;
    senddata[1]=0x03;
    for (i=0;i<length;i++)
    {
    	senddata[2+i]=data[i];
    }
    length+=2;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->tx_data_handles.value_handle;
    hvx_params.p_data = senddata;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}


uint32_t ble_nus_send_debug_data(ble_nus_t * p_nus, uint8_t * data, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;
    
    if (!p_nus->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (length > NUS_MAX_DATA_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_nus->debug_data_handles.value_handle;
    hvx_params.p_data = data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}


