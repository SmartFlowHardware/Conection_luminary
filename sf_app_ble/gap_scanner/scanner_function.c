///************************************************************************************************************************************
// * File: scanner_functions.c
// * ----------------------------------------------------------------------------------------------------------------------------------
// * Description:
// *	This file comprises essential functions for initiating the scanner. The scanner is designed to detect and analyze all devices
// *	advertised via Bluetooth, only the devices connectable. The observer function is equipped with a filtering mechanism to specifically
// *	target devices based on criteria like their name or other attributes.
// *	The primary purpose of this file is to enable the detection of nearby Bluetooth devices and provide a means to selectively filter
// *	and identify desired devices based on their characteristics. This can be particularly useful in scenarios where specific devices
// *	need to be tracked, monitored, or interacted with using Bluetooth technology.
// *	The scanner function serves as an important component in the broader Bluetooth communication system, enhancing the capability
// *	to interact with a diverse range of devices in various application contexts. By effectively identifying and filtering devices,
// *	this functionality contributes to a more efficient and focused Bluetooth communication process.
// *
// *
// * Author:
// *   Your Name
// *
// * Date:
// *   4 ago 2023
// *
// * Version:
// *   Version number of the code
// ***********************************************************************************************************************************/
//
//#include "wiced_bt_ble.h"
//#include "wiced_bt_trace.h"
//#include "wiced_platform.h"
//#include "wiced_hal_wdog.h"
//#include "wiced_hal_nvram.h"
//#include "wiced_memory.h"
//#include "wiced_bt_cfg.h"
//#include "wiced_hal_gpio.h"
//#include "wiced_bt_uuid.h"
//
//#ifdef HCI_CONTROL
//#include "wiced_transport.h"
//#include "hci_control_api.h"
//#endif
//
//#include "biometric_functions.h"
//#include "scanner_functions.h"
//#include "config_ports.h"
//
//
///************************************************************************************************************************************
// * Function Name: start_observe(void)
// * ----------------------------------------------------------------------------------------------------------------------------------
// * Summary:
// * 	The "start_scanner" function is designed to initiate both the observer and the scanner for Bluetooth Low Energy (BLE) devices.
// * 	The function uses the WICED Bluetooth stack API to accomplish this task.
// ***********************************************************************************************************************************/
//void start_scanner(void)
//{
//	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
//
//	wiced_result_t		status;
//
//	if( found_sensor != MAX_NUM_OF_SENSORS )
//	{
//		/* Start the scanner to identify the devices that can be connected */
//		status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY,	// Type of scan: It scan more frequently and actively, making easier to find devices, but consumes more energy
//						   	   	    WICED_TRUE,						// Enabled filter: The scan will ignore duplicate advertisement packets sent by the same device within a short time
//									scanCallback );					// Pointer to call the function
//		WICED_BT_TRACE( "wiced_bt_ble_scan: %d\n", status );
//	}
//	else
//	{
//		status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, scanCallback );
//		WICED_BT_TRACE( "wiced_bt_ble_scan: %d\n", status );
//	}
//}
//
//
///************************************************************************************************************************************
// * Function Name: scanCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
// * ----------------------------------------------------------------------------------------------------------------------------------
// * Summary:
// * 	The scanCallback function is a callback function that is executed when a Bluetooth Low Energy (BLE) scan operation finds an
// * 	advertising device. Its purpose is to search for a specific Advertisement data type in the advertisement data received from the
// * 	device. If the desired UUID is found, it initiates a connection with the device.
// *
// * Parameters:
// * 	wiced_bt_gatt_evt_t *p_scan_result				: Pointer to structure with information about the scanned device.
// * 	uint8_t *p_adv_data								: Pointer to array with advertisement packets.
// ***********************************************************************************************************************************/
//void scanCallback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
//{
//	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
//
//	wiced_result_t         		status;				// Variable to identify the status of Bluetooth operation
//    wiced_bool_t           		ret_status;			// Variable to indicate if the scan found the device to generating the connection
//    uint16_t                	service_uuid16 = 0;	// Variable assistant to convert a uint16_t value
//
//
//    if ( p_scan_result )
//    {
//    	uint8_t		*p_data_uuid;	// Pointer to data for searched information in the advertisement package
//    	uint8_t		length_scan;	// Variable to store the length of the parameter being searched for the function in the advertisement
//
//    	/* Search for SERVICE_UUID_16 element in the Advertisement data received.Check for complete list Advertisement data from Devices */
//    	p_data_uuid = wiced_bt_ble_check_advertising_data( p_adv_data,  BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE, &length_scan );
//
//    	// Find data
//    	if ( p_data_uuid == NULL )
//    	{
//    		/* Search for SERVICE_UUID_16 element in the Advertisement data received.Check for partial list*/
//    		p_data_uuid = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL, &length_scan );
//
//            if ( p_data_uuid == NULL )
//            {
//                return;     // No UUID_16 element
//            }
//        }
//
//
//    // Ask for the data in the NVRAM
//    numbytes16 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START + 16, sizeof( data_nvram ), &data_nvram[0], &status16 );
//    //WICED_BT_TRACE( "Value read from NVRAM: %d\tBytes Read: %d\t\tStatus: 0x%02x\n\r", data_nvram[21], numbytes16, status16 );
//
//    // Check if the data has not been saved in the NVRAM yet
//    if( data_nvram[21] == WICED_FALSE )
//    {
//    	// Start a while loop to process the byte stream as long as there is enough data remaining for a 16-bit UUID.
//    	while ( length_scan >= LEN_UUID_16 )
//    	{
//    		// Read a 16-bit unsigned integer (UUID) from the byte stream pointed to by 'p_data_uuid' and store it in 'service_uuid16'.
//    		STREAM_TO_UINT16( service_uuid16, p_data_uuid );
//
//    		// Filter to UUID matches the device's UUID, ensure not to connect the same sensor, and validate that the RSSI is within the specified limits
//    		if ( ( service_uuid16 == UUID_SERVICE_HEALTH_THERMOMETER ) && ( found_sensor != HEALTH_THERMOMETER_SENSOR ) && ( p_scan_result->rssi > RSSI_THRESHOLD ) )
//    		{
//    			// UUID16 Service found
//    			WICED_BT_TRACE( "Health Thermometer UUID: %02X\r\n", service_uuid16 );
//    			sensor_type = HEALTH_THERMOMETER_SENSOR;
//    			found_sensor |= (1<<0);
//    			device_scanned = WICED_TRUE;
//    			WICED_BT_TRACE("Health Thermometer | sensor_type:%d | found_sensor:%d | device_scanned:%d\r\n", sensor_type, found_sensor, device_scanned);
//    			break;
//    		}
//    		// Filter UUID matches the device's UUID, this going to connect after the Health Thermometer Sensor
//    		else if ( ( service_uuid16 == UUID_SERVICE_HEART_RATE ) && ( found_sensor != 0 && found_sensor != HEART_RATE_SENSOR ) && ( p_scan_result->rssi > RSSI_THRESHOLD ) )
//    		{
//    			// UUID16 Service found
//    			WICED_BT_TRACE( "Heart Rate UUID: %02X\r\n", service_uuid16 );
//    			sensor_type = HEART_RATE_SENSOR;
//    			found_sensor |= (1<<1);
//    			device_scanned = WICED_TRUE;
//    			WICED_BT_TRACE("Heart Rate | sensor_type:%d | found_sensor:%d | device_scanned:%d\r\n", sensor_type, found_sensor, device_scanned);
//    			break;
//    		}
//    		// Decrement the remaining length of the byte stream by the size of a 16-bit UUID (LEN_UUID_16)
//    		length_scan -= LEN_UUID_16;
//    	}
//
//    	// Check if the UUID read is different to any UUID Service.
//    	if ( device_scanned == WICED_FALSE )
//    	{
//    		// UUID16 Service not found. Ignore device
//    		return;
//    	}
//    }
//    // If the data is saved in the NVRAM
//    else
//    {
//    	wiced_bool_t	bd_addr_found;
//    	bd_addr_found = scanning_mac(p_scan_result);
//
//    	// Check if the UUID read is different to the constant value 'UUID_SERVICE' or the Health Thermometer is the 'First'.
//    	if ( ( bd_addr_found == WICED_FALSE ) || ( device_scanned == WICED_FALSE ) )
//    	{
//    		// Bluetoothe Device Address not found. Ignore device
//    		return;
//    	}
//    }
//
//    /* Device Found by the Scanner */
//    WICED_BT_TRACE( "Found Device | Bluetooth Device Address : %B \n", p_scan_result->remote_bd_addr );
//
//    /* Stop the scan since the desired device is found */
//    status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, scanCallback );
//    WICED_BT_TRACE( "scan off status %d\n", status );
//
//    /* Initiate the connection */
//    ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE );
//    WICED_BT_TRACE( "wiced_bt_gatt_connect status %d\n", ret_status );
//
//    }
//    else
//    {
//        WICED_BT_TRACE( "Scan completed\n" );
//    }
//    // Avoid the warning "unused variable"
//    UNUSED_VARIABLE(ret_status);
//    UNUSED_VARIABLE(status);
//}
//
//
///************************************************************************************************************************************
// * Function Name: wiced_bool_t scanning_mac(wiced_bt_ble_scan_results_t *p_scan_result)
// * ----------------------------------------------------------------------------------------------------------------------------------
// * Summary:
// * 	The function will to search the device with the Bluetooth Device Address that was saved in the NVRAM, only will activate after a
// * 	first scan ( for save the Device Bluetooth Address ) and going to connect only with 2 devices (Health Thermometer Sensor and Heart
// * 	Rate Sensor).
// *
// * Parameters:
// * 	wiced_bt_gatt_evt_t *p_scan_result				: Pointer to structure with information about the scanned device.
// * 	uint8_t *p_adv_data								: Pointer to array with advertisement packets.
// *
// * Return:
// *  wiced_bool_t									: Return a True if the device was found or a False if the device wasn't found.
// ***********************************************************************************************************************************/
//wiced_bool_t scanning_mac(wiced_bt_ble_scan_results_t *p_scan_result)
//{
//	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
//
//	// Declare variables to save the values in the NVRAM
//	uint8_t		bd_addr_health_thermometer[BD_ADDR_LEN];
//	uint8_t		bd_addr_heart_rate[BD_ADDR_LEN];
//
//	// Pass the data in NVRAM to the variables
//	memcpy( bd_addr_health_thermometer, &data_nvram[1], sizeof(bd_addr_health_thermometer) );
//	memcpy( bd_addr_heart_rate, &data_nvram[8], sizeof(bd_addr_health_thermometer) );
//
//	// Filter for the BD ADDR and to connect only one type sensor
//	if( ( memcmp(bd_addr_health_thermometer, p_scan_result->remote_bd_addr, BD_ADDR_LEN ) == 0 ) && ( found_sensor != HEALTH_THERMOMETER_SENSOR ) && (p_scan_result->rssi > -78) )
//	{
//		// Bluetooth Device Address found, define variables to Health Thermometer Device
//		sensor_type = HEALTH_THERMOMETER_SENSOR;
//		found_sensor |= (1<<0);
//		device_scanned = WICED_TRUE;
//		WICED_BT_TRACE("Health Thermometer | sensor_type:%d | found_sensor:%d | device_scanned:%d\r\n", sensor_type, found_sensor, device_scanned);
//		return WICED_TRUE;
//	}
//	else if( ( memcmp(bd_addr_heart_rate, p_scan_result->remote_bd_addr, BD_ADDR_LEN ) == 0 ) && ( found_sensor != 0 && found_sensor != HEART_RATE_SENSOR ) && ((p_scan_result->rssi > -78)) )
//	{
//		// Bluetooth device Address found, define variables to Heart Rate Device
//		sensor_type = HEART_RATE_SENSOR;
//		found_sensor |= (1<<1);
//		device_scanned = WICED_TRUE;
//		WICED_BT_TRACE("Heart Rate | sensor_type:%d | found_sensor:%d | device_scanned:%d\r\n", sensor_type, found_sensor, device_scanned);
//		return WICED_TRUE;
//	}
//
//	return WICED_FALSE;
//}
