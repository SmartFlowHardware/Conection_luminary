/************************************************************************************************************************************
 * File: observe_function.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *	This file comprises essential functions for initiating the observer. The observer is designed to detect and analyze all devices
 *	advertised via Bluetooth. The observer function is equipped with a filtering mechanism to specifically target devices based on
 *	criteria like their name or other attributes.
 *	The primary purpose of this file is to enable the detection of nearby Bluetooth devices and provide a means to selectively filter
 *	and identify desired devices based on their characteristics. This can be particularly useful in scenarios where specific devices
 *	need to be tracked, monitored, or interacted with using Bluetooth technology.
 *	The observer function serves as an important component in the broader Bluetooth communication system, enhancing the capability
 *	to interact with a diverse range of devices in various application contexts. By effectively identifying and filtering devices,
 *	this functionality contributes to a more efficient and focused Bluetooth communication process.
 *
 *
 * Author:
 *   Your Name
 *
 * Date:
 *   4 ago 2023
 *
 * Version:
 *   Version number of the code
 ***********************************************************************************************************************************/

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_platform.h"
#include "wiced_rtos.h"
#include "wiced_hal_wdog.h"
#include "wiced_hal_rand.h"
#include "wiced_hal_nvram.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_memory.h"
#include "wiced_bt_cfg.h"

#include "observe_function.h"
#include "config_ports.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
//#include "biometric_functions.h"


/************************************************************************************************************************************
 * Function Name: start_observe(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The "start_observe" function is designed to initiate both the observer and the scanner for Bluetooth Low Energy (BLE) devices.
 * 	The function uses the WICED Bluetooth stack API to accomplish this task.
 ***********************************************************************************************************************************/
void start_observe(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	/* Start the observer to identify the devices that can be scanned and connected */
	wiced_bt_ble_observe( WICED_TRUE,					// Variable to indicate if the Observer stars or stops
						  0,							// Duration of Observer in active mode (0 to infinite)
						  observer_mesh_adv_report );	// Pointer to call the function
}



/************************************************************************************************************************************
 * Function Name: observer_mesh_adv_report( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function is a callback function that is executed when a Bluetooth Low Energy (BLE) scan operation finds an advertising device.
 * 	Its purpose is to search for a specific Advertisement data type in the advertisement data received from the device.
 *
 * Parameters:
 * 	wiced_bt_gatt_evt_t *p_scan_result				: Pointer to structure with information about the scanned device.
 * 	uint8_t *p_adv_data								: Pointer to array with advertisement packets.
 ***********************************************************************************************************************************/
void observer_mesh_adv_report( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
	wiced_result_t	status;				// Variable to identify the status of Bluetooth operation
	wiced_bool_t	ret_status;			// Variable to indicate if the scan found the device to generating the connection
	uint8_t			*p_data_name;		// Pointer to data for searched information in the advertisement package
	uint8_t			*p_device_class;	// Pointer to data fot searched information in the advertisement package
	uint8_t			length_scan;		// Variable to store the length of the parameter being searched for the function in the advertisement
	uint8_t			length_scan_node;	// Variable to store the length of the parameter being searched for the function in the advertisement

    if (p_scan_result == NULL )
        return;


    //WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    /* --- Filters --- */
    if( p_scan_result && p_scan_result->rssi > -48 )
    {
    	/* Search for Name element in the Advertisement data received. Check for complete list Advertisement data from Devices */
    	p_data_name = wiced_bt_ble_check_advertising_data( p_adv_data,  BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &length_scan );
    	p_device_class = wiced_bt_ble_check_advertising_data( p_adv_data,  BTM_BLE_ADVERT_TYPE_DEV_CLASS, &length_scan_node );

    	// Find data
    	if ( p_data_name == NULL )
    	{
    		/* Search for Name element in the Advertisement data received. Check for partial list*/
    		p_data_name = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_SHORT, &length_scan );
        }

//    	WICED_BT_TRACE("RSSI: %d\t", p_scan_result->rssi);
//    	WICED_BT_TRACE_ARRAY(p_adv_data, 19, "Advertisement Package: ");

    	// The Lamp Device is found
    	if( memcmp(filter_lamp, p_data_name, sizeof(filter_lamp)) == 0 )
    	{
    		//WICED_BT_TRACE("FIND LAMP RSSI:%d\r\n", p_scan_result->rssi);
    		find_lamp = WICED_TRUE;
    		wiced_hal_gpio_configure_pin(LED_PERSON, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
    		start_lamp_timer();
    	}

    	// The Tag Device is found
    	if( memcmp(filter_tag, p_data_name, sizeof(filter_tag)) == 0 )
    	{
    		//WICED_BT_TRACE("FIND TAG RSSI:%d\r\n", p_scan_result->rssi);
    		find_tag = WICED_TRUE;
    		wiced_hal_gpio_configure_pin(LED_VEHICLE, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
    		start_tag_timer();
    	}

    	// The Node Device is Found
    	if( memcmp(filter_node, p_device_class, sizeof(filter_node)) == 0 )
    	{
    		WICED_BT_TRACE("FIND NODE RSSI:%d\r\n", p_scan_result->rssi);
    		find_node = WICED_TRUE;
    		wiced_hal_gpio_configure_pin(LED_WARNING, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
    		start_node_timer();
    	}
    }
    else
    {
    	return;
    }
}



void init_event_ADC(void)
{
	if(!value_Adc)
	{
	  wiced_start_timer( &timer_radc, 10);
      //stop_timer_st_online();
      //wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);
		//value_Adc= WICED_FALSE;
       WICED_BT_TRACE("ADC REPORT \n");
	}
}


void f_timer_radc( uint32_t data )
{
	WICED_BT_TRACE("ADC TIMER \n");
	//wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_HIGH);
	value_Adc= WICED_TRUE;
	//wiced_hal_wdog_reset_system ();
}


void init_event_RAC(void)
{
  if(!value_ac)
	{
	gap_transfer();
	}
}


void init_event_gap(void)
{
  if(!value_gap)
	{
	  WICED_BT_TRACE("Init event gap \n");
	  beacon_set_app_advertisement_data3();
	  prevention_status();
	  value_da= WICED_TRUE;

	}
}





