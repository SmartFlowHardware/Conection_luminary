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

#include "mesh_definitions.h"
#include "observe_function.h"
#include "config_ports.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"


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


extern wiced_bt_device_address_t bda;
//wiced_bool_t conection_complet;
uint8_t conection_complet=0, conection_complet_node=0, flag=0;
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
	uint8_t			*p_mesh_beacon;
	uint8_t			*p_uid_beacon;
	uint8_t			*p_uid_edystone=NULL;
	uint8_t 		*p_uid_node;        //Response of the UID of the node to conect
	uint8_t			length_scan;		// Variable to store the length of the parameter being searched for the function in the advertisement
	uint8_t			length_scan_node;	// Variable to store the length of the parameter being searched for the function in the advertisement
	uint8_t			length_scan_beacon;
	uint8_t			length_uid_node;
	uint8_t			len, lengthmac=0;
	uint16_t                	service_uuid16 = 0;	// Variable assistant to convert a uint16_t value

	uint8_t *p_name=NULL;    //
	p_name= &p_adv_data[5];  //

    if (p_scan_result == NULL )
        return;


    //     WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
    /* --- Filters --- */
    if( p_scan_result && p_scan_result->rssi > -90)
    {
    	/* Search for Information in the Advertisement data received. */
    	p_data_name = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &length_scan );
    	p_device_class = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_DEV_CLASS, &length_scan_node );
    	p_mesh_beacon = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_MESH_BEACON, &length_scan_beacon );
    	p_uid_node = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, &length_uid_node );

    	p_uid_beacon = &p_adv_data[0];

    	// Find data
    	if ( p_data_name == NULL )
    	{
    		/* Search for Name element in the Advertisement data received. Check for partial list*/
    		p_data_name = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_SHORT, &length_scan );

    		// If doesn't exist information in the advertisement data, return
//    		if( (p_data_name==NULL) && (p_device_class==NULL) && (p_mesh_beacon==NULL) )
//    		{
//    			return;
//    		}
        }

//    	WICED_BT_TRACE("RSSI: %d\t", p_scan_result->rssi);
//    	WICED_BT_TRACE_ARRAY(p_adv_data, 21, "Advertisement Package: ");

    	/** ------------------------- Filters to detect the devices ------------------------- */
    	if( is_provisioned )
    	{
        	// ----- The Node Device is Found, add the proccess to found NODEL BSL -----
//        	if(memcmp(NODEL_BSL1,&p_name[0],5)==0)
//        	{
    		flag=0;
        	if(conection_complet == 0)
        	{
        		/* 1.- Processes to disvocer a luminary whit close RSSI */
        		Conect_process1(p_scan_result);  /* *************** */
        		conection_complet =+ 1;

        		find_node = WICED_TRUE;
        		start_node_timer();

        		// Start a timer to blinking the LED, one Time
        		if(!blinking_led_timer)
        		{
        			start_bled_timer();
        			blinking_led_timer = WICED_TRUE;
        		}
        		//p_data_name = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, &length_scan );
        	}
        	/* Id found the response of a device UID: CN20000000, first confirm the information save from the node */
        	if(conection_complet==1 && memcmp(CN1, &p_uid_node[4],2)==0 && addr11== p_uid_node[6])
        	{
        		WICED_BT_TRACE("\n Red responce of the nodel conection, coection complet %d \n", conection_complet);
        	  WICED_BT_TRACE("\n %s from %B \n",&p_uid_node[4], p_scan_result->remote_bd_addr);
              fill_data_base(p_scan_result, p_uid_node);
              //conection_complet = 0; put in the end of the timer
        	}
    	}
    	/** ------------------------- Filters to connect at the network *Procces of the no central to conect with the central------------------------- */
    	else
    	{
    		if(flag == 0)
    		{
    			//Conect_process1(p_scan_result,1);
    			WICED_BT_TRACE("\n Paro el UID \n");
    			stop_uid();
    			flag=1;
    			conection_complet = 0;
    		}
    		if(conection_complet_node == 0)
    		{
        		p_uid_edystone = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, &lengthmac );
        		if(p_uid_edystone != NULL)
        		{
        			if(memcmp(KEY,&p_uid_edystone[4],3)==0)  /* If i read NET, is the word that the provisioner send to add nodes  */
        			{										 /* Read:NET2000000 in the UID  */
    //    				WICED_BT_TRACE_ARRAY(p_uid_edystone, 16, "Beacon UID: ");
        				// Copy the information of the net
        				START_LED_provisioner();		/* Turn of the LED THAT indicate the closer of the provisioner */
        				wiced_hal_gpio_configure_pin(LED_NODE, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH); /* Turn on the led to indicate the provisioner*/
        				WICED_BT_TRACE("\n Copy net information \n");
        				copy_info_net(p_uid_edystone);   //*************  Complet UID
        			}
        		}
    		}
    		else if(conection_complet_node == 1)
    		{
    			/* Blink of responce when the provisioner safe the information of the node */
    			p_uid_edystone = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, &lengthmac );
    			if(memcmp(bda,&p_uid_edystone[4],6)==0)
    			{
    				WICED_BT_TRACE("\n Response \n");
    				stop_uid();
    				start_blink();
    				conection_complet_node =2;
    			}
    		}
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
