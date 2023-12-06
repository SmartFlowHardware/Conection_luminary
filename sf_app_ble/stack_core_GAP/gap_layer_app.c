/************************************************************************************************************************************
 * File: gap_layer_app.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *   This file contains It's the initialize of the Main Application, has functions related to initializing and managing the Bluetooth
 *   stack, beacon device, and advertisement data. It also includes a callback function to handle various Bluetooth management events
 *   in GAP, and to register callback events for GATT (connecting with other devices, peripherals). If a GAP event like
 *   BTM_BLE_REMOTE_CONNECTION_PARAM_REQ_EVT or BTM_BLE_CONNECTION_PARAM_UPDATE is received, this may involve a connection with a
 *   central device, leading to a switch to OTA mode.
 *   Additionally, it manages the re-broadcasting of Beacon-type advertisement data, including information that is being processed
 *   for the device.
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

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_rtos.h"
#include "cycfg.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_memory.h"
#include "wiced_bt_gatt_util.h"
#include "bt_types.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_beacon.h"
#include "wiced_bt_ble.h"
#include "wiced_timer.h"
#include "wiced_rtos.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_wdog.h"
#include "wiced_hal_rand.h"
#include "wiced_hal_gpio.h"
#include "wiced_gki.h"

#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_core.h"

#include "config_ports.h"
#include "gap_layer_app.h"


#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
/* Adv parameter used for multi-adv*/
wiced_bt_ble_multi_adv_params_t adv_param =
#else
wiced_bt_beacon_multi_advert_data_t adv_param =
#endif
{
    .adv_int_min = BTM_BLE_ADVERT_INTERVAL_MIN,
    .adv_int_max = BTM_BLE_ADVERT_INTERVAL_MAX,
    .adv_type = MULTI_ADVERT_NONCONNECTABLE_EVENT,
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,
    .adv_filter_policy = BTM_BLE_ADV_POLICY_FILTER_CONN_FILTER_SCAN,
    .adv_tx_power = MULTI_ADV_TX_POWER_MAX,
    .peer_bd_addr = {0},
    .peer_addr_type = BLE_ADDR_PUBLIC,
    .own_bd_addr = {0},
    .own_addr_type = BLE_ADDR_PUBLIC
};


/* Beacon timer */
//static wiced_timer_t beacon_timer;
uint16_t      beacon_conn_id = 0;

/* Holds the device application info */
//sensor_app_t g_device_app;

/************************************************************************************************************************************
 *  												Function Definitions
 ***********************************************************************************************************************************/

/************************************************************************************************************************************
 * Function Name: gap_stack_init(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The gap_stack_init function is responsible for initializing the Bluetooth stack and the application using the provided
 * 	configuration settings.
 * 	The gap_stack_init function is essential for setting up the Bluetooth stack and enabling the application to interact with the
 * 	stack for various Bluetooth operations.
 ***********************************************************************************************************************************/
void gap_stack_init(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Function to initialize configuration logs and consult data from flash for the name (Function of config_logs.h)
	init_config_logs();

	// Initialize the Bluetooth stack
	wiced_bt_stack_init( beacon_management_callback, &app_cfg_settings2, app_buf_pools2 );
}


/************************************************************************************************************************************
 * Function Name: beacon_init(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The beacon_init function is responsible for initializing the beacon device.
 * 	The beacon_init function is crucial for setting up the beacon device, configuring advertisements, and enabling other necessary
 * 	features for the Bluetooth Low Energy (BLE) operation.
 ***********************************************************************************************************************************/
void beacon_init(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

#ifdef CYW20706A2
#if defined(USE_256K_SECTOR_SIZE)
    // For CYW20706A2 and using 256K sector size, configure SFLASH erase sector size and 4-byte address mode.
    wiced_hal_sflash_use_erase_sector_size_256K(1);
    wiced_hal_sflash_use_4_byte_address(1);
#endif
#endif

    /* Register with stack to receive GATT callback */
    //gatt_status = wiced_bt_gatt_register( beacon_gatt_callback );
    //WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    //-----------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------

    // Register NULL for HCI trace to disable trace output for this function.
    wiced_bt_dev_register_hci_trace(NULL);

    /* Allow the peer to pair with this device. */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    // Initialize OTA firmware upgrade if applicable.
    /*
	WICED_BT_TRACE( "DATA 2 \n");
	numbytes = wiced_hal_read_nvram( WICED_NVRAM_VSID_START, sizeof(data_save_flash), &data_save_flash, &status );
	WICED_BT_TRACE_ARRAY( data_save_flash, 8 ,"DATA SAVED HEX: ");
	WICED_BT_TRACE( "DATA SAVED STRING: ");
	for(int i=0;i<8; i++){wiced_hal_puart_write(data_save_flash[i]);}
	WICED_BT_TRACE( "\n");
	*/

//#if OTA_FW_UPGRADE
    /* OTA Firmware upgrade Initialization */
/*#ifdef OTA_SECURE_FIRMWARE_UPGRADE
	if (!wiced_ota_fw_upgrade_init(&ecdsa256_public_key, NULL, NULL))
#else
    if (!wiced_ota_fw_upgrade_init(NULL, NULL, NULL))
#endif
{
	WICED_BT_TRACE("OTA upgrade Init failure !!! \n");
}
#endif*/


    // Set the advertising parameters and make the device discoverable.
    //beacon_set_app_advertisement_data2();

    // Re-broadcast low energy advertisements (commented out in the provided code).
    //!(is_provisioned)?(node_set_app_advertisement_data()):(gap_rebroadcastLR(1));

    // Depending of node mode this going to advertisement a package of information
    if(wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
    {
    	// Node unprovisioned
    	WICED_BT_TRACE("Node Adv--------\n\r");
    	wiced_hal_gpio_configure_pin(LED_CONECTION, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
    	is_provisioned = WICED_FALSE;
    	node_set_app_advertisement_data();
    	app_set_scan_response_data();
    }
    else
    {
    	// Node provisioned ( Created Network )
    	WICED_BT_TRACE("Mesh Adv--------\n\r");
    	//data_name_node = transmit_node_data(node, "NELAS");
    	wiced_hal_gpio_configure_pin(LED_CONECTION, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
    	is_provisioned = WICED_TRUE;
    	mesh_set_app_advertisement_data();
    	app_set_scan_response_data();
    }

    // Log message to indicate that UUID is cleared.
    WICED_BT_TRACE("CLEAR UUID\n");

    //-----------------------------------------------------------------------------------------------
    // Initialize the ADC configuration.
    //set_adc_p();

    // Initialize MAC logs by reading MAC address from flash.
    init_mac_logs();

	config_clk_timers(); 		// Configura los timers
	start_BTimers(); 			// Inician los timers

    //-----------------------------------------------------------------------------------------------

    // Start advertising the device in an undirected high duty cycle mode.
    // The first parameter (BTM_BLE_ADVERT_UNDIRECTED_HIGH) sets the advertising mode.
    // The second parameter (0) sets the duration (0 for infinite duration).
    // The third parameter (NULL) specifies that the scan response data is not provided.
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}


/************************************************************************************************************************************
 * Function Name: beacon_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function is a callback function that handles various Bluetooth management events. It is used in the context of a beacon
 * 	application. The function performs different actions based on the received event, such as initializing the beacon, configuring
 * 	UART port, scanning for devices, handling security, and managing advertisement.
 *
 * Parameters:
 * 	wiced_bt_management_evt_t event					: Variable to enum of Bluetooth management.
 * 	wiced_bt_management_evt_data_t *p_event_data	: Pointer to structure definitions of Bluetooth management.
 *
 * Return:
 *  wiced_result_t									: Enum of Bluetooth results (list).
 ***********************************************************************************************************************************/
wiced_result_t beacon_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Variable declarations
	wiced_result_t					result = WICED_BT_SUCCESS;
    uint8_t                         *p_keys;
    wiced_bt_ble_advert_mode_t      *p_mode;
    uint8_t 						close_interval = 34;
    uint8_t 						close_interval2 = 31;

    //WICED_BT_TRACE("beacon_management_callback: %d\n", event);

    // Check if the event is one of the close_interval events and handle it
    if( ( close_interval == event ) || ( close_interval2 == event ) )
    {
    	WICED_BT_TRACE("beacon_management_callback2: %x\n", event);
    	event_select_OTA();				// Change OTA mode
    }

    // Switch statement to handle various Bluetooth management events
    switch( event )
    {
    	case BTM_ENABLED_EVT:								/* Bluetooth stack enabled */
    		WICED_BT_TRACE( "BLUETOOTH EVENT\r\n" );
    		// Perform initialization and configuration tasks here
    		beacon_init();				// Start the application configuration
    		config_Transceiver(); 		// Configura puerto uart
    		start_observe();
//    		if(is_provisioned)
//    		{
//    			start_observe(); 			// Star the observer
//    		}

    		set_outPuts(); 				// Configura pines de salida
    		set_intPuts(); 				// Configura pines de entrada
    		register_pin_interrupt(); 	// Configura las interrupciones
    		//start_pwm();				// Inicia el PWM
    		//start_scanner();
    		break;

    	case BTM_DISABLED_EVT:								/* Bluetooth stack disabled event */
    		break;

    	case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:	/* Handling BLE pairing request event */
    		// Set the IO capabilities, authentication requirements, and encryption keys for pairing
    		p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
    		p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
    		p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
    		p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
    		p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
    		p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
    		break;

    	case BTM_SECURITY_REQUEST_EVT:
    		wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
    		break;

    	case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
    		break;

    	case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
    		/* No keys stored so we need to return error to get Stack to generate them */
    		result = WICED_BT_ERROR;
    		break;

    	case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
    		break;


    	case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
    		break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT:
			p_mode = &p_event_data->ble_scan_state_changed;
			switch( *p_mode )
			{
				case BTM_BLE_SCAN_TYPE_NONE:
					WICED_BT_TRACE( "Scanning stopped.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
					WICED_BT_TRACE( "High duty scanning.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_LOW_DUTY:
					WICED_BT_TRACE( "Low duty scanning.\r\n" );
					break;
			}
			break;

    	case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
    		p_mode = &p_event_data->ble_advert_state_changed;
    		WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
    		if (*p_mode == BTM_BLE_ADVERT_OFF)
    		{
    			beacon_advertisement_stopped();
    		}
    		break;

		case BTM_BLE_CONNECTION_PARAM_UPDATE:
			WICED_BT_TRACE( "Notification start\n");
			//wiced_bt_util_set_gatt_client_config_descriptor( connection_id, 38,  GATT_CLIENT_CONFIG_NOTIFICATION);
			break;

    	default:
    		break;
    }
    return result;
}


/************************************************************************************************************************************
 * Function Name: beacon_advertisement_stopped(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function helps ensure that the BLE advertisement remains active and discoverable when the application is not connected to any
 * 	device. If a connection exists, it indicates that the advertisement has stopped, and the function prints a trace message to
 * 	inform the developer.
 ***********************************************************************************************************************************/
void beacon_advertisement_stopped(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Declare a variable to store the WICED result
    wiced_result_t result;

    // Check if the application is not connected to any device (app_conn_id is 0)
    if (beacon_conn_id == 0)
    {
    	// If not connected, restart the BLE advertisement with low-power settings
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);

        // Print the result of starting advertisement
        WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result);
    }
    else
    {
    	// If the application is connected to a device, print a trace message indicating that the advertisement has stopped
        WICED_BT_TRACE("ADV stop\n");
    }
}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void beacon_set_app_advertisement_data2(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)app_cfg_settings2.device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)app_cfg_settings2.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

void beacon_set_app_advertisement_data3(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_result_t         result;
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)"1A5EK  BLE");
    adv_elem[num_elem].p_data       = (uint8_t*)"1A5EK  BLE";
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}


void mesh_set_app_advertisement_data(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_result_t         result;
    wiced_bt_ble_advert_elem_t adv_elem[4];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)"MESH1 BSL");
    adv_elem[num_elem].p_data       = (uint8_t*)"MESH1 BSL";
    num_elem++;

    adv_elem[num_elem].advert_type	= BTM_BLE_ADVERT_TYPE_DEV_CLASS;
    adv_elem[num_elem].len			= sizeof(mesh_device);
    adv_elem[num_elem].p_data		= mesh_device;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_MESH_BEACON;
    adv_elem[num_elem].len          = sizeof(mesh_beacon);
    adv_elem[num_elem].p_data       = mesh_beacon;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}


void node_set_app_advertisement_data(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_result_t         result;
    wiced_bt_ble_advert_elem_t adv_elem[4];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)"NODEL BSL");
    adv_elem[num_elem].p_data       = (uint8_t*)"NODEL BSL";
    num_elem++;

    adv_elem[num_elem].advert_type	= BTM_BLE_ADVERT_TYPE_DEV_CLASS;
    adv_elem[num_elem].len			= sizeof(app_cfg_settings2.device_class);
    adv_elem[num_elem].p_data		= app_cfg_settings2.device_class;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_MESH_BEACON;
    adv_elem[num_elem].len          = sizeof(mesh_beacon);
    adv_elem[num_elem].p_data       = mesh_beacon;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}



/*******************************************************************************
* Function Name: void app_set_scan_response_data( void )
********************************************************************************/
void app_set_scan_response_data( void )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    wiced_bt_ble_advert_elem_t adv_elem[1] = { 0 };
    uint8_t num_elem = 0;

    /* Advertisement Element for Service UUID */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_MESH_MSG;
    adv_elem[num_elem].len = sizeof(mesh_message);
    adv_elem[num_elem].p_data = mesh_message;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_scan_response_data( num_elem, adv_elem );
}




void gap_rebroadcastLR(int8_t slt)
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	//WICED_BT_TRACE("RSSI3:%d\n",rssi);
	//uint8_t *Data_Txg=NULL;
	//Data_Txg=&bda[0];
	//memcpy(data_mcc,Data_Txg,6);


	/* Set sample values for Eddystone UID*/
	uint8_t eddystone_ranging_data = 0xf0;
	uint8_t eddystone_namespace[EDDYSTONE_UID_NAMESPACE_LEN];
	uint8_t eddystone_instance[EDDYSTONE_UID_INSTANCE_ID_LEN];
	uint16_t time_send_data;

	switch( slt )
	{

		case 0:
			// Start advertisement the Node
	    	wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_UID);
	    	node_set_app_advertisement_data();
	        return;

		case 1:
			// Star advertisement the Node in the Network
	    	wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_UID);
	    	mesh_set_app_advertisement_data();
	    	return;

		default:
	    	//Clean the main array
	    	//memset(data_txsf, 0, sizeof(data_txsf));

	    	// Add the identifier ( 'SA' ) at first four values
	    	//memcpy(data_txsf, array_int_send, 2);

	    	// Add the Bluetooth device address and the Beacon ID
	    	//memcpy(data_txsf + 2, array_bd_addr, 9);

	    	// Add not name here
	    	// Clean the eddystone_instance
	    	//memset(eddystone_instance, 0, sizeof(eddystone_instance));
			break;
	}

	//memcpy(data_txsf + sizeof(filt_sf1), &data_mcc, sizeof(data_mcc));
	//data_txsf[8]=rssi;
	//data_txsf[2]=slt;

	WICED_BT_TRACE("Data_Txg1 ");
	wiced_hal_puart_print(data_txsf);
	WICED_BT_TRACE("\n");
	WICED_BT_TRACE_ARRAY(data_txsf, 16, "DATARRXXXX ");

	//-----------------------------------------------------------------------------------------------
    uint8_t adv_data_uid[31];
    uint8_t adv_len_uid = 0;

    memset(adv_data_uid, 0, 31);

    /* Call Eddystone UID api to prepare adv data*/
    wiced_bt_eddystone_set_data_for_uid(eddystone_ranging_data,  (uint8_t*)data_txsf, eddystone_instance, adv_data_uid, &adv_len_uid);

    /* Sets adv data for multi adv instance*/
    wiced_set_multi_advertisement_data(adv_data_uid, adv_len_uid, BEACON_EDDYSTONE_UID);

    /* Start Eddystone UID advertisements */
    adv_param.adv_int_min = time_send_data; // 1000 ms
    adv_param.adv_int_max = time_send_data;

#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
    wiced_set_multi_advertisement_params(BEACON_EDDYSTONE_UID, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BEACON_EDDYSTONE_UID, adv_param.adv_tx_power);
#endif

    wiced_start_multi_advertisements(MULTI_ADVERT_START, BEACON_EDDYSTONE_UID);

	/*wiced_result_t         result;
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)Data_Txg);
    adv_elem[num_elem].p_data       = (uint8_t*)Data_Txg;
    num_elem++;

    WICED_BT_TRACE("Data_Txg2");
    wiced_hal_puart_print(Data_Txg);
    WICED_BT_TRACE("\n");
    rssi2=Data_Txg[8];
    WICED_BT_TRACE("RSSI4:%d\n",rssi2);

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);*/
}






void gap_rebroadcastBIO(uint8_t t_sensor, uint16_t data_device)
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	//WICED_BT_TRACE("RSSI3:%d\n",rssi);
	//uint8_t *Data_Txg=NULL;
	//Data_Txg=&bda[0];
	//memcpy(data_mcc,Data_Txg,6);

    uint8_t first_part 	= (uint8_t)(data_device >> 8);		// Get the byte high
    uint8_t second_part = (uint8_t)(data_device & 0xFF);	// Get the byte low

    uint8_t thousands;
    uint8_t hundreds;
    uint8_t tens;
    uint8_t units;

	/* Set sample values for Eddystone UID*/
	uint8_t eddystone_ranging_data = 0xf0;
	uint8_t eddystone_namespace[EDDYSTONE_UID_NAMESPACE_LEN];
	uint8_t eddystone_instance[EDDYSTONE_UID_INSTANCE_ID_LEN];
	uint16_t time_send_data;

	uint8_t scanner_active[]={0x01,0x01,0x01};
	uint8_t scanner_innactive[]={0x00,0x00,0x00};

	switch( t_sensor )
	{

		case 0:
			//memcpy(data_txsf,filt_NULL,3);
	    	wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BEACON_EDDYSTONE_UID);
	    	node_set_app_advertisement_data();
	        return;

	    // Case for send data from Health Thermometer
	    case 1:
	    	//Clean the main array
	    	memset(data_txsf, 0, sizeof(data_txsf));

	    	// Add the identifier ( 'HT' ) at first four values
	    	memcpy(data_txsf, device_id_HT, sizeof(device_id_HT));

	    	thousands = data_device / 1000;
	    	hundreds = ( data_device % 1000 ) / 100;

	    	first_part = ( thousands*10 ) + hundreds;

	    	tens = ( data_device % 100 ) / 10;
	    	units = ( data_device % 10 );

	    	second_part= ( tens*10 ) + units ;

	    	// Add the Bluetooth device address and the Beacon ID
	    	memcpy(data_txsf + 2, &first_part, sizeof(first_part));

	    	// Add the Bluetooth device address and the Beacon ID
	    	memcpy(data_txsf + 3, &second_part, sizeof(second_part));

	    	// Add not name here
	    	// Clean the eddystone_instance
	    	//memset(eddystone_instance, 0, sizeof(eddystone_instance));

	    	time_send_data = 450;	// 250 miliseconds

	        break;

		// Case for send data from Heart Rate
		case 2:
			//Clean the main array
			memset(data_txsf, 0, sizeof(data_txsf));

			// Add the identifier ( 'HR' ) at first four values
			memcpy(data_txsf, device_id_HR, sizeof(device_id_HR));

			// Add the Bluetooth device address and the Beacon ID
			memcpy(data_txsf + 2, &second_part, sizeof(second_part));

			// Add the Bluetooth device address and the Beacon ID
			memcpy(data_txsf + 3, &first_part, sizeof(first_part));


			// Separate the URL into two arrays
			//separate_array_into_two( array_name_beacon, &data_txsf[2], eddystone_instance ); // &data_txsf[2]

			//memcpy(data_txsf + 10, eddystone_instance, 16);

			// Add the Bluetooth device address and the Beacon ID
			//memcpy(data_txsf + 2, array_bd_addr, 9);

			// Clean the eddystone_instance
			//memset(eddystone_instance, 0, sizeof(eddystone_instance));

			// Add the name of Beacon 2 at the eddystone instance
			//memcpy(eddystone_instance, array_name_beacon, EDDYSTONE_UID_INSTANCE_ID_LEN);

			time_send_data = 400;	// 250 miliseconds

		    break;

		case 3:
			//Clean the main array
			memset(data_txsf, 0, sizeof(data_txsf));

			memcpy(data_txsf, scanner_active, sizeof(scanner_active));
			break;

		case 4:
			//Clean the main array
			memset(data_txsf, 0, sizeof(data_txsf));

			memcpy(data_txsf, scanner_innactive, sizeof(scanner_innactive));
			break;

		default:
			break;
	}

	//memcpy(data_txsf + sizeof(filt_sf1), &data_mcc, sizeof(data_mcc));
	//data_txsf[8]=rssi;
	//data_txsf[2]=slt;

//	WICED_BT_TRACE("Data_Txg1 ");
//	wiced_hal_puart_print(data_txsf);
//	WICED_BT_TRACE("\n");
//	WICED_BT_TRACE_ARRAY(data_txsf, 16, "DATARRXXXX ");

	//-----------------------------------------------------------------------------------------------
    uint8_t adv_data_uid[31];
    uint8_t adv_len_uid = 0;

    memset(adv_data_uid, 0, 31);

    /* Call Eddystone UID api to prepare adv data*/
    wiced_bt_eddystone_set_data_for_uid(eddystone_ranging_data,  (uint8_t*)data_txsf, eddystone_instance, adv_data_uid, &adv_len_uid);

    /* Sets adv data for multi adv instance*/
    wiced_set_multi_advertisement_data(adv_data_uid, adv_len_uid, BEACON_EDDYSTONE_UID);

    /* Start Eddystone UID advertisements */
    adv_param.adv_int_min = time_send_data; // 1000 ms
    adv_param.adv_int_max = time_send_data;

#if defined(CYW20735B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX)
    wiced_set_multi_advertisement_params(BEACON_EDDYSTONE_UID, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BEACON_EDDYSTONE_UID, adv_param.adv_tx_power);
#endif

    wiced_start_multi_advertisements(MULTI_ADVERT_START, BEACON_EDDYSTONE_UID);

	/*wiced_result_t         result;
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)Data_Txg);
    adv_elem[num_elem].p_data       = (uint8_t*)Data_Txg;
    num_elem++;

    WICED_BT_TRACE("Data_Txg2");
    wiced_hal_puart_print(Data_Txg);
    WICED_BT_TRACE("\n");
    rssi2=Data_Txg[8];
    WICED_BT_TRACE("RSSI4:%d\n",rssi2);

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);*/
}








/************************************************************************************************************************************
 * Function Name: stop_rbdkst(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function stops the rebroadcasting of advertisement data. This happens by calling the rebroadcasting function with a
 * 	parameter value of zero, which is responsible for halting the advertisement.
 ***********************************************************************************************************************************/
void stop_rbdkst(void)
{
	 //node_set_app_advertisement_data();
	 gap_rebroadcastLR(0);
	 WICED_BT_TRACE("CLEAR UUID\n");
}





extern void process_Write(uint8_t *data_Write);
extern void process_CMA(uint8_t *data_C_MA);
extern void process_data_config(uint8_t *data_dc);
extern void process_ODT(uint8_t *data_ODT);
extern void process_SOM(uint8_t *data_S_OM);


uint8_t Uart_BuffRX[64];
int cont_buffRX=0;
// Callback function for the transceiver interrupt service
void rx_cback(void *data)
{
    uint8_t readbyte;
    uint8_t *dat = NULL;
    wiced_result_t result;
    uint8_t buffer[(6 * 2) + 16];
    wiced_bt_mesh_core_init_t init = { 0 };

    /* Read one byte from the buffer and reset the interrupt (unlike GPIO) */
    wiced_hal_puart_read(&readbyte);
    wiced_hal_puart_reset_puart_interrupt();

    dat = &readbyte;
    Uart_BuffRX[cont_buffRX] = dat[0];
    cont_buffRX++;

    // Check if the received byte is a newline character
    if (readbyte == '\n')
    {
        cont_buffRX = 0;  // Reset buffer index when receiving a newline

        // Process different commands based on the first character of the received buffer
        switch (Uart_BuffRX[0])
        {
            case 'W':
                //process_Write(Uart_BuffRX);		// Command to input MAC address
                break;

            case 'C':
                //process_CMA(Uart_BuffRX);			// Command for random MAC address
                break;

            case 'R':
                //process_data_config(Uart_BuffRX);	// Command to query MAC and name
                break;

            case 'O':
                //process_ODT(Uart_BuffRX);			// Command for ODT
                break;

            case 'S':
                //process_SOM(Uart_BuffRX);			// Command to send to OTA
                break;
        }

        memset(Uart_BuffRX, '\0', 64);				// Clear the buffer after processing
    }
}
