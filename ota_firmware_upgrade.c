/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * Bluetooth Over the Air (OTA) Firmware Upgrade Application
 *
 * This is a stub application that demonstrates how to link with the
 * OTA FW Upgrade library.  The application responsibility is to
 * publish OTA FW upgrade service in the GATT database and to pass
 * stack callbacks to the library.
 * See libraries/fw_upgrade_lib/ota_fw_upgrade.c file for the
 * description of the OTA protocol.
 * This version of the OTA firmware upgrade relies on the Bluetooth
 * standard security.  The application also can use ECC cryptography
 * to validate the image.
 * It is expected that full implementation
 * will use application level verification that downloaded firmware is
 * for this Product ID and that the image has not been tempered with.
 *
 * Features demonstrated
 *  - OTA Firmware Upgrade
 *
 *  Applications to perform and verify OTA FW upgrade are provided for Windows 10 and Android platforms
 *  Executable and source code are provided
 *  Windows Peer App: WsOTAUpgrade.exe
 *  Android Peer App: LeOTAApp (app-debug.apk)
 *  Windows and Android applications can be used to perform and verify secure (signed) and unsecured (unsigned) OTA upgrade
 *  Both applications accept a secure (signed) or unsecured (unsigned) OTA binary images as input (*.ota.bin & *.ota.bin.signed)
 *
 * To use OTA, find the peer OTA applications in folder below and follow the readme.txt.
 * ModusToolbox - <Install Dir>\ModusToolbox_1.1\libraries\bt_sdk-1.x\components\BT-SDK
 * WICED Studio - <Install Dir>\WICED-Studio-X.X
 * \common\peer_apps\ota_firmware_upgrade\readme.txt
 *
 * NOTE: 20719B1 platforms - During an attempt to reboot if the board notices that programs like ClientControl/TeraTerm have an UART/PUART open for tracing etc.
 * then the board continues to wait for HCI commands over UART and prevents successful reboot. This is a currently by design.
 * Please close the UART/PUART ports as the upgrade process nears completion and the OTA firmware should upgrade and reboot as expected.
 *
 */
#include "sparcommon.h"							/**< Include: Common definitions and macros used in the WICED SDK. */
#include "wiced_bt_dev.h"						/**< Include: Necessary functions and structures to interact with the Bluetooth management device. */
#include "wiced_bt_ble.h"						/**< Include: Contains definitions and functions specific to working with Bluetooth Low Energy (BLE). */
#include "wiced_bt_gatt.h"						/**< Include: GATT protocol (Generic Attribute Profile) in Bluetooth. */
#include "wiced_bt_cfg.h"						/**< Include: Contains the specific configuration of the Bluetooth stack on the WICED platform. */
#include "wiced_bt_uuid.h"						/**< Include: Definitions and functions related to the manipulation and management of service identifiers, features and descriptions in Bluetooth. */
#include "wiced_timer.h"
#include "wiced_hal_nvram.h"					/**< Include: Function and definitions for accessing non-volatile memory. */
#include "wiced_platform.h"						/**< Include: Pin definitions, hardware configurations, and other features specific to the WICED platform. */
#include "wiced_bt_trace.h"						/**< Include: Contains functions and macros to perform traces or debugging messages related to Bluetooth communication. */
#include "wiced_transport.h"					/**< Include: Transport data between Bluetooth devices or use different transport protocols for communication. */

/* Include OTA library */
#include "wiced_bt_ota_firmware_upgrade.h"		/**< Include: Over-the-Air (OTA) firmware upgrades over Bluetooth. */
#include "wiced_bt_stack.h"						/**< Include: Configure and manage the Bluetooth stack on the WICED platform. */
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif

/* Include Header Files */
#include "string.h"								/**< Include: Standard C library that provides functions for handling strings. */
#include "wiced_hal_puart.h"					/**< Include: Library related to the peripheral UART (PUART) functionality in the WUCED SDK. */
#include "wiced_hal_wdog.h"						/**< Include: Library for hardware watchdog timer management. */

#include "config_ports.h"


/************************************************************************************************************************************
 *                                							Constants/Macros
 ***********************************************************************************************************************************/

// Define specific handles and values related to the OTA (Over-the-Air) Firmware Upgrade functionality
#define HANDLE_OTA_FW_UPGRADE_GATT_SERVICE              1								/**< Handle for the GATT service related to OTA Firmware Upgrade. */
#define HANDLE_OTA_FW_UPGRADE_GAP_SERVICE               2								/**< Handle for the GAP service related to OTA Firmware Upgrade. */
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME             3								/**< Handle for the characteristic device name related to OTA Firmware Upgrade.*/
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME_VAL         4								/**< Handle for the value of the characteristic device name related to OTA Firmware Upgrade. */
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE       5								/**< Handle for the characteristic device appearance related to OTA Firmware Upgrade. */
#define HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE_VAL   6								/**< Handle for the value of the characteristic device appearance related to OTA Firmware Upgrade.*/

#define OTA_FW_UPGRADE_LOCAL_KEYS_VS_ID                 WICED_NVRAM_VSID_START			/**< Virtual Storage ID for storing local keys used in OTA Firmware Upgrade. */
#define OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID           (WICED_NVRAM_VSID_START + 1)	/**< Virtual Storage ID for storing peer device keys used in OTA Firmware Upgrade. */

#define CLOCK_CHANGE_STACK								10

#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include "bt_types.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"

// If secure version of the OTA firmware upgrade is used, the app should be linked with the ecdsa256_pub.c
// which exports the public key
extern Point    ecdsa256_public_key;
#endif



 /************************************************************************************************************************************
  *  												Constants Definitions GATT/Transport
  ***********************************************************************************************************************************/

/* This is the GATT database for the OTA upgrade application. The database defines mandatory GATT and GAP service and OTA FW Upgrade
 * service itself.
 *
 * To merge OTA functionality into a different application, copy service UUID_OTA_FW_UPGRADE_SERVICE and relative characteristic to
 * the database of the target application.
 *
 * Note that the handles do not need to be sequential, but need to be sorted in ascending order. The first handle of the service is
 * HANDLE_OTA_FW_UPGRADE_SERVICE = 0xff00. The service needs to be placed at the end of the device's GATT database. */
const uint8_t ota_fw_upgrade_gatt_database[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16(HANDLE_OTA_FW_UPGRADE_GATT_SERVICE, UUID_SERVICE_GATT),

    // Declare mandatory GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16(HANDLE_OTA_FW_UPGRADE_GAP_SERVICE, UUID_SERVICE_GAP),

        // Declare mandatory GAP service characteristic: Dev Name
        CHARACTERISTIC_UUID16(HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME, HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME_VAL,
            UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE /*| LEGATTDB_PERM_AUTH_READABLE*/),

        // Declare mandatory GAP service characteristic: Appearance
        CHARACTERISTIC_UUID16(HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE, HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE_VAL,
            UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE /*| LEGATTDB_PERM_AUTH_READABLE*/),

    // Handle 0xff00: Broadcom vendor specific WICED Upgrade Service.
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE),
#else
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE),
#endif
        // Handles 0xff03: characteristic WS Control Point, handle 0xff04 characteristic value.
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ /*| LEGATTDB_PERM_AUTH_WRITABLE*/),

            // Declare client characteristic configuration descriptor
            // Value of the descriptor can be modified by the client
            // Value modified shall be retained during connection and across connection
            // for bonded devices.  Setting value to 1 tells this application to send notification
            // when value of the characteristic changes.  Value 2 is to allow indications.
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ /*| LEGATTDB_PERM_AUTH_WRITABLE */),

        // Handle 0xff07: characteristic WS Data, handle 0xff08 characteristic value. This
        // characteristic is used to send next portion of the FW Similar to the control point
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),
};

// Define and initialize a constant variable 'transport_cfg' of type 'wiced_transport_cfg_t'.
const wiced_transport_cfg_t  transport_cfg =
{
    .type = WICED_TRANSPORT_UART,					// Specify the type of transport to be used as UART (Universal Asynchronous Receiver/Transmitter).
    .cfg =											// Provide configuration details for the UART transport.
    {
        .uart_cfg =									// Set the UART mode to HCI mode (Host Controller Interface).
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD		// Use the default baud rate for HCI UART communication.
        },
    },
    .rx_buff_pool_cfg =								// Configure the RX buffer pool for the transport.
    {
// Set the buffer size for received data based on the device type.
#if ( defined(CYW20706A2) || defined(CYW20835B1) )
        .buffer_size  = 0,							// For specific device types, set buffer size and count to zero.
        .buffer_count = 0
#else
        .buffer_size  = 1024,						// For other device types, set a buffer size of 1024 bytes and buffer count to 1.
        .buffer_count = 1
#endif
    },
    .p_status_handler = NULL,						// Set the status handler to NULL, meaning no status handling is performed.
    .p_data_handler = NULL,							// Set the data handler to NULL, meaning no data handling is performed.
    .p_tx_complete_cback = NULL						// Set the transmit complete callback to NULL, meaning no callback function is defined.
};



/************************************************************************************************************************************
 *  												Function Declarations
 ***********************************************************************************************************************************/

static void                     app_init(void);
static wiced_result_t           app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                     app_advertisement_stopped(void);
static wiced_bt_gatt_status_t   app_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                     app_hci_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
static void                     app_set_advertisement_data(void);
static wiced_bool_t             app_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t             app_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static void                     app_timeout(uint32_t count);
static wiced_bt_gatt_status_t   app_connection_status_event(wiced_bt_gatt_connection_status_t *p_status);
static wiced_bt_gatt_status_t   app_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t   app_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t   app_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data);
static wiced_bt_gatt_status_t   app_indication_cfm_handler(uint16_t conn_id, uint16_t handle);

void							handle_OTA_change_reset( uint32_t data );
void							event_select_OTA(void);

extern void						gap_stack_init(void);


/***********************************************************************************************************************************
 * 													Global/Static Variables
 ***********************************************************************************************************************************/

wiced_result_t	status10;
uint16_t		numbytes10;

wiced_timer_t	ota_app_timer;
wiced_timer_t	ota_change_reset_timer;
uint16_t		app_conn_id = 0;
uint8_t			OTA_Mode_Enabled = 0;

extern const wiced_bt_cfg_settings_t app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t app_buf_pools[];



/***********************************************************************************************************************************
 * 													Function Definitions
 ***********************************************************************************************************************************/

/************************************************************************************************************************************
 * 													 APPLICATION_START
 * ----------------------------------------------------------------------------------------------------------------------------------
 * 	Entry point to the application. Set device configuration and start BT stack initialization.  The actual application initialization
 * 	will happen when stack reports that BT device is ready.
 ***********************************************************************************************************************************/
APPLICATION_START()
{
	// Initialize the transport layer with the given configuration.
    wiced_transport_init(&transport_cfg);

#ifdef WICED_BT_TRACE_ENABLE			// Enable Bluetooth trace logging if defined.
    // Set the debug UART to use the default baud rate of 115200 for PUART (Peripheral UART).
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

#ifdef CYW20706A2						// If the target is CYW20706A2, select the UART pads for PUART.
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
//    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    //Read the value stored in non-volatile memory (NVRAM) at the given address and store it in 'OTA_Mode_Enabled'
    numbytes10 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START + 10, sizeof(OTA_Mode_Enabled), &OTA_Mode_Enabled, &status10 );

    if( OTA_Mode_Enabled )
    {
        // Mode OTA (Over-The-Air) - Switching to OTA mode.
        WICED_BT_TRACE("--- Mode OTA (Over-The-Air) --- \r\n--- NAME DEVICE: %s ---\r\n", app_cfg_settings.device_name);

        // Initialize a timer called 'timer_stack' to call the function 'handle_OTA_change_reset' periodically, with a delay of 0 seconds.
        wiced_init_timer( &ota_change_reset_timer, handle_OTA_change_reset, 0, WICED_SECONDS_TIMER );

        // Start the timer to execute the 'f_stack' function repeatedly according to the delay specified.
        wiced_start_timer( &ota_change_reset_timer, CLOCK_CHANGE_STACK );

        // Register call back and configuration with stack
        wiced_bt_stack_init( app_management_callback, &app_cfg_settings, app_buf_pools );
    }
    else
    {
        // Mode Application Bluetooth - Switching to Application Bluetooth.
        WICED_BT_TRACE("--- Mode Application Bluetooth ---\r\n");

        // Turn On the Charge LED
        wiced_hal_gpio_configure_pin(LED_CHARGE, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

        // Initialize the Generic Access Profile (GAP) stack for Bluetooth Low Energy communication.
        gap_stack_init();
    }
}

/************************************************************************************************************************************
 * Function Name: app_init(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The app_init function is responsible for initializing the Bluetooth Low Energy (BLE) application. It performs various tasks, such
 *	as registering GATT (Generic Attribute Profile) callbacks, initializing the GATT database, starting advertisements, enabling
 *	pairing, and setting up OTA (Over-the-Air) firmware upgrade functionality if applicable. Additionally, it sets up the HCI (Host
 *	Controller Interface) trace callback and an application timer for periodic timeouts.
 *
 * 	Overall, the app_init function ensures that the BLE application is properly configured and ready to perform its intended
 * 	functionality, including advertising its presence and handling GATT operations for data exchange with other Bluetooth devices.
 ***********************************************************************************************************************************/
void app_init(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Declare variables to store GATT status and WICED result
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

#if !defined(CYW20835B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2)
    wiced_bt_app_init();									// Initialize WICED app for the relevant platform
#endif

#ifdef CYW20706A2
    #if defined(USE_256K_SECTOR_SIZE)
    wiced_hal_sflash_use_erase_sector_size_256K(1);			// Set the serial flash erase sector size to 256K if needed
    wiced_hal_sflash_use_4_byte_address(1);					// Use 4-byte addressing mode for the serial flash if needed
    #endif
#endif

    // Register with stack to receive GATT callback
    gatt_status = wiced_bt_gatt_register(app_gatts_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    //  Initialize the GATT database with the provided GATT database structure
    gatt_status =  wiced_bt_gatt_db_init(ota_fw_upgrade_gatt_database, sizeof(ota_fw_upgrade_gatt_database));
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

    // Register the HCI trace callback function to receive trace data from the Bluetooth stack
    wiced_bt_dev_register_hci_trace(NULL);

#ifdef WICED_BT_TRACE_ENABLE
    // Starting the app timer with periodic timeouts
    wiced_init_timer(&ota_app_timer, app_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
    wiced_start_timer(&ota_app_timer, 1);
#endif

    // Allow the peer device to pair with this device
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    // OTA Firmware upgrade Initialization
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    if (!wiced_ota_fw_upgrade_init(&ecdsa256_public_key, NULL, NULL))
#else
    if (!wiced_ota_fw_upgrade_init(NULL, NULL, NULL))
#endif
    {
        WICED_BT_TRACE("OTA upgrade Init failure !!! \n");
    }

    // Set the advertising parameters and make the device discoverable
    app_set_advertisement_data();

    // Start Bluetooth advertisements with the specified advertising mode and parameters
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}


/************************************************************************************************************************************
 * Function Name: app_hci_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Pass protocol traces up through the UART. The app_hci_trace_callback function serves as an HCI (Host Controller Interface) trace
 *	callback for the application. It is called when the Bluetooth stack generates HCI trace data during its operation.
 *
 * 	The main purpose of this function is to forward the received HCI trace data to the transport layer for further processing, such
 * 	as logging or debugging. It allows the application to capture and analyze the low-level Bluetooth communication data for monitoring
 * 	and troubleshooting purposes.
 *
 * Parameters:
 * 	wiced_bt_hci_trace_type_t type			: A parameter indicating the type of HCI trace data, such as commands, events, or data.
 * 	uint16_t length							: An integer parameter specifying the length of the HCI trace data.
 * 	uint8_t *p_data							: A pointer to a buffer containing the HCI trace data
 ***********************************************************************************************************************************/
void app_hci_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Send the received HCI trace data to the transport layer for further processing. The follow function is used for this purpose.
    wiced_transport_send_hci_trace(NULL,	// The first argument is set to NULL, meaning that the callback doesn't require a specific context.
    							   type,	// The 'type' parameter indicates the type of HCI trace data (e.g., command, event, data).
								   length,	// The 'length' parameter specifies the length of the HCI trace data.
								   p_data);	// The 'p_data' parameter points to the buffer containing the HCI trace data.
}


/************************************************************************************************************************************
 * Function Name: app_set_advertisement_data(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	This code defines a function app_set_advertisement_data to set the advertisement data for BLE advertising. It creates an array
 *	adv_elem to store BLE advertisement elements and a variable num_elem to track the number of elements in the array.
 *	The function then sets the advertisement flags and the complete device name as advertisement elements.
 ***********************************************************************************************************************************/
void app_set_advertisement_data(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Declare an array of BLE advertisement elements and a variable to track the number of elements
    wiced_bt_ble_advert_elem_t adv_elem[2];
    uint8_t num_elem = 0;

    // Define the advertising flags (General Discoverable and BR/EDR (Basic Rate/Enhanced Data Rate) not supported)
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    // Set the first advertisement element to indicate advertising flags
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    // Set the second advertisement element to indicate the complete device name
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)app_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)app_cfg_settings.device_name;
    num_elem++;

    // Set the raw advertisement data with the specified advertisement elements
    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}


/************************************************************************************************************************************
 * Function Name: app_advertisement_stopped(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 * 	This function is called when BLE advertising has stopped. It checks if the application is connected to any device. If not
 * 	connected, it restarts BLE advertising with low-power settings. If the application is connected, it prints a trace message
 * 	indicating that the advertising has stopped.
 ***********************************************************************************************************************************/
void app_advertisement_stopped(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Declare a variable to store the WICED result
    wiced_result_t result;

    // Check if the application is not connected to any device (app_conn_id is 0)
    if (app_conn_id == 0)
    {
        // If not connected, restart the BLE advertising with low-power settings
        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result);
    }
    else
    {
        // If the application is connected to a device, print a trace message indicating that the advertisement has stopped
        WICED_BT_TRACE("ADV stop\n");
    }
}


/************************************************************************************************************************************
 * Function Name: app_timeout(uint32_t count)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	This function is a timer callback that is called periodically after a specified interval. It keeps track of the number of timer
 *	timeouts and prints trace messages to indicate the current state. If the number of timer timeouts exceeds 120 (equivalent to 2
 *	minutes), it prints a trace message indicating a system reset and then performs a system reset using the watchdog. The function
 *	prints trace messages at each timeout, indicating the current count and "BLE Online" status.
 *
 * Parameters:
 * 	uint32_t count						: Variable size of 32 bits
 ***********************************************************************************************************************************/
void app_timeout(uint32_t count)
{
	WICED_BT_TRACE( "[%s]\r\n", __FUNCTION__ );

	// Declare a variable to keep track of the number of timer timeouts
    static uint32_t timer_count = 0;
    WICED_BT_TRACE("timer count: %d\n", timer_count++);

    // Blinking LED of charge
	(wiced_hal_gpio_get_pin_input_status(LED_CHARGE))?(wiced_hal_gpio_set_pin_output(LED_CHARGE, GPIO_PIN_OUTPUT_LOW)):(wiced_hal_gpio_set_pin_output(LED_CHARGE, GPIO_PIN_OUTPUT_HIGH));

    // Check if 'timer_count' has reached a value grater than 120 seconds ( 2 Minutes )
    if( timer_count > 120 )
    {
    	WICED_BT_TRACE("RST SYSTEM\r\n");
    	wiced_hal_wdog_reset_system();
    }
}


/************************************************************************************************************************************
 * Function Name: wiced_bt_dev_status_t app_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	This function serves as a callback for handling various Bluetooth management events. When called with a specific event, it
 *	performs different actions accordingly. The events it handles include Bluetooth stack initialization, pairing requests,
 *	encryption status, security access, key updates, link key requests, and advertisement state changes. Additionally, it configures
 *	the PWM and LED states based on the advertisement and connection status.
 *
 * Parameters:
 * 	wiced_bt_management_evt_t event						: Variable with Bluetooth management events.
 * 	wiced_bt_management_evt_data_t *p_event_data		: Pointer to structure with information about the status of Bluetooth.
 ***********************************************************************************************************************************/
wiced_result_t app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
	WICED_BT_TRACE("[%s]: %x\r\n", __FUNCTION__, event);

    wiced_result_t                    result = WICED_BT_SUCCESS;
    uint8_t                          *p_keys;
    wiced_bt_ble_advert_mode_t       *p_mode;

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:									// Bluetooth Controller and Host Stack Enabled
        app_init();
        break;

    case BTM_DISABLED_EVT:									// Bluetooth Controller and Host Stack Disabled
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:		// IO capabilities request
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        break;

	case BTM_PAIRING_COMPLETE_EVT: 							// Pairing Complete event
		break;

	case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
		break;

    case BTM_SECURITY_REQUEST_EVT:							// Security access
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:			// Save link keys with application
        app_save_link_keys(&p_event_data->paired_device_link_keys_update);

#ifdef CYW20706A2
        wiced_bt_dev_add_device_to_address_resolution_db(&p_event_data->paired_device_link_keys_update, p_event_data->paired_device_link_keys_update.key_data.ble_addr_type);
#else
        wiced_bt_dev_add_device_to_address_resolution_db(&p_event_data->paired_device_link_keys_update);
#endif
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:			// Retrieval saved link keys
        if (app_read_link_keys(&p_event_data->paired_device_link_keys_request))
        {
            WICED_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

     case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:				// Save keys to NVRAM
         /* save keys to NVRAM */
         p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
         wiced_hal_write_nvram ( OTA_FW_UPGRADE_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
         WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
         break;


     case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:				// Read keys from NVRAM
         /* read keys from NVRAM */
         p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
         wiced_hal_read_nvram( OTA_FW_UPGRADE_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
         WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
         break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Scan State Change
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        if (*p_mode == BTM_BLE_ADVERT_OFF)
        {
            app_advertisement_stopped();
        }
        break;

    default:
        break;
    }

    return result;
}


/************************************************************************************************************************************
 * Function Name: wiced_bt_gatt_status_t app_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Process Read request or command from peer device. Define a function to handle GATT Read requests from the peer device.
 *
 * Parameter
 * 	conn_id					: State of the connection.
 * 	* p_read_data			: Pointer at structure with Attribute read request.
 *
 * Return
 * 	wiced_bt_gatt_status_t	: After handling the read request, the function returns success status. GATT STATUS (codes).
 ***********************************************************************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    int to_copy;

    // Check if the read request is for the OTA FW upgrade service, if so, pass it to the library for processing
    if (p_read_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }

    // Handle different characteristics based on the handle provided in the read request
    switch(p_read_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CHAR_DEV_NAME_VAL:
    	// Check if the read offset is beyond the length of the device name
        if (p_read_data->offset >= strlen((const char *)app_cfg_settings.device_name))
            return WICED_BT_GATT_INVALID_OFFSET;

        // Calculate the amount of data to copy based on the remaining device name length and the provided value length
        to_copy = strlen((const char *)app_cfg_settings.device_name) - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        // Copy the appropriate portion of the device name to the provided buffer
        memcpy(p_read_data->p_val, app_cfg_settings.device_name + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    case HANDLE_OTA_FW_UPGRADE_CHAR_DEV_APPEARANCE_VAL:
    	// Check if the read offset is beyond the size of the appearance value (2 bytes)
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        // Calculate the amount of data to copy based on the remaining appearance value size and the provided value length
        to_copy = 2 - p_read_data->offset;
        if (*p_read_data->p_val_len < to_copy)
            to_copy = *p_read_data->p_val_len;

        // Copy the appropriate portion of the appearance value to the provided buffer
        memcpy(p_read_data->p_val, ((uint8_t*)&app_cfg_settings.gatt_cfg.appearance) + p_read_data->offset, to_copy);
        *p_read_data->p_val_len = to_copy;
        break;

    default:
    	// If the provided handle does not match any known characteristics, return an invalid handle status
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    // Return success status after handling the read request
    return WICED_BT_GATT_SUCCESS;
}


/************************************************************************************************************************************
 * Function Name: wiced_bt_gatt_status_t app_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Process write request or write command from peer device. Define a function to handle GATT Write requests from the peer device.
 *
 * Parameter
 * 	conn_id					: State of the connection.
 * 	* p_write_data			: Pointer at structure with Attribute write request.
 *
 * Return
 * 	wiced_bt_gatt_status_t	: After handling the write request, the function returns success status. GATT STATUS (codes).
 ***********************************************************************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Check if the write request is for the OTA FW upgrade service, if so, pass it to the library for processing
    if (p_write_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_write_data);
    }

    // If the provided handle does not match any known characteristics, return an invalid handle status
    return WICED_BT_GATT_INVALID_HANDLE;
}


/************************************************************************************************************************************
 * Function Name: wiced_bt_gatt_status_t app_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Process indication_confirm from peer device. Define a function to handle GATT Indication Confirmations from the peer device.
 *
 * Parameter
 * 	conn_id					: State of the connection.
 * 	handle					: Handle of 16 bits.
 *
 * Return
 * 	wiced_bt_gatt_status_t	: After handling the indication, the function returns status. GATT STATUS (codes).
 ***********************************************************************************************************************************/
wiced_bt_gatt_status_t app_indication_cfm_handler(uint16_t conn_id, uint16_t handle)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Check if the indication confirmation is for the OTA FW upgrade service, if so, pass it to the library for processing
    if (handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }

    // If the provided handle does not match any known characteristics, return an invalid handle status
    return WICED_BT_GATT_INVALID_HANDLE;
}


/************************************************************************************************************************************
 * Function Name: app_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Connection up/down event. Define a function to handle GATT connection status events.
 *
 * Parameter
 * 	*p_status					: Pointer to structure with information about connection status
 *
 * Return
 * 	wiced_bt_gatt_status_t		: Return a GATT Status (codes).
 ***********************************************************************************************************************************/
wiced_bt_gatt_status_t app_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    wiced_result_t result;

    WICED_BT_TRACE("connection status event connected: %d\n", p_status->connected);

    // If the device is connected
    if (p_status->connected)
    {
    	// Save the connection ID and stop advertising
        app_conn_id = p_status->conn_id;
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    }
    else
    {
    	// If the device is disconnected
        app_conn_id = 0;
        WICED_BT_TRACE("disconnect reason: 0x%x \n", p_status->reason);

        // Start advertising again after disconnection
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        WICED_BT_TRACE("[%s] start adv status %d \n", __FUNCTION__, result);
    }

    // Pass connection up/down event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);

    // Return success status after handling the connection status event
    return WICED_BT_GATT_SUCCESS;
}


/************************************************************************************************************************************
 * Function Name: app_gatts_req_callback( wiced_bt_gatt_attribute_request_t *p_data )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Connection up/down event. Define a function to handle GATT connection status events.
 *
 * Parameter
 * 	*p_status					: Pointer to structure with information about connection status
 *
 * Return
 * 	wiced_bt_gatt_status_t		: Return a GATT Status (codes).
 ***********************************************************************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = app_gatts_req_read_handler(p_data->conn_id, &p_data->data.read_req);
        break;

    case GATTS_REQ_TYPE_WRITE:
    case GATTS_REQ_TYPE_PREP_WRITE:
        result = app_gatts_req_write_handler(p_data->conn_id, &p_data->data.write_req);
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_MTU:
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATTS_REQ_TYPE_CONF:
        result = app_indication_cfm_handler(p_data->conn_id, p_data->data.handle);
        break;

   default:
        WICED_BT_TRACE("%s: Unsupported type: %d\n", __func__, p_data->request_type);
        break;
    }
    return result;
}


/************************************************************************************************************************************
 * Function Name: wiced_bt_gatt_status_t app_gatt_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	This function is a callback used to handle various events in a Bluetooth application related to the Generic Attribute Profile
 *	(GATT). GATT is a specification that defines how to exchange data between devices in a Bluetooth network.
 *
 * Parameter
 * 	wiced_bt_gatt_evt_t event					: Enum with information about the GATT events.
 * 	wiced_bt_gatt_event_data_t *p_data			: Pointer to structure with information about the device connected.
 *
 * Return
 * 	wiced_bt_gatt_status_t						: Return a GATT Status (codes).
 ***********************************************************************************************************************************/
wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Initialize the result with an invalid GATT PDU status
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    // Switch statement to handle different GATT events
    switch (event)
    {
    // Event triggered when the GATT connection status changes
    case GATT_CONNECTION_STATUS_EVT:
        // Call a function to handle the connection status event and store the returned status
        result = app_connection_status_event(&p_data->connection_status);
        break;

    // Event triggered when a GATT attribute request is received
    case GATT_ATTRIBUTE_REQUEST_EVT:
        // Call a function to handle the GATT attribute request and store the returned status
        result = app_gatts_req_callback(&p_data->attribute_request);
        break;

    // Default case for any other GATT events that are not explicitly handled
    default:
        break;
    }

    // Return the result, which could be an updated status after handling the event
    return result;
}



/************************************************************************************************************************************
 * Function Name: app_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Define a function to read the link keys of a peer device from Non-Volatile RAM (NVRAM). The function returns true if the number
 *	of bytes read is equal to the size of the wiced_bt_device_link_keys_t structure, indicating that the read operation was
 *	successful and the link keys were retrieved from NVRAM. Otherwise, it returns false.
 *
 * Parameter
 * 	*p_keys					: Pointer to structure with link keys (about paired device).
 *
 * Return
 * 	wiced_bool_t			: Return a False or True
 ***********************************************************************************************************************************/
wiced_bool_t app_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    uint8_t          bytes_written;    	// Variable to store the number of bytes written
    wiced_result_t   result;           	// Variable to store the result of the write operation

#ifdef CYW20706A2
    extern uint32_t Config_VS_Location;	// External variable declaration for Config_VS_Location (only relevant for CYW20706A2)
#endif

    // Write the link keys data pointed by "p_keys" to the specified location in NVRAM
    // The number of bytes written and the result of the write operation are stored in "bytes_written" and "result", respectively
    bytes_written = wiced_hal_write_nvram(OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);

#ifdef CYW20706A2
    // If the code is compiled for CYW20706A2, print the information about the write operation
    WICED_BT_TRACE("Saved %d bytes at id:%d Config_VS_Location:%x\n", bytes_written, OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID, Config_VS_Location);
#else
    // If the code is not compiled for CYW20706A2, print the information about the write operation without Config_VS_Location
    WICED_BT_TRACE("Saved %d bytes at id:%d\n", bytes_written, OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID);
#endif

    // Return "WICED_TRUE" if the number of bytes written is equal to the size of "wiced_bt_device_link_keys_t", else return "WICED_FALSE"
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}


/************************************************************************************************************************************
 * Function Name: app_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	Define a function to read the link keys of a peer device from Non-Volatile RAM (NVRAM). The function returns true if the number
 *	of bytes read is equal to the size of the wiced_bt_device_link_keys_t structure, indicating that the read operation was successful
 *	and the link keys were retrieved from NVRAM. Otherwise, it returns false.
 *
 * Parameter
 * 	*p_keys					: Pointer to structure with link keys
 *
 * Return
 * 	wiced_bool_t			: Return a False or True
 ***********************************************************************************************************************************/
wiced_bool_t app_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    uint8_t         bytes_read;
    wiced_result_t  result;

    // Read the link keys of the peer device from NVRAM with the ID "OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID"
    bytes_read = wiced_hal_read_nvram(OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d\n", bytes_read, OTA_FW_UPGRADE_PEER_DEVICE_KEYS_VS_ID);

    // Return true if the number of bytes read is equal to the size of the wiced_bt_device_link_keys_t structure
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}


/************************************************************************************************************************************
 * Function Name: f_stack(uint32_t data)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *  This function is called to change the stack mode based on the value of the global variable OTA_Mode_Enabled. It toggles the value
 *  of OTA_Mode_Enabled between true (non-zero) and false (0) and writes the updated value to NVRAM (non-volatile memory). The
 *  function also prints a message to indicate the change in stack mode and can optionally reset the system using the watchdog timer,
 *  depending on the current value of OTA_Mode_Enabled.
 *
 * Parameter
 * 	uint32_t data					:
 ***********************************************************************************************************************************/
void handle_OTA_change_reset( uint32_t data )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Print a message "Change stack"
    WICED_BT_TRACE("Change stack\n");

    // Check the value of "OTA_Mode_Enabled"
    if (OTA_Mode_Enabled)
    {
        // If "OTA_Mode_Enabled" is true (non-zero), set it to false (0)
        OTA_Mode_Enabled = 0;
        // Write the updated value of "OTA_Mode_Enabled" to NVRAM (non-volatile memory)
        numbytes10 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 10, sizeof(OTA_Mode_Enabled), &OTA_Mode_Enabled, &status10);

        // Reset the system using the watchdog timer
        // Note: The line "wiced_hal_wdog_reset_system();" is commented out, so it won't execute in the current code.
    }
    /*else
    {
        // If "OTA_Mode_Enabled" is false (0), set it to true (non-zero)
        OTA_Mode_Enabled = 1;

        // Write the updated value of "OTA_Mode_Enabled" to NVRAM (non-volatile memory)
        numbytes10 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 10, sizeof(OTA_Mode_Enabled), &OTA_Mode_Enabled, &status10);

        // Reset the system using the watchdog timer
        // Note: The lines for resetting the system are commented out, so they won't execute in the current code.
    }*/
}


/************************************************************************************************************************************
 * Function Name: event_select_OTA(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 * 	This function is called to select OTA (Over-The-Air) mode. It sets the global variable OTA_Mode_Enabled to true (non-zero) and
 * 	writes the updated value to NVRAM (non-volatile memory) for persistence across power cycles. The function then resets the system
 * 	using the watchdog timer to activate OTA mode.
 ***********************************************************************************************************************************/
void event_select_OTA(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Set "OTA_Mode_Enabled" to true (non-zero)
    OTA_Mode_Enabled = 1;

    // Write the updated value of "OTA_Mode_Enabled" to NVRAM (non-volatile memory)
    numbytes10 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 10, sizeof(OTA_Mode_Enabled), &OTA_Mode_Enabled, &status10);

    // Reset the system using the watchdog timer will reset the system after setting "OTA_Mode_Enabled" to true
    wiced_hal_wdog_reset_system();
}
