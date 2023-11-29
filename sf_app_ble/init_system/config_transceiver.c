/************************************************************************************************************************************
 * File: config_transceiver.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *	In essence, the config_Transceiver() function prepares the PUART for communication by initializing it, setting up necessary
 *	configurations, enabling transmission and reception, and configuring interrupts for data reception. This function is typically
 *	used during the setup process for communication over the PUART in a microcontroller or embedded system.
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

#include "wiced_bt_trace.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_stack.h"
#include "wiced_hal_puart.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "config_transceiver.h"



/************************************************************************************************************************************
 *  												Function Definitions
 ***********************************************************************************************************************************/

/************************************************************************************************************************************
 * Function Name: config_Transceiver(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The code defines a function that configures the Peripheral UART (PUART) for communication. It initializes the PUART, turns off
 * 	flow control, enables transmission and reception, and sets up an interrupt callback for PUART reception. The PUART will generate
 * 	an interrupt when at least one byte of data is received. This function is likely part of the setup process for using the PUART in
 * 	a microcontroller or embedded system.
 ***********************************************************************************************************************************/
void config_Transceiver(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Initialize the Peripheral UART (PUART) for communication.
    wiced_hal_puart_init();

    // Turn off flow control on the PUART.
    wiced_hal_puart_flow_off();

    // Enable transmission on the PUART.
    wiced_hal_puart_enable_tx();

    // Register the interrupt callback function 'rx_cback' for PUART reception.
    wiced_hal_puart_register_interrupt( rx_cback );

    // Set the watermark level of PUART to 1 byte. This means an interrupt will be generated when at least one byte is received.
    wiced_hal_puart_set_watermark_level( 1 );

    // Enable reception on the PUART.
    wiced_hal_puart_enable_rx();
}
