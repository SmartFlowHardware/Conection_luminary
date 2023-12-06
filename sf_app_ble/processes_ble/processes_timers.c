/************************************************************************************************************************************
 * File: processes_timer.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *	 These functions contribute to various aspects of the Bluetooth-enabled system's operation, such as scanning, timing of specific
 *	 tasks, handling flags, and executing different processes based on specific conditions. The source code file reflects the
 *	 integration of these functions into the overall system behavior.
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

#include "wiced_hal_wdog.h"
#include "wiced_hal_rand.h"
#include "wiced_hal_nvram.h"

#include "wiced_bt_trace.h"
//#include "wiced_bt_cfg.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "wiced_bt_beacon.h"
#include "string.h"
#include "sparcommon.h"
#include "wiced_rtos.h"
//#include "GeneratedSource/cycfg_gatt_db.h"
#ifndef CYW43012C0
#include "wiced_bt_ota_firmware_upgrade.h"
#endif
#include "wiced_hal_puart.h"
#include "wiced_platform.h"
#include "wiced_transport.h"

#include <malloc.h>

#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_core.h"
#include "processes_timers.h"
#include "config_ports.h"



/************************************************************************************************************************************
 * Function Name: f_app_main( TIMER_PARAM_TYPE arg )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function is responsible for initiating a High-Duty BLE scan if a scan id not currently being performed and other things as a
 * 	the sensor has not been found or less sensor to found- The scan result and related operations are logged to the trace console using
 * 	WICED_BT_TRACE.
 *
 *
 * Parameters:
 * 	void
 *
 * Return:
 *  void
 ***********************************************************************************************************************************/
void f_app_main( TIMER_PARAM_TYPE arg )
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

//    wiced_result_t               status;
//
//    if( ( wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE ) && ( found_sensor != 3 ) && ( device_scanned == WICED_FALSE ) )
//    {
//        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, scanCallback );
//        WICED_BT_TRACE( "wiced_bt_ble_scan: %d\n", status );
//    }
//    UNUSED_VARIABLE(status);
//
//    app_timer_count++;
}


/************************************************************************************************************************************
 * Function Name: f_timer_Online
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function runs continuously with once
 * 	every 800 milliseconds, executed the task
 *
 * Parameters:
 * 	void
 *
 * Return:
 *  void
 ***********************************************************************************************************************************/
void f_timer_Online( uint32_t data )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	//WICED_BT_TRACE( "BLE Online\n" );
	//start_TOnline();
    //wiced_start_timer( &timer_Online, clock_Online);
	if(ctr_p6 == WICED_TRUE)
	{
		ctr_p6 = WICED_FALSE;
	    //wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);
	}
}



void f_timer_sm( uint32_t data )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
	//start_TOnline();
	start_TOsm();
	ctr_p6 = WICED_TRUE;
    //wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_HIGH);
}

void f_timer_st_Online( uint32_t data )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

//	if(status_Online)
//	{
//		WICED_BT_TRACE( "GUIF: Online\n" );
//		status_Online = WICED_FALSE;
//		start_TOnline_long();
//		//wiced_start_timer( &timer_st_Online, clock_st_Online);
//	}
//	else
//	{
//		if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_input_status( PORT_INT_25 ) )
//		{
//			//wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW );
//			WICED_BT_TRACE( "PIN 26 HIGTH\n" );
//		}
//		else
//		{
//		   WICED_BT_TRACE( "PIN HIGTH\n" );
//		   //wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_LOW);
//		   //wiced_rtos_delay_milliseconds( 1000, ALLOW_THREAD_TO_SLEEP );
//		   //wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_HIGH);
//		   WICED_BT_TRACE( "PIN LOW\n" );
//
//		   //wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH );
//		}
//
//		   //WICED_BT_TRACE( "PIN HIGTH\n" );
//		   /*wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);
//		   wiced_rtos_delay_milliseconds( 1000, ALLOW_THREAD_TO_SLEEP );
//		   wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_HIGH);*/
//		   //WICED_BT_TRACE( "PIN LOW\n" );
//		   status_Online = WICED_FALSE;
//		   start_TOnline_long();
//		   //wiced_start_timer( &timer_st_Online, clock_st_Online);
//   }

}


void f_timer_inspection( uint32_t data )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	extern void gap_out_f(void);
	WICED_BT_TRACE("Enter inspection\n");
	value_inspection =  !value_inspection ? WICED_TRUE : WICED_FALSE;
	WICED_BT_TRACE("Value inspection: %d\n", value_inspection);
    prevention_status2();
    gap_out_f();
}

void f_timer_return( uint32_t data )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	WICED_BT_TRACE("Enter return\n");
	//wiced_hal_wdog_reset_system ();
	beacon_set_app_advertisement_data4();
    //wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);
    value_gap = WICED_TRUE;

}


void f_timer_rOTA( uint32_t data )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
	process_rOTA();
}

void f_timer_SPI( void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
	stop_rbdkst();
}

void f_timer_da( void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
	value_da= WICED_FALSE;
	WICED_BT_TRACE("Clear da\n");
}

void f_timer_clrspi( void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	value_sdb = WICED_FALSE;
	WICED_BT_TRACE("Clear clrspi\n");
	stop_rbdkst();
}


/************************************************************************************************************************************
 * Function Name: void f_timer_cback1( void )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	This function going to generate an interrupt with a timer when the
 * 	user press the button this is for avoid the mechanical noise in the
 * 	button. When the button is pressed and if there are beacons nearby
 * 	the LED appropriate at beacon going be ON and stand. The timer start
 * 	in button_cback_1().
 *
 * 	Parameters:		void
 * 	Return:			void
 ***********************************************************************************************************************************/
void f_timer_cback1( void ){
	WICED_BT_TRACE("timer callback 1\n");

	// Change the value of the variable state_button
//	(state_button > 0)? (state_button = 0) : (state_button++);
//
//	// Check if the count is started
//	if( count_led_on == 0  && state_button == 1 )
//	{
//		// Keep a LED on while transmit data
//		for( int i = 1; i <= MAX_NUM_OF_BEACONS; i++ )
//		{
//			if( array_beacons_observed[POSITION_INITIAL] == i || array_beacons_observed[POSITION_INITIAL] == EXTRA_BEACON )
//			{
//				// If Beacon 1 has the configuration activated
//				if( array_beacons_observed[POSITION_INITIAL] == EXTRA_BEACON )
//				{
//					array_led_on[BEACON1] = 1;
//					wiced_hal_gpio_set_pin_output( LED_BLUE, GPIO_PIN_OUTPUT_HIGH );
//
//					// Start broadcast
//					gap_rebroadcastLR( 1 );
//					break;
//				}
//				else
//				{
//					array_led_on[--i] = 1;
//					wiced_hal_gpio_set_pin_output( LED_BLUE, GPIO_PIN_OUTPUT_HIGH );
//
//					// Start broadcast
//					gap_rebroadcastLR( 1 );
//					break;
//				}
//			}
//		}
//	}
//	else
//	{
//		// Turn Off the transmit data
//		for( int i = 0; i < MAX_NUM_OF_BEACONS; i++ )
//		{
//			if( array_led_on[i] == 1 )
//			{
//				// Turn Off the LED
//				array_led_on[i] = 0;
//				break;
//			}
//		}
//
//		// Stop broadcast
//	}
}



/************************************************************************************************************************************
 * Function Name: f_app_main( TIMER_PARAM_TYPE arg )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function is responsible for initiating and checkout some processes for the correct handle of the variables that indicate if the
 * 	Lamp or Tag are saw.
 *
 *
 * Parameters:
 * 	TIMER_PARAM_TYPE arg	:
 *
 * Return:
 *  void
 ***********************************************************************************************************************************/
void f_timer_lamp( TIMER_PARAM_TYPE arg )
{
	find_lamp = WICED_FALSE;
	wiced_hal_gpio_set_pin_output(LED_PERSON, GPIO_PIN_OUTPUT_LOW);
	WICED_BT_TRACE("Clear Lamp: %d\n", find_lamp);
}



/************************************************************************************************************************************
 * Function Name: f_app_main( TIMER_PARAM_TYPE arg )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function is responsible for initiating and checkout some processes for the correct handle of the variables that indicate if the
 * 	Lamp or Tag are saw.
 *
 *
 * Parameters:
 * 	TIMER_PARAM_TYPE arg	:
 *
 * Return:
 *  void
 ***********************************************************************************************************************************/
void f_timer_tag( TIMER_PARAM_TYPE arg )
{
	find_tag = WICED_FALSE;
	wiced_hal_gpio_set_pin_output(LED_VEHICLE, GPIO_PIN_OUTPUT_LOW);
	WICED_BT_TRACE("Clear Tag: %d\n", find_tag);
}


/************************************************************************************************************************************
 * Function Name: f_app_main( TIMER_PARAM_TYPE arg )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function is responsible for initiating and checkout some processes for the correct handle of the variables that indicate if the
 * 	Lamp or Tag are saw.
 *
 *
 * Parameters:
 * 	TIMER_PARAM_TYPE arg	:
 *
 * Return:
 *  void
 ***********************************************************************************************************************************/
void f_timer_node( TIMER_PARAM_TYPE arg )
{
	find_node = WICED_FALSE;
	blinking_led_timer = WICED_FALSE;

	// Stop the timer that blinking the LED and keep the LED On
	wiced_stop_timer(&bled_timer);
	wiced_hal_gpio_set_pin_output(LED_CHARGE, GPIO_PIN_OUTPUT_LOW);

	WICED_BT_TRACE("Clear Node: %d\n", find_node);
}


/************************************************************************************************************************************
 * Function Name: f_app_main( TIMER_PARAM_TYPE arg )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function is responsible for initiating and checkout some processes for the correct handle of the variables that indicate if the
 * 	Lamp or Tag are saw.
 *
 *
 * Parameters:
 * 	TIMER_PARAM_TYPE arg	:
 *
 * Return:
 *  void
 ***********************************************************************************************************************************/
void f_timer_bled( TIMER_PARAM_TYPE arg )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Variables to set the values
	static uint8_t		counter_timer_led;

	// Check if counter is minor at limit
	if( counter_timer_led <= TIMES_BLINKING_LED )
	{
		// Blinking the Led and increment the value of the counter
		wiced_hal_gpio_set_pin_output(LED_CHARGE, !wiced_hal_gpio_get_pin_output(LED_CHARGE));
		counter_timer_led++;
	}
	else if( counter_timer_led > TIMES_BLINKING_LED )
	{
		// Kepp the LED On
		wiced_hal_gpio_set_pin_output(LED_CHARGE, GPIO_PIN_OUTPUT_LOW);
		counter_timer_led++;

		// When pass one second reset the value to going blinking the LED
		if(counter_timer_led == 8)
			counter_timer_led = 0;
	}
}
