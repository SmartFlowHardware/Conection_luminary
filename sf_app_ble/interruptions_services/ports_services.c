/************************************************************************************************************************************
 * File: ports_services.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *	 The functions enable interactions with physical buttons by detecting changes in GPIO pin input statuses. They respond to button
 *	 presses and execute specific actions based on press durations. The button_cback_0() function also incorporates a debouncing
 *	 mechanism to ensure reliable button press detection. This source code file plays a pivotal role in handling button-based user
 *	 interactions within a Bluetooth-enabled system.
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

#include "wiced_bt_dev.h"
#include "sparcommon.h"

#include "wiced_bt_mesh_models.h"

#include "wiced_platform.h"
#include "wiced_hal_aclk.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_mia.h"
#include "wiced_gki.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "ports_services.h"
#include "config_ports.h"

#define TIME_PUSH_BUTTON_MIN	3


/************************************************************************************************************************************
 * Function Name: button_control_init(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * The function is used to initialize and record the previous states of two buttons, and provides trace information about these
 * states. This type of function is commonly used at the start of a program or system to establish the initial state of components
 * that will be used later.
 ***********************************************************************************************************************************/
void button_control_init(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	button_previous_value_acuse = wiced_hal_gpio_get_pin_input_status( PORT_INT_ACUSE );
	button_previous_value_onoff = wiced_hal_gpio_get_pin_input_status( PORT_INT_ON_OFF );

	WICED_BT_TRACE("Button Previous Value Acuse:%d\r\n", button_previous_value_acuse);
	WICED_BT_TRACE("Button Previous Value OnOff:%d\r\n", button_previous_value_onoff);
}



void button_cback_26( void *data, uint8_t port_pin )
{
//	if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output(LED_GPIO_6 ) )
//	{
//		 //wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);
//		 WICED_BT_TRACE( "Interrup1 LOW\n\r" );
//
//	}
//	else
//	{
//		 //wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_HIGH);
//		 WICED_BT_TRACE( "Interrup1 HIGH\n\r" );
//	}
}

void button_cback_4( void *data, uint8_t port_pin )
{
	/*if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output(LED_GPIO_13 ) )
	{
		 //wiced_hal_gpio_set_pin_output( LED_GPIO_13, GPIO_PIN_OUTPUT_LOW);
		 WICED_BT_TRACE( "Interrup2 LOW\n\r" );

	}
	else
	{
		 //wiced_hal_gpio_set_pin_output( LED_GPIO_13, GPIO_PIN_OUTPUT_HIGH);
		 WICED_BT_TRACE( "Interrup2 HIGH\n\r" );
	}*/

	WICED_BT_TRACE( "Interrup2 HIGH\n\r" );

	init_event_gap();// evento confirmacion de abordamiento
}


/************************************************************************************************************************************
 * Function Name: void button_cback_1( void *data, uint8_t port_pin )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	This function going to detect if the user press the button in PORT 0 if the button is pressed the function going start an interrupt
 * 	timer. This function register if the user press the button, for a time to change the mode.
 *
 * 	Parameters:
 * 		*data:			Pointer to user data (optional)
 * 		port_pin:		Number of the pin that generated the interrupt
 ***********************************************************************************************************************************/
void button_cback_0( void *data, uint8_t port_pin )
{
	static uint16_t button_pushed_time = 0;
//	static uint32_t last_button_event_time_ms = 0;
//	uint32_t current_time_ms = app_timer_count * 1000; 	// Convert to milliseconds
//	uint32_t debounce_time_ms = 50; 					// Debounce time in milliseconds

	// Check if enough time has passed since the last event or the button was pressed one time
//	if ( (current_time_ms - last_button_event_time_ms) < debounce_time_ms  )
//	{
//		// Ignore button event due to debounce
//		return;
//	}

//	WICED_BT_TRACE( "Interrup0 HIGH\n\r" );
//	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
//	//WICED_BT_TRACE("%s, app timer : %d\n", __FUNCTION__, app_timer_count );
//
//	// Check if the button is pressed
//	if( wiced_hal_gpio_get_pin_input_status( PORT_INT_0 ) == GPIO_PIN_OUTPUT_LOW )
//	{
//		WICED_BT_TRACE("Button pressed\r\n");
//		button_pushed_time = app_timer_count;
//	}
//	// Check if the variable was changed
//	else if( button_pushed_time != 0 )
//	{
//		// Review if the user was pressed the button for more than TIME_PUSH_BUTTON_MIN seconds
//		if( ( app_timer_count - button_pushed_time ) > TIME_PUSH_BUTTON_MIN )
//		{
//			WICED_BT_TRACE("Button pressed more than %d seconds, RESET NVRAM\r\n", TIME_PUSH_BUTTON_MIN);
//			reset_values();
//		}
//		// If the user was pressed the button less than TIME_PUSHED_BUTTON_MIN
//		else
//		{
//			WICED_BT_TRACE("Button pressed less than %d seconds, SCAN ON\r\n", TIME_PUSH_BUTTON_MIN);
//		}
//
//		button_pushed_time = 0;		// Reset the value to know the time when was pressed the button
//		app_timer_count = 0;		// Reset the value of the counter
//	}
}


/************************************************************************************************************************************
 * Function Name: void button_cback_acuse( void *data, uint8_t port_pin )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	This function going to detect if the user press the button in PORT 0 if the button is pressed the function going start an interrupt
 * 	timer. This function register if the user press the button, for a time to change the mode.
 *
 * 	Parameters:
 * 		*data:			Pointer to user data (optional)
 * 		port_pin:		Number of the pin that generated the interrupt
 ***********************************************************************************************************************************/
void button_cback_acuse( void *data, uint8_t port_pin )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    uint32_t value_acuse = wiced_hal_gpio_get_pin_input_status(PORT_INT0);
    //uint32_t current_time_acuse = wiced_bt_mesh_core_get_tick_count();
    uint32_t button_pushed_duration_acuse;

//    if(value_acuse == button_previous_value_acuse)
//    {
//        WICED_BT_TRACE("interrupt_handler: duplicate pin:%d value:%d current_time:%d\n", port_pin, value_acuse, current_time_acuse);
//        return;
//    }

//    button_previous_value_acuse = value_acuse;

	// If the Mesh has been created or node is provisioned return the function
    if(is_provisioned)
    {
    	// Node provisioned
    	WICED_BT_TRACE("Device are provisioned\r\n");
    	return;
    }

    WICED_BT_TRACE("STATE OF BUTTON:%d\r\n", wiced_hal_gpio_get_pin_input_status(PORT_INT0));
    // Check if the button is pressed
    if(wiced_hal_gpio_get_pin_input_status(PORT_INT0) == GPIO_PIN_OUTPUT_HIGH)
    {
        WICED_BT_TRACE("interrupt_handler: button pressed pin:%d value:%d current_time:%d\n", port_pin, value_acuse);// current_time_acuse);

        // Create the Network
        create_network();

        //button_pushed_time = current_time_acuse;

        // if button is not released within 500ms, we will start sending move events
        //wiced_start_timer(&button_timer, 500);
        return;
    }
}


/************************************************************************************************************************************
 * Function Name: void button_cback_1( void *data, uint8_t port_pin )
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	This function going to detect if the user press the button in PORT 0 if the button is pressed the function going start an interrupt
 * 	timer. This function register if the user press the button, for a time to change the mode.
 *
 * 	Parameters:
 * 		*data:			Pointer to user data (optional)
 * 		port_pin:		Number of the pin that generated the interrupt
 ***********************************************************************************************************************************/
void button_cback_on_off( void *data, uint8_t port_pin )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Check if the button is pressed
//	if( wiced_hal_gpio_get_pin_input_status( PORT_INT_ON_OFF ) == GPIO_PIN_OUTPUT_LOW )
//	{
//		WICED_BT_TRACE("Button pressed\r\n");
//		//button_pushed_time = app_timer_count;
//
//		(wiced_hal_gpio_get_pin_input_status(LED_VEHICLE))?(wiced_hal_gpio_set_pin_output(LED_VEHICLE, GPIO_PIN_OUTPUT_LOW)):(wiced_hal_gpio_set_pin_output(LED_VEHICLE, GPIO_PIN_OUTPUT_HIGH));
//	}
}
