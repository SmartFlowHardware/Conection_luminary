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
#include "mesh_definitions.h"
#include "ports_services.h"
#include "config_ports.h"

#define TIME_PUSH_BUTTON_MIN	3


/************************************************************************************************************************************
 *  												Function Definitions
 ***********************************************************************************************************************************/

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

	button_previous_value_acuse = wiced_hal_gpio_get_pin_input_status(PORT_INT_ACUSE);
	button_previous_value_onoff = wiced_hal_gpio_get_pin_input_status(PORT_INT_ON_OFF);

	WICED_BT_TRACE("Button Previous Value Acuse:%d\r\n", button_previous_value_acuse);
	WICED_BT_TRACE("Button Previous Value OnOff:%d\r\n", button_previous_value_onoff);
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
//	static uint16_t button_pushed_time = 0;
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
	uint32_t current_time_acuse = app_timer_count * 500; 	// Convert to milliseconds

	// Check if enough time has passed since the last event or the button was pressed one time
	if ( ( (current_time_acuse - lst_btn_evt_time_acuse) < debounce_time_ms ) )
	{
		// Ignore button event due to debounce or to multi press
		return;
	}

	WICED_BT_TRACE("[%s] app timer count:%d\n", __FUNCTION__, app_timer_count );

	// Check if button is pressed
	if( wiced_hal_gpio_get_pin_input_status( PORT_INT_ACUSE ) == GPIO_PIN_OUTPUT_HIGH )
	{
        btn_push_tm_acuse = app_timer_count;
        WICED_BT_TRACE( "Button ACUSE pressed | btn_push_tm_acuse: %d\r\n", btn_push_tm_acuse );
	}
	// Check if the button was released
	else if ( btn_push_tm_acuse != 0 )
	{
		WICED_BT_TRACE( "btn_push_tm_acuse:%d | app_timer_count:%d\r\n", btn_push_tm_acuse, app_timer_count );

    	// Check that time was pressed the button ( if is higher TIME_PUSH_BUTTON_MIN ) change configuration
    	if( (app_timer_count - btn_push_tm_acuse) > TIME_PUSH_BUTTON_MIN )
    	{
    		WICED_BT_TRACE( "Button released After more than 2s, Create Network\r\n" );

    		// If the Mesh has been created or node is provisioned return the function
    		if(is_provisioned)
    		{
    			// Node provisioned
    			WICED_BT_TRACE("Device are provisioned\r\n");
    			//return;
    		}
    		else
    		{
    			// Create the Network
    			create_network();
    			//return;
    		}

    	}
    	else if( ( ( app_timer_count - btn_push_tm_acuse ) <= TIME_PUSH_BUTTON_MIN ) )  /* Contestacion de 1 segundo donde vamos a reponder le conexion*/
    	{
            WICED_BT_TRACE( "Button released Before less than 2s, Change Advertisement\r\n");

            // Blinking the Led and increment the value of the counter
            wiced_hal_gpio_set_pin_output(LED_PERSON, !wiced_hal_gpio_get_pin_output(LED_PERSON));

            if((node.addr == 1) && find_node) /* Central */
            {
            	WICED_BT_TRACE("Advertisement Connect Message Active\r\n");

            	// Start to transmit the "connect" message
            	mode_send_info = WICED_TRUE;
            	gap_rebroadcastLR(BEACON_INFO_MESH_UID_ADV,0);
            }

    	}  //true in observe
    	if(   conn_node_mesh    == WICED_TRUE && is_provisioned == WICED_FALSE)  /* Respuesta de conexion ****************/
    	{
    		//gap_rebroadcastLR(NODE_ADV,info_mesh.addr); /* Send response */
    																    /* Response UID */
    		beacon_set_eddystone_uid_advertisement_data_1(info_mesh.addr,        1         );  //*****************************************-----------------
    	}
    	btn_push_tm_acuse = 0; 						// Reset the time of button after process the event
    	lst_btn_evt_time_acuse = current_time_acuse;
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
	uint32_t current_time_onoff = app_timer_count * 500; 	// Convert to milliseconds


	// Check if enough time has passed since the last event or the button was pressed one time
	if ( ( (current_time_onoff - lst_btn_evt_time_onoff) < debounce_time_ms ) )
	{
		// Ignore button event due to debounce or to multi press
		return;
	}

	WICED_BT_TRACE("[%s] app timer count:%d\n", __FUNCTION__, app_timer_count );

	// Check if button is pressed
	if( wiced_hal_gpio_get_pin_input_status( PORT_INT_ON_OFF ) == GPIO_PIN_OUTPUT_HIGH )
	{
        btn_push_tm_onoff = app_timer_count;
        WICED_BT_TRACE( "Button ON/OFF pressed | btn_push_tm_onoff: %d\r\n", btn_push_tm_onoff );
	}
	// Check if the button was released
	else if ( btn_push_tm_onoff != 0 )
	{
		WICED_BT_TRACE( "btn_push_tm_onoff:%d | app_timer_count:%d\r\n", btn_push_tm_onoff, app_timer_count );

    	// Check that time was pressed the button ( if is higher TIME_PUSH_BUTTON_MIN ) change configuration
    	if( (app_timer_count - btn_push_tm_onoff) > TIME_PUSH_BUTTON_MIN )
    	{
    		WICED_BT_TRACE( "Button released After more than 2s, Delete Network\r\n" );

    		// If the Mesh has been created or node is provisioned return the function
    		if(is_provisioned)
    		{
    			WICED_BT_TRACE("Delete Network\r\n");

    			// Delete information about the Mesh
    			mesh_app_factory_reset();
    			//return;
    		}
    		else
    		{
    			// Nothing to delete
    			WICED_BT_TRACE("There is not a Network\r\n");
    			//return;
    		}

    	}
    	else if( ( ( app_timer_count - btn_push_tm_onoff ) <= TIME_PUSH_BUTTON_MIN ) )
    	{
    		// Turn On the LED

            WICED_BT_TRACE( "Button released Before less than 2s, Start Timer\r\n");

            // Blinking the Led and increment the value of the counter
            wiced_hal_gpio_set_pin_output(LED_VEHICLE, !wiced_hal_gpio_get_pin_output(LED_VEHICLE));
    	}

    	btn_push_tm_onoff = 0; 						// Reset the time of button after process the event
    	lst_btn_evt_time_onoff = current_time_onoff;
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
void blinking_led( uint8_t pin_led )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Variables to set the values
	static uint8_t		counter_timer_led;

	// Check if counter is minor at limit
	if( counter_timer_led <= TIMES_BLINKING_LED )
	{
		// Blinking the Led and increment the value of the counter
		wiced_hal_gpio_set_pin_output(pin_led, !wiced_hal_gpio_get_pin_output(pin_led));
		counter_timer_led++;
	}
	else if( counter_timer_led > TIMES_BLINKING_LED )
	{
		// Kepp the LED On
		wiced_hal_gpio_set_pin_output(pin_led, GPIO_PIN_OUTPUT_LOW);
		counter_timer_led++;

		// When pass one second reset the value to going blinking the LED
		if(counter_timer_led == 8)
			counter_timer_led = 0;
	}
}


// Check if the button is pressed
//	if( wiced_hal_gpio_get_pin_input_status( PORT_INT_ON_OFF ) == GPIO_PIN_OUTPUT_LOW )
//	{
//		WICED_BT_TRACE("Button pressed\r\n");
//		//button_pushed_time = app_timer_count;
//
//	}


//    //uint32_t value_acuse = wiced_hal_gpio_get_pin_input_status(PORT_INT_ACUSE);
//	static uint32_t last_button_event_time_ms = 0;
//    uint32_t current_time_acuse = app_timer_count * 500; 	// Convert to milliseconds
//    uint32_t debounce_time_acuse = 100;						// Debounce time in milliseconds
//    uint16_t button_pushed_acuse;
//
//	// Check if enough time has passed since the last event or the button was pressed one time
//	if ( (current_time_acuse - last_button_event_time_ms) < debounce_time_acuse  )
//	{
//		// Ignore button event due to debounce
//		return;
//	}
//
//    WICED_BT_TRACE("STATE OF BUTTON:%d\r\n", wiced_hal_gpio_get_pin_input_status(PORT_INT_ACUSE));
//
//    // Check if the button is pressed
//    if(wiced_hal_gpio_get_pin_input_status(PORT_INT_ACUSE) == GPIO_PIN_OUTPUT_HIGH)
//    {
//    	// Save the "time" when the button is pushed and start to counter
//    	button_pushed_acuse = app_timer_button;
//    	acuse_pressed = WICED_TRUE;
//    }
//    // Check when the button is released
//    else if( (app_timer_button - button_pushed_acuse) >= 6 ) // Hold pushed for 3 Seconds
//    {
//    	acuse_pressed = WICED_FALSE;
//
//    	// If the Mesh has been created or node is provisioned return the function
//        if(is_provisioned)
//        {
//        	// Node provisioned
//        	WICED_BT_TRACE("Device are provisioned\r\n");
//        	return;
//        }
//        else
//        {
//            // Create the Network
//            create_network();
//            return;
//        }
//    }
//    else // Not hold pushed button
//    {
//    	acuse_pressed = WICED_FALSE;
//
//		// Blinking the Led and increment the value of the counter
//		wiced_hal_gpio_set_pin_output(LED_VEHICLE, !wiced_hal_gpio_get_pin_output(LED_VEHICLE));
//    }
//
//    button_pushed_acuse = 0;
//    last_button_event_time_ms = current_time_acuse;
