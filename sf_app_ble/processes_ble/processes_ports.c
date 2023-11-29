/*
 * processes_ports.c
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */


#include "processes_ports.h"
#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_rtos.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_mia.h"
#include "wiced_gki.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "config_ports.h"

extern void start_trOTA(uint32_t t_clk);
extern void start_trOTA_stop(void);
extern void event_select_OTA(void);
int crOTA = 1;

void event_recover_OTA(void)
{
	/*if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output(LED_GPIO_6 ) )
	{
		 WICED_BT_TRACE( "PIN6 LOW\n\r" );
		 return;
	}
	else if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output(LED_GPIO_13 ) )
	{
		 WICED_BT_TRACE( "PIN113 LOW\n\r" );
		 return;
	}*/

//	 wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_HIGH);
//	 WICED_BT_TRACE( "PIN16 LOW\n\r" );
//	 wiced_hal_gpio_set_pin_output( LED_GPIO_13, GPIO_PIN_OUTPUT_LOW);
//     WICED_BT_TRACE( "PIN13 LOW\n\r" );
//     //wiced_rtos_delay_milliseconds( 1000, ALLOW_THREAD_TO_SLEEP );
//	 wiced_hal_gpio_set_pin_output( LED_GPIO_13, GPIO_PIN_OUTPUT_HIGH);
//     WICED_BT_TRACE( "PIN13 HIGH\n\r" );
//     wiced_rtos_delay_milliseconds( 5000, ALLOW_THREAD_TO_SLEEP );
//	 wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_LOW);
//     WICED_BT_TRACE( "PIN16 HIGH\n\r" );
//     wiced_rtos_delay_milliseconds( 500, ALLOW_THREAD_TO_SLEEP );
//	 wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_HIGH);
//     WICED_BT_TRACE( "PIN16 LOW\n\r" );
//     wiced_rtos_delay_milliseconds( 10000, ALLOW_THREAD_TO_SLEEP );
//     wiced_hal_gpio_set_pin_output( LED_GPIO_13, GPIO_PIN_OUTPUT_LOW);
//     WICED_BT_TRACE( "PIN13 LOW\n\r" );
//     WICED_BT_TRACE( "OTA END\n\r" );
//     wiced_rtos_delay_milliseconds( 8000, ALLOW_THREAD_TO_SLEEP );
//     WICED_BT_TRACE( "RESET\n\r" );
}

void process_rOTA(void)
{

	switch (crOTA)
	{
    case 1:

//		 wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_LOW);
//		 WICED_BT_TRACE( "PIN16 LOW\n\r" );
//		 wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);	//LED_GPIO_7
//		 WICED_BT_TRACE( "PIN7 LOW\n\r" );
//		 wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_HIGH);	//LED_GPIO_7
		 WICED_BT_TRACE( "PIN7 HIGH\n\r" );
    	 start_trOTA(5000);
    	 crOTA++;
    break;

    case 2:

		 //wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_HIGH);
		 WICED_BT_TRACE( "PIN16 HIGH\n\r" );
    	 start_trOTA(500);
    	 crOTA++;
    break;

    case 3:
		 //wiced_hal_gpio_set_pin_output( LED_GPIO_16, GPIO_PIN_OUTPUT_LOW);
		 WICED_BT_TRACE( "PIN16 LOW\n\r" );
    	 start_trOTA(10000);
    	 crOTA++;
    break;

    case 4:

    	//wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);	//LED_GPIO_7
        WICED_BT_TRACE( "PIN7 LOW\n\r" );

    	start_trOTA(8000);
    	crOTA++;
    break;

    case 5:
    	start_trOTA_stop();
    	crOTA=1;

        WICED_BT_TRACE( "PIN7 HIGH\n\r" );
    	WICED_BT_TRACE( "OTA END\n\r" );
    	//event_select_OTA();
    break;

	}
}

void prevention_status(void)
{
	//wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);
}

void prevention_status2(void)
{
//	if(GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output(LED_GPIO_6 ))
//	{
//	wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_LOW);
//	}
//	else
//	{
//    wiced_hal_gpio_set_pin_output( LED_GPIO_6, GPIO_PIN_OUTPUT_HIGH);
//	}
}

