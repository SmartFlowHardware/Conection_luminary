/************************************************************************************************************************************
 * File: config_timers.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *	This source code file manages and configures various timers crucial for maintaining the functionality and interactions of a
 *	Bluetooth application. These timers facilitate tasks such as managing online status, state transitions, inspections, data handling,
 *	and communication operations.
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
#include "config_timers.h"


/************************************************************************************************************************************
 *  												Function Definitions
 ***********************************************************************************************************************************/

/************************************************************************************************************************************
 * Function Name: config_clk_timers(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Configures various timers for the application.
 *
 * 	This function initializes several timers using the WICED SDK's timer API.
 * 	Timers are configured for periodic and millisecond-based intervals to handle different tasks in the application.
 ***********************************************************************************************************************************/
void config_clk_timers(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_init_timer( &app_main_timer, f_app_main, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
//	wiced_init_timer( &timer_Online, f_timer_Online, 0, WICED_MILLI_SECONDS_TIMER );
//    wiced_init_timer( &timer_st_Online, f_timer_st_Online, 0, WICED_MILLI_SECONDS_TIMER );
//    wiced_init_timer( &timer_inspection, f_timer_inspection, 0, WICED_SECONDS_TIMER);
//    wiced_init_timer( &timer_return, f_timer_return, 0, WICED_SECONDS_TIMER);
//    wiced_init_timer( &timer_sm, f_timer_sm, 0, WICED_MILLI_SECONDS_TIMER );
//    wiced_init_timer( &timer_rOTA, f_timer_rOTA, 0, WICED_MILLI_SECONDS_TIMER );
//    wiced_init_timer( &timer_SPI, f_timer_SPI, 0, WICED_SECONDS_TIMER );
//    wiced_init_timer( &timer_da, f_timer_da, 0, WICED_SECONDS_TIMER );
//    wiced_init_timer( &timer_clrspi, f_timer_clrspi, 0, WICED_MILLI_SECONDS_TIMER );

    wiced_init_timer( &timer_cback1, f_timer_cback1, 0, WICED_MILLI_SECONDS_TIMER );

    // Pulse Timers
	wiced_init_timer( &lamp_timer, f_timer_lamp, 0, WICED_SECONDS_TIMER);
	wiced_init_timer( &tag_timer, f_timer_tag, 0, WICED_SECONDS_TIMER);
	wiced_init_timer( &node_timer, f_timer_node, 0, WICED_SECONDS_TIMER);
	wiced_init_timer( &bled_timer, f_timer_bled, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
}


/************************************************************************************************************************************
 * Function Name: start_BTimers(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timers with predefined intervals. This function starts two timers using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_BTimers(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_start_timer(&app_main_timer, APP_TIMEOUT_MILISECONDS);
//    wiced_start_timer( &timer_Online, 1000);
//    wiced_start_timer( &timer_st_Online, clock_st_Online);
}


/************************************************************************************************************************************
 * Function Name: start_TOnline(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timers with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_TOnline(void)
{
	 wiced_start_timer( &timer_Online, clock_Online);
}


/************************************************************************************************************************************
 * Function Name: start_TOnline_long(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timers with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_TOnline_long(void)
{
	wiced_start_timer( &timer_st_Online, clock_st_Online);
}


/************************************************************************************************************************************
 * Function Name: stop_TOnline(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Stop a specific timer with predefined. This function stops a timer and starts other using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void stop_TOnline(void)
{
	wiced_stop_timer(&timer_Online);
	wiced_start_timer( &timer_Online, clock_Online_long);
}


/************************************************************************************************************************************
 * Function Name: prevention_inspection(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void prevention_inspection(void)
{
	  /* if (wiced_init_timer(&timer_inspection, f_timer_inspection, 0, WICED_SECONDS_TIMER) == WICED_SUCCESS)
	    {
		   WICED_BT_TRACE("Init timer inspection\n");
		   wiced_start_timer( &timer_inspection, clock_inspection);
	    }*/
	   WICED_BT_TRACE("Init timer inspection\n");
	   wiced_start_timer( &timer_inspection, clock_inspection);
}


/************************************************************************************************************************************
 * Function Name: stop_timer_st_online(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Stop a specific timer with predefined. This function stops a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void stop_timer_st_online(void)
{
	wiced_stop_timer( &timer_st_Online);
}


/************************************************************************************************************************************
 * Function Name: start_Treturn(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_Treturn(void)
{
	wiced_start_timer( &timer_return, clock_return5);
}


/************************************************************************************************************************************
 * Function Name: start_Tsm(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_Tsm(void)
{
	wiced_start_timer( &timer_sm, 600);
}


/************************************************************************************************************************************
 * Function Name: start_TOsm(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_TOsm(void)
{
	wiced_start_timer( &timer_Online, 500);
}


/************************************************************************************************************************************
 * Function Name: start_trOTA(uint32_t t_clk)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_trOTA(uint32_t t_clk)
{
	wiced_start_timer( &timer_rOTA, t_clk);
}


/************************************************************************************************************************************
 * Function Name: start_trOTA(uint32_t t_clk)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Stops specific timer with predefined intervals. This function stops a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_trOTA_stop(void)
{
	wiced_stop_timer( &timer_rOTA);
}


/************************************************************************************************************************************
 * Function Name: start_TSPI(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_TSPI(void)
{
	wiced_start_timer( &timer_SPI, 3600);
}


/************************************************************************************************************************************
 * Function Name: clear_da(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void clear_da(void)
{
	wiced_start_timer( &timer_da, 10);
}


/************************************************************************************************************************************
 * Function Name: clr_spi(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Stops specific timer with predefined intervals. This function stops a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void clr_spi(void)
{
	wiced_start_timer( &timer_clrspi, 2800);
}


/************************************************************************************************************************************
 * Function Name: start_cbck1(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Starts specific timer with predefined intervals. This function starts a timer using the WICED SDK's timer API.
 ***********************************************************************************************************************************/
void start_cbck1(void)
{
	wiced_start_timer( &timer_cback1, 250 );
}


/************************************************************************************************************************************
 * Function Name: (void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The "start_TOnline(void)" function is designed to start the select timers.
 ***********************************************************************************************************************************/
//void start_TOnline(void)
//{
//	wiced_start_timer(&app_main_timer, APP_TIMEOUT_IN_SECONDS);
//}



/************************************************************************************************************************************
 * Function Name: (void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The "start_TOnline(void)" function is designed to start the select timers.
 ***********************************************************************************************************************************/
void start_lamp_timer(void)
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_start_timer(&lamp_timer, TIMEOUT_IN_SECONDS);
}



/************************************************************************************************************************************
 * Function Name: (void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The "start_TOnline(void)" function is designed to start the select timers.
 ***********************************************************************************************************************************/
void start_tag_timer(void)
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_start_timer(&tag_timer, TIMEOUT_IN_SECONDS);
}


/************************************************************************************************************************************
 * Function Name: (void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The "start_TOnline(void)" function is designed to start the select timers.
 ***********************************************************************************************************************************/
void start_node_timer(void)
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_start_timer(&node_timer, TIMEOUT_IN_SECONDS);
}


/************************************************************************************************************************************
 * Function Name: (void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The "start_TOnline(void)" function is designed to start the select timers.
 ***********************************************************************************************************************************/
void start_bled_timer(void)
{
	//WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_start_timer(&bled_timer, 500);
}


/************************************************************************************************************************************
 * Function Name: stop_timers(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The "stop_timers(void)" function is designed to stop the select timers.
 ***********************************************************************************************************************************/
void stop_timers(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Stop all timers to let configure Node like Embedded Provisioner
	wiced_stop_timer(&app_main_timer);
	wiced_stop_timer(&lamp_timer);
	wiced_stop_timer(&tag_timer);
	wiced_stop_timer(&node_timer);
}
