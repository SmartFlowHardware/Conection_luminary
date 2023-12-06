/************************************************************************************************************************************
 * File: processes_Rx_trans.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *   This file contains the functions that are controlled by the Input UART, depending on certain types of commands specific
 *   functions are activated, Each function, upon completion, saves variables, Bluetooth Address and some flags of reference, By
 *   resetting the system with these variables and starting refresh, this going to take another way in the program.
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
#include "processes_Rx_trans.h"
#include "wiced_hal_wdog.h"
#include "wiced_hal_rand.h"
#include "wiced_hal_nvram.h"
//#include "processes_ports.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_memory.h"
#include "wiced_bt_cfg.h"

/***********************************************************************************************************************************
 * 													Function Definitions
 ***********************************************************************************************************************************/

/************************************************************************************************************************************
 * Function Name: app_hci_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The process_Write function takes a pointer to uint8_t data and processes it based on the values it contains. It performs
 *	different operations depending on whether the data matches predefined patterns (data_WBN, data_WBF, data_WMA, data_WiFi). If the
 *	data matches a specific pattern, it writes data to NVRAM and prints information to the debug trace using UART. Some sections of
 *	the code are commented out and do not affect the current implementation.
 *
 * Parameters:
 * 	uint8_t *data_Write						: Pointer to data.
 ***********************************************************************************************************************************/
void process_Write(uint8_t *data_Write)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Copy the first 3 bytes of data_Write into the data_f array
	memcpy(data_f, data_Write, 3);

    // Check if data_f matches data_WBN, put the device in OTA if receives "WBN"
    if(memcmp( DATA_WBN, data_f, sizeof(data_f) ) == 0)
    {
        // If they match, copy the next 16 bytes of data_Write into the data_uuid array
        memcpy(data_uuid, &data_Write[3], 16);

        // Copy data_uuid into the data_bn_save array
        memcpy(data_bn_save, data_uuid, 16);

        // Write data_bn_save to non-volatile memory (NVRAM)
        numbytes1 = wiced_hal_write_nvram( WICED_NVRAM_VSID_START, sizeof(data_bn_save), &data_bn_save, &status1 );

        // Print "Beacon Name Saved:" to the debug trace
        WICED_BT_TRACE("Beacon Name Saved: ");

        // Print each byte of data_bn_save to the debug trace
        for (int i = 0; i < sizeof(data_bn_save) - 1; i++)
        {
            wiced_hal_puart_write(data_bn_save[i]);
        }

        WICED_BT_TRACE("\n");

        // Set save_name to 1
        save_name = 1;

        // Write name_saved to NVRAM
        numbytes2 = wiced_hal_write_nvram( WICED_NVRAM_VSID_START + 1, sizeof(save_name), &save_name, &status2 );

        // Reset the system watchdog
        wiced_hal_wdog_reset_system();
    }

    // Check if data_f matches data_WBF, put the device in OTA if receives "WBF"
    else if (memcmp(DATA_WBF, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_WBF, set save_name to 0 and write it to NVRAM
    	save_name = 0;
        numbytes2 = wiced_hal_write_nvram( WICED_NVRAM_VSID_START + 1, sizeof(save_name), &save_name, &status2 );

        // Print BT_LOCAL_NAME to the debug trace using the UART
        wiced_hal_puart_print(BT_LOCAL_NAME);

        // Reset the system watchdog
        wiced_hal_wdog_reset_system();
    }

    // Check if data_f matches data_WMA, put the device in OTA if receives "WMA"
    else if (memcmp(DATA_WMA, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_WMA, do some operations (commented out in the code)

        /*memcpy(data_uuid, &data_Write[3], 16);
        memcpy(data_ma_save, data_uuid, 16);
        numbytes3 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 2, sizeof(data_ma_save), &data_ma_save, &status3);
        WICED_BT_TRACE("Mac Address Saved: ");
        for (int i = 0; i < 6; i++)
        {
            wiced_hal_puart_write(data_ma_save[i]);
        }
        WICED_BT_TRACE("\n");
        flag2 = 1;
        numbytes4 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 3, sizeof(flag2), &flag2, &status4);
        flag3 = 0;
        numbytes5 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 4, sizeof(flag2), &flag3, &status5);
        start_Tsm();*/

        // Commented code is not executed, so this section has no effect in the current implementation.

    }

    // Check if data_f matches data_WiFi, put the device in OTA if receives "WiF"
    else if (memcmp(DATA_WiFi, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_WiFi, print "OCS: OK" to the debug trace
        WICED_BT_TRACE("OCS: OK");

        // Set status_Online to WICED_TRUE
        status_Online = WICED_TRUE;
    }
}


/************************************************************************************************************************************
 * Function Name: app_hci_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The function takes a pointer to uint8_t data and processes it based on the values it contains. It checks whether the first 3
 *	bytes of the input data match predefined pattern data_CMA. If the data matches the pattern, it sets flag2 to 0 and writes it to
 *	NVRAM. Additionally, it sets flag3 to 1 and writes it to NVRAM. The function concludes by resetting the system watchdog.
 *	The function generates a random MAC.
 *
 * Parameters:
 * 	uint8_t *data_C_MA						: Pointer to data.
 ***********************************************************************************************************************************/
void process_CMA(uint8_t *data_C_MA)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Copy the first 3 bytes of data_C_MA into the data_f array
    memcpy(data_f, data_C_MA, 3);

    // Check if data_f matches data_CMA, put the device in OTA if receives "CMA"
    if (memcmp(DATA_CMA, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_CMA, set flag2 to 0 and write it to NVRAM
    	save_MAC = 0; // Set flag to indicate whether the MAC address needs to be saved
        numbytes4 = wiced_hal_write_nvram( WICED_NVRAM_VSID_START + 3, sizeof(save_MAC), &save_MAC, &status4 );

        // Set flag3 to 1 and write it to NVRAM
        flag3 = 1; // flag_process_CMA.  Set flag to indicate that CMA data needs to be processed
        numbytes5 = wiced_hal_write_nvram( WICED_NVRAM_VSID_START + 4, sizeof(save_MAC), &flag3, &status5 );

        // Reset the system watchdog
        wiced_hal_wdog_reset_system();
    }
}


/************************************************************************************************************************************
 * Function Name: process_data_config(uint8_t *data_dc)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The process_data_config function takes a pointer to uint8_t data and processes it based on the values it contains. It checks
 *	whether the first 3 bytes of the input data match predefined patterns data_RBN and data_RMA. If the data matches any of these
 *	patterns, it reads or sets save_name, save_MAC, and flag3 from/to NVRAM and performs certain actions accordingly. The function also
 *	displays relevant information using the UART debug trace. The names of the flags can be improved to better reflect their specific
 *	purpose and meaning in the context of the application.
 *
 * Parameters:
 * 	uint8_t *data_dc						: Pointer to data.
 ***********************************************************************************************************************************/
void process_data_config(uint8_t *data_dc)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Copy the first 3 bytes of data_dc into the data_f array
    memcpy(data_f, data_dc, 3);

    // Check if data_f matches data_RBN, put the device in OTA if receives "RBN"
    if (memcmp(DATA_RBN, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_RBN, read the value of save_name from NVRAM
        // and display the Beacon Name accordingly
        numbytes2 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START + 1, sizeof(save_MAC), &save_name, &status2 );

        // If save_name is 1, read the Beacon Name from NVRAM and display it
        if (save_name == 1)
        {
            numbytes1 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(data_bn_save), &data_bn_save, &status1);
            WICED_BT_TRACE("Name Beacon: ");
            for (int i = 0; i < sizeof(data_bn_save) - 1; i++)
            {
                wiced_hal_puart_write(data_bn_save[i]);
            }
            WICED_BT_TRACE("\n");
            memcpy(BT_LOCAL_NAME, data_bn_save, 8);
        }
        // If save_name is 0, display the default Beacon Name and use it
        else
        {
            WICED_BT_TRACE("Name Beacon: ");
            for (int i = 0; i < sizeof(BT_LOCAL_NAME_DEFAULT) - 1; i++)
            {
                wiced_hal_puart_write(BT_LOCAL_NAME_DEFAULT[i]);
            }
            WICED_BT_TRACE("\n");
            memcpy(BT_LOCAL_NAME, BT_LOCAL_NAME_DEFAULT, 8);
        }
    }

    // Check if data_f matches data_RMA, put the device in OTA if receives "RMA"
    else if (memcmp(DATA_RMA, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_RMA, read the values of flag2 and flag3 from NVRAM
        numbytes4 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 3, sizeof(save_MAC), &save_MAC, &status4);
        numbytes5 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 4, sizeof(save_MAC), &flag3, &status5);

        // If both flag2 and flag3 are set, read the MAC address from NVRAM and display it
        if (save_MAC && flag3)
        {
            numbytes6 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 5, sizeof(data_ma_save), &data_ma_save, &status6);
            WICED_BT_TRACE("Mac Address: %s \n", data_ma_save);
        }
        // If flag2 is not set and flag3 is set, display "Mac Address Random"
        else if (!save_MAC && flag3)
        {
            WICED_BT_TRACE("Mac Address Random\n");
        }
        // If flag2 is set and flag3 is not set, read the MAC address from NVRAM and display it
        else if (save_MAC && !flag3)
        {
            numbytes4 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 3, sizeof(save_MAC), &save_MAC, &status4);
            WICED_BT_TRACE("Mac Address: %s\n", data_ma_save);
        }
        // If neither flag2 nor flag3 is set, display "Mac Address Null"
        else
        {
            WICED_BT_TRACE("Mac Address Null\n");
        }
    }
}


/************************************************************************************************************************************
 * Function Name: process_ODT(uint8_t *data_ODT)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The function takes a pointer to uint8_t data and processes it based on the values it contains. It checks whether the first 3
 *	bytes of the input data match predefined patterns data_OS and data_OTA. If the data matches data_OS, it prints "Finish" to the
 *	debug trace and stops a process called TOnline. If the data matches data_OTA, it calls a function start_trOTA with a parameter of
 *	500. The commented-out lines seem to be related to timers, but they are currently not active in the code.
 *
 * Parameters:
 * 	uint8_t *data_ODT						: Pointer to data.
 ***********************************************************************************************************************************/
void process_ODT(uint8_t *data_ODT)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Copy the first 3 bytes of data_ODT into the data_f array
    memcpy(data_f, data_ODT, 3);

    // Check if data_f matches data_OS, put the device in OTA if receives "OCS"
    if (memcmp(DATA_OS, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_OS, print "Finish" to the debug trace
        WICED_BT_TRACE("Finish\n");

        // Stop an ongoing process called TOnline
        stop_TOnline();

        // The next two lines are commented out, but they may start and stop timers related to clock_Online_long
        //wiced_stop_timer(&timer_Online);
        //wiced_start_timer(&timer_Online, clock_Online_long);
    }
    // Check if data_f matches data_OTA, put the device in OTA if receives "OTA"
    else if (memcmp(DATA_OTA, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_OTA, call the function start_trOTA with a parameter of 500
        // (This could be a function that initiates Over-The-Air (OTA) firmware updates)
        start_trOTA(500);
    }
}


/************************************************************************************************************************************
 * Function Name: process_SOM(uint8_t *data_S_OM)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The process_SOM function takes a pointer to uint8_t data and processes it based on the values it contains. It checks whether the
 *	first 3 bytes of the input data match various predefined patterns (data_SOM, data_SSC, data_SAC, data_SEG, data_SPI). Depending
 *	on the matching pattern, the function performs different actions, such as printing messages to the debug trace and calling
 *	specific functions (event_select_OTA, init_event_ADC, init_event_RAC, init_event_gap, gap_rebroadcastLR, start_TSPI, clr_spi).
 *	The names of the patterns and variables could be improved to be more descriptive and informative.
 *
 * Parameters:
 * 	uint8_t *data_S_OM						: Pointer to data.
 ***********************************************************************************************************************************/
void process_SOM(uint8_t *data_S_OM)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Copy the first 3 bytes of data_S_OM into the data_f array
    memcpy(data_f, data_S_OM, 3);

    // Check if data_f matches data_SOM, put the device in OTA if receives "SOM"
    if (memcmp(DATA_SOM, data_f, sizeof(data_f)) == 0)
    {
        WICED_BT_TRACE("Command \"SOM\" received to switch the setting to OTA\r\n");
        event_select_OTA();					// Call the function event_select_OTA (switch to OTA)
    }

    // Check if data_f matches data_SSC, call the event to initialize the event ADC
    else if (memcmp(DATA_SSC, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_SSC, print "Event SSC" to the debug trace
        WICED_BT_TRACE("Command \"SSC\" received call to initialize the event ADC\r\n");
        init_event_ADC();					// Call the function to initialize the event ADC
    }

    // Check if data_f matches data_SAC, call the event to initialize the event RAC
    else if (memcmp(DATA_SAC, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_SAC, print "Report RAC" to the debug trace
        WICED_BT_TRACE("Command \"SAC\" received call to initialize the event RAC\r\n");
        init_event_RAC();					// Call the function to initialize the event RAC
    }

    // Check if data_f matches data_SEG
    else if (memcmp(DATA_SEG, data_f, sizeof(data_f)) == 0)
    {
        // If data_f matches data_SEG, print "Event SEG" to the debug trace
    	WICED_BT_TRACE("Command \"SEG\" received call to initialize the event GAP\r\n");
        init_event_gap();
    }
    // Check if data_f matches data_SPI
    else if (memcmp(DATA_SPI, data_f, sizeof(data_f)) == 0)
    {
    	WICED_BT_TRACE("Command \"SPI\"\r\n");

        // Check the values of variables value_da and value_sdb
        if (value_da == WICED_FALSE)
        {
            // If value_da is WICED_FALSE, perform the following actions:

            if (value_sdb == WICED_FALSE)
            {
                // If value_sdb is WICED_FALSE, print "Event SPI" to the debug trace
                WICED_BT_TRACE("Event SPI\n");

                // Call the function gap_rebroadcastLR with a parameter of 1
                gap_rebroadcastLR(2);//1);

                // Set value_sdb to WICED_TRUE and start_TSPI()
                value_sdb = WICED_TRUE;
                start_TSPI();
            }

            // Clear the variable spi (clr_spi)
            clr_spi();
        }
        else
        {
            // If value_da is not WICED_FALSE, print "NULL Event SPI" to the debug trace
            WICED_BT_TRACE("NULL Event SPI\n");
        }
    }
}


/************************************************************************************************************************************
 * Function Name: gap_transfer(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The gap_transfer function reads the value of flag11 from NVRAM and performs certain actions if the flag is not set (i.e., its
 *	value is 0). It prints the current value of flag11 to the debug trace using the UART. Then, it sets flag11 to 1 and writes it back
 *	to NVRAM. Finally, it prints BT_LOCAL_NAME to the debug trace using the UART and resets the system watchdog. The purpose of this
 *	function is to handle some transfer operation based on the value of flag11, but the specific actions are not provided in this
 *	function.
 ***********************************************************************************************************************************/
void gap_transfer(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Read the value of flag11 from NVRAM
    numbytes11 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 11, sizeof(flag11), &flag11, &status11);

    // Check if flag11 is not set (!flag11 means flag11 is 0)
    if (!flag11)
    {
        // Print the value of flag11 to the debug trace
        WICED_BT_TRACE("FLAG11: %d\n", flag11);

        // Set flag11 to 1 and write it to NVRAM
        flag11 = 1;
        numbytes11 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 11, sizeof(flag11), &flag11, &status11);

        // Print BT_LOCAL_NAME to the debug trace using the UART
        wiced_hal_puart_print(BT_LOCAL_NAME);

        // Reset the system watchdog
        wiced_hal_wdog_reset_system();
    }
}


/************************************************************************************************************************************
 * Function Name: gap_out_f(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary
 *	The gap_out_f function reads the value of flag11 from NVRAM and performs certain actions if the flag is set (i.e., its value is
 *	non-zero). It prints the current value of flag11 to the debug trace using the UART. Then, it sets flag11 to 0 and writes it back
 *	to NVRAM. Finally, it prints BT_LOCAL_NAME to the debug trace using the UART and resets the system watchdog. The specific purpose
 *	of this function is not evident from the provided code, but it seems to handle some "GAP OUT" operation based on the value of
 *	flag11.
 ***********************************************************************************************************************************/
void gap_out_f(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    // Read the value of flag11 from NVRAM
    numbytes11 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 11, sizeof(flag11), &flag11, &status11);

    // Check if flag11 is set (non-zero)
    if (flag11)
    {
        // Print the value of flag11 to the debug trace
        WICED_BT_TRACE("GAP OUT FLAG11: %d\n", flag11);

        // Set flag11 to 0 and write it to NVRAM
        flag11 = 0;
        numbytes11 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 11, sizeof(flag11), &flag11, &status11);

        // Print BT_LOCAL_NAME to the debug trace using the UART
        wiced_hal_puart_print(BT_LOCAL_NAME);

        // Reset the system watchdog
        wiced_hal_wdog_reset_system();
    }
}





