/************************************************************************************************************************************
 * File: config_logs.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *	This file includes the primary variables necessary for configuring a device in its central role. These variables serve as the
 *	functional settings for the device's operation. The following functions utilize these variables to initiate the device's
 *	functionality.
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
#include "config_logs.h"

/************************************************************************************************************************************
 *  												Function Definitions
 ***********************************************************************************************************************************/

/************************************************************************************************************************************
 * Function Name: init_config_logs(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function reads certain data from non-volatile memory (NVRAM) and then sets the BT_LOCAL_NAME variable to a specific name
 * 	based on the values of save_name, flag11, and Uart_pu8PutTx. Depending on the combination of these values, the beacon's name is
 * 	configured accordingly. The function also prints the name of the beacon to the UART for debugging purposes.
 ***********************************************************************************************************************************/
void init_config_logs(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Read data from NVRAM starting at address WICED_NVRAM_VSID_START+X, store in flagX variable
	numbytes2 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START+1, sizeof(save_name), &save_name, &status2 );
	numbytes11 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START+11, sizeof(flag11), &flag11, &status11 );
	numbytes12 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START+12, sizeof(Uart_pu8PutTx), &Uart_pu8PutTx, &status12 );

	// If save_name is 1 and flag11 is 0, enter this block
	if( ( save_name == 1 ) && ( flag11 == 0 ) )
	{
		//------------------------------------------------------------------------------------------
		// Read data from NVRAM starting at address WICED_NVRAM_VSID_START, store in data_bn_save variable
		numbytes1 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(data_bn_save), data_bn_save, &status1);

		WICED_BT_TRACE("Name Beacon1: ");
		for (int i = 0; i < sizeof(data_bn_save) - 1; i++)
		{
			wiced_hal_puart_write(data_bn_save[i]);
		}
		WICED_BT_TRACE("\n");

		// Copy the name of the beacon (first 10 bytes) to BT_LOCAL_NAME variable.
		memcpy(BT_LOCAL_NAME, data_bn_save, 10);
		//-----------------------------------------------------------------------------------------------
	}

	// If save_name is 0 and flag11 is 0, enter this block
	else if( ( save_name == 0 ) && ( flag11 == 0 ) )
	{
		WICED_BT_TRACE( "Name Beacon2: ");
		//for(int i=0;i<sizeof(BT_LOCAL_NAME_DEFAULT)-1; i++){wiced_hal_puart_write(BT_LOCAL_NAME_DEFAULT[i]);}
		WICED_BT_TRACE( "\n");


		//memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULT,10);
		//WICED_BT_TRACE( "Porciento3 = %X \n\r",Uart_pu8PutTx);
		//WICED_BT_TRACE( "Porciento4 = %d \n\r",Uart_pu8PutTx);

		// Select a default name for the beacon based on the value of Uart_pu8PutTx and copy it into BT_LOCAL_NAME.
		if( *Uart_pu8PutTx >= 0 && *Uart_pu8PutTx < 74 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb1,10);
			//for(int i=0;i<sizeof(BT_LOCAL_NAME_DEFAULTa0)-1; i++){wiced_hal_puart_write(BT_LOCAL_NAME_DEFAULTa0[i]);}
		}
		else if( *Uart_pu8PutTx >= 74 && *Uart_pu8PutTx < 76 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb2,10);
		}
		else if( *Uart_pu8PutTx >= 76 && *Uart_pu8PutTx < 78 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb3,10);
		}
		else if( *Uart_pu8PutTx >= 78 && *Uart_pu8PutTx < 80 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb4,10);
		}
		else if( *Uart_pu8PutTx >= 80 && *Uart_pu8PutTx < 82 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb5,10);
		}
		else if( *Uart_pu8PutTx >= 82 && *Uart_pu8PutTx < 84 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb6,10);
		}
		else if( *Uart_pu8PutTx >= 84 && *Uart_pu8PutTx < 86 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb7,10);
		}
		else if( *Uart_pu8PutTx >= 86 && *Uart_pu8PutTx < 88 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb8,10);
		}
		else if( *Uart_pu8PutTx >= 88 && *Uart_pu8PutTx < 91 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb9,10);
		}
		else if( *Uart_pu8PutTx >= 91 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTb10,10);
		}

		// Print the newly selected default name by sending each character to the UART
		for( int i=0; i<10; i++ ){ wiced_hal_puart_write(BT_LOCAL_NAME[i]); }
		WICED_BT_TRACE( "\n");
	}

	// If neither of the previous conditions is true, enter this block
	else
	{
		// Select a different set of default names based on the value of Uart_pu8PutTx and copy it into BT_LOCAL_NAME
		if( *Uart_pu8PutTx >= 0 && *Uart_pu8PutTx < 10 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT0,10);
			//for(int i=0;i<sizeof(BT_LOCAL_NAME_DEFAULTa0)-1; i++){wiced_hal_puart_write(BT_LOCAL_NAME_DEFAULTa0[i]);}
		}
		else if( *Uart_pu8PutTx >= 10 && *Uart_pu8PutTx < 20 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT1,10);
		}
		else if( *Uart_pu8PutTx >= 20 && *Uart_pu8PutTx < 30 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT2,10);
		}
		else if( *Uart_pu8PutTx >= 30 && *Uart_pu8PutTx < 40 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT3,10);
		}
		else if( *Uart_pu8PutTx >= 40 && *Uart_pu8PutTx < 50 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT4,10);
		}
		else if( *Uart_pu8PutTx >= 50 && *Uart_pu8PutTx < 60 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT5,10);
		}
		else if( *Uart_pu8PutTx >= 60 && *Uart_pu8PutTx < 70 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT6,10);
		}
		else if( *Uart_pu8PutTx >= 70 && *Uart_pu8PutTx < 80 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT7,10);
		}
		else if( *Uart_pu8PutTx >= 80 && *Uart_pu8PutTx < 90 )
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT8,10);
		}
		else if( *Uart_pu8PutTx >= 90 && *Uart_pu8PutTx < 100)
		{
			memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULTT9,10);
		}

		//-------------------------------------------
		WICED_BT_TRACE( "Name Beacon3: ");

		for( int i=0; i<10; i++ ){ wiced_hal_puart_write(BT_LOCAL_NAME[i]); }
		WICED_BT_TRACE( "\n");
		//for(int i=0;i<sizeof(BT_LOCAL_NAME_DEFAULT2)-1; i++){wiced_hal_puart_write(BT_LOCAL_NAME_DEFAULT2[i]);}
		//WICED_BT_TRACE( "\n");
		//memcpy(BT_LOCAL_NAME,BT_LOCAL_NAME_DEFAULT2,10);
		//memcpy(BT_LOCAL_NAME +10,&Uart_pu8GetTx,sizeof(Uart_pu8GetTx));
	 }
}


/************************************************************************************************************************************
 * Function Name: init_mac_logs(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	The function reads certain data from non-volatile memory (NVRAM) and sets the Bluetooth device address ('bda') based on the
 * 	values of flag2 and flag3. If certain conditions are met, it reads and prints the stored Mac Address; otherwise, it generates a
 * 	random Mac Address or uses a default value. The final Bluetooth device address is set using wiced_bt_set_local_bdaddr() function.
 ***********************************************************************************************************************************/
void init_mac_logs(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_bt_device_address_t bda;

	// Read data from NVRAM starting at address WICED_NVRAM_VSID_START+X, store in flagX variable
	numbytes4 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 3, sizeof(save_MAC), &save_MAC, &status4);
	numbytes5 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 4, sizeof(save_MAC), &flag3, &status5);

	// Check if both flag2 and flag3 are true (non-zero)
	if( save_MAC && flag3 )
	{
		// Read data from NVRAM starting at address WICED_NVRAM_VSID_START+5, store in data_ma_save variable.
		numbytes6 = wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 5, sizeof(data_ma_save), data_ma_save, &status6);

		WICED_BT_TRACE("Mac Address: %B ", data_ma_save);
		WICED_BT_TRACE("\n");

		// Copy the Mac Address to the 'bda' (Bluetooth Device Address) array.
		for( uint8_t i = 0; i < BD_ADDR_LEN; i++ )
		{
			bda[i] = data_ma_save[i];
		}
	}

	// Check if flag2 is false (0) and flag3 is true (non-zero)
	else if( !save_MAC && flag3 )
	{
		WICED_BT_TRACE("Mac Address Random\n");

		// Generate a random Mac Address and store it in 'bda' array
		for(uint8_t i = 0; i < BD_ADDR_LEN; i++)
		{
			bda[i] = wiced_hal_rand_gen_num();
		}

		// Set to true and save it in NVRAM.
		save_MAC = 1;
		numbytes4 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 3, sizeof(save_MAC), &save_MAC, &status4);

		// Copy the generated Mac Address to 'data_ma_save' and save it in NVRAM
		memcpy(data_ma_save, bda, 16);
		numbytes6 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 5, sizeof(data_ma_save), data_ma_save, &status6);

		WICED_BT_TRACE("Mac Address Random: %B\n", data_ma_save);

	}

	// Check if flag2 is true (non-zero) and flag3 is false (0)
	else if( save_MAC && !flag3 )
	{
		numbytes4 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START+3, sizeof(save_MAC), &save_MAC, &status4 );

		// Read data from NVRAM starting at address WICED_NVRAM_VSID_START+2, store in data_ma_save variable
		numbytes3 = wiced_hal_read_nvram( WICED_NVRAM_VSID_START+2, sizeof(data_ma_save), data_ma_save, &status3 );

		//for(int i=0;i< sizeof(Msm_MCU)-1; i++){wiced_hal_puart_write(Msm_MCU[i]);}

		WICED_BT_TRACE("Mac Address: %B \r\n", data_ma_save);

		//for(int i=0;i< 6; i++){wiced_hal_puart_write(data_ma_save[i]);}

		// Copy the Mac Address to the 'bda' (Bluetooth Device Address) array
		for(uint8_t i = 0; i < BD_ADDR_LEN; i++)
		{
			bda[i] = data_ma_save[i];
		}
	}

	// If none of the previous conditions are true, enter this block (Mac Address Null)
	else
	{
		// Set a default Mac Address in 'bda' array
		WICED_BT_TRACE("Mac Address Null\n");
		bda[0]=0x10;
		bda[1]=0x10;
		bda[2]=0x10;
		bda[3]=0x10;
		bda[4]=0x10;
		bda[5]=0x10;
	}

	// Set the local Bluetooth device address using the 'bda' array
	wiced_bt_ble_address_type_t macc = BLE_ADDR_PUBLIC_ID;
	wiced_bt_set_local_bdaddr(bda,macc);
}


/************************************************************************************************************************************
 * Function Name: send_level_p(uint8_t *p_dlevel)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	This function takes a pointer to a uint8_t (byte) as an input parameter, updates two global variables Uart_pu8PutTx and
 * 	Uart_pu8GetTx to point to the same memory location as the input parameter, and then writes the value of Uart_pu8PutTx to NVRAM.
 * 	Note that Uart_pu8PutTx and Uart_pu8GetTx are assumed to be global variables defined elsewhere in the code.
 *
 * Parameters:
 * 	uint8_t *p_dlevel					      			: Pointer to data type to compare in the function.
 ***********************************************************************************************************************************/
void send_level_p(uint8_t *p_dlevel)
{
	// Uncomment the following lines to print the hexadecimal and decimal values of p_dlevel to the UART.
    // WICED_BT_TRACE("Porciento1 = %X \n\r",p_dlevel);
    // WICED_BT_TRACE("Porciento2 = %d \n\r",p_dlevel);

    // Set the value of the global variable Uart_pu8PutTx to the address pointed by p_dlevel.
    // This makes Uart_pu8PutTx point to the same memory location as p_dlevel.
    Uart_pu8PutTx = p_dlevel;

    // Set the value of the global variable Uart_pu8GetTx to the address pointed by p_dlevel.
    // This makes Uart_pu8GetTx point to the same memory location as p_dlevel.
    Uart_pu8GetTx = p_dlevel;

    // Uncomment the following lines to print the hexadecimal and decimal values of Uart_pu8PutTx to the UART.
    // WICED_BT_TRACE("Porciento3 = %X \n\r",Uart_pu8PutTx);
    // WICED_BT_TRACE("Porciento4 = %d \n\r",Uart_pu8PutTx);

    // Write the value of Uart_pu8PutTx to NVRAM starting at address WICED_NVRAM_VSID_START+12.
    // The size of the data being written is sizeof(Uart_pu8PutTx).
    // The status of the NVRAM write operation is stored in the status12 variable.
    numbytes12 = wiced_hal_write_nvram(WICED_NVRAM_VSID_START + 12, sizeof(Uart_pu8PutTx), &Uart_pu8PutTx, &status12);
}
