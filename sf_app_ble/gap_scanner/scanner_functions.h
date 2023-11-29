/*
 * scanner_functions.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_GAP_SCANNER_SCANNER_FUNCTIONS_H_
#define SF_APP_BLE_GAP_SCANNER_SCANNER_FUNCTIONS_H_

/************************************************************************************************************************************
 *  Type  Definitions
 ***********************************************************************************************************************************/

#define LENGTH_NVRAM		22			/**< Maximum length to store in the NVRAM, it is for 3 Devices Max */
#define RSSI_THRESHOLD		-48			/**< RSSI limit to consider a strong signal */



/************************************************************************************************************************************
 *  Import Global Variables Definitions
 ***********************************************************************************************************************************/

/** Variables to manipulate the NVRAM */
extern wiced_result_t		status16;
extern uint16_t				numbytes16;

/** Variable to store the data in the NVRAM */
extern uint8_t				data_nvram[LENGTH_NVRAM];



/************************************************************************************************************************************
 *  Global/Static Variables Definitions
 ***********************************************************************************************************************************/

/** Variable to indicate if the sensor was found by the scanner */
uint8_t						found_sensor;

/** Variable to store the type of sensor found by the scanner */
volatile uint8_t			sensor_type;

/** Variable to indicate if the device was scanned */
wiced_bool_t				device_scanned;



/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

void               	start_scanner(void);
void 				scanCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
wiced_bool_t 		scanning_mac(wiced_bt_ble_scan_results_t *p_scan_result);


#endif /* SF_APP_BLE_GAP_SCANNER_SCANNER_FUNCTIONS_H_ */

