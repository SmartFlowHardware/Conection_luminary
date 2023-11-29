/*
 * process_Rx_trans.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_PROCESSES_BLE_PROCESSES_RX_TRANS_H_
#define SF_APP_BLE_PROCESSES_BLE_PROCESSES_RX_TRANS_H_


/************************************************************************************************************************************
 *  Type  Definitions
 ***********************************************************************************************************************************/

#define DATA_WBN 				"WBN"				/**< Meaning of WBN: Write Beacon Name */
#define DATA_WBF 				"WBF"				/**< Meaning of WBF: Write Beacon */
#define DATA_WMA 				"WMA"				/**< Meaning of WMA: Write  */
#define DATA_RBN 				"RBN"				/**< Meaning of RBN: Read Beacon Name */
#define DATA_RMA 				"RMA"				/**< Meaning of RMA: Read Beacon MAC*/
#define DATA_CMA 				"CMA"				/**< Meaning of CMA: Generate a random MAC */
#define DATA_WiFi				"WiF"				/**< Meaning of WiF: WiFi */
#define DATA_OS  				"OCS"				/**< Meaning of OCS:  */
#define DATA_OTA 				"OTA"				/**< Meaning of OTA: Over The Air */
#define DATA_SOM 				"SOM"				/**< Meaning of SOM: */
#define DATA_SSC 				"SSC"				/**< Meaning of SSC: */
#define DATA_SAC 				"SAC"				/**< Meaning of SAC: */
#define DATA_SEG 				"SEG"				/**< Meaning of SEG: */
#define DATA_SPI 				"SPI"				/**< Meaning of SPI: */

#define BT_LOCAL_NAME_DEFAULT	"BIOME BLE"



/************************************************************************************************************************************
 *  Extern Variables
 ***********************************************************************************************************************************/

extern uint8_t 			BT_LOCAL_NAME[64];
extern wiced_bool_t 	status_Online;
extern wiced_bool_t 	value_da;



/************************************************************************************************************************************
 *  Global Variables Declarations
 ***********************************************************************************************************************************/

wiced_result_t  status1;
uint16_t        numbytes1;

wiced_result_t  status2;
uint16_t        numbytes2;

wiced_result_t  status3;
uint16_t        numbytes3;

wiced_result_t  status4;
uint16_t        numbytes4;

wiced_result_t  status5;
uint16_t        numbytes5;

wiced_result_t  status6;
uint16_t        numbytes6;

wiced_result_t  status11;
uint16_t        numbytes11;

uint8_t 		data_f[3];

uint8_t			save_name  = 0;
uint8_t 		save_MAC  = 0;
uint8_t 		flag3  = 0;
uint8_t 		flag11 = 0;

uint8_t 		data_bn_save[10];
uint8_t 		data_ma_save[6];
uint8_t 		data_flash[20];
uint8_t 		data_uuid[16];

wiced_bool_t 	ctr_sm = WICED_FALSE;
wiced_bool_t 	value_sdb = WICED_FALSE;



/************************************************************************************************************************************
 *  Export Function Declarations
 ***********************************************************************************************************************************/

extern void		stop_TOnline(void);
extern void 	event_recover_OTA(void);
extern void 	event_select_OTA(void);
extern void 	init_event_ADC(void);
extern void 	init_event_RAC(void);
extern void 	init_event_gap(void);
extern void 	start_Tsm(void);
extern void 	start_trOTA(uint32_t t_clk);
extern void 	start_TSPI(void);
extern void 	clr_spi(void);
extern void 	gap_rebroadcastLR(int8_t slt);
extern void 	gap_rebroadcastURL(int8_t slt);



/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

void 			process_Write(uint8_t *data_Write);
void 			process_CMA(uint8_t *data_C_MA);
void 			process_data_config(uint8_t *data_dc);
void 			process_OCS(uint8_t *data_OCS);
void 			gap_transfer(void);
void 			gap_out_f(void);



#endif /* SF_APP_BLE_PROCESSES_BLE_PROCESSES_RX_TRANS_H_ */

