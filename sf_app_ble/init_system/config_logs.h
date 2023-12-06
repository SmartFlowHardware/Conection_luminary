/*
 * config_logs.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_INIT_SYSTEM_CONFIG_LOGS_H_
#define SF_APP_BLE_INIT_SYSTEM_CONFIG_LOGS_H_

/************************************************************************************************************************************
 *  Type  Definitions
 ***********************************************************************************************************************************/

#define BT_LOCAL_NAME_DEFAULT		"NODEL BLE8"
#define BT_LOCAL_NAME_DEFAULT2		"Nodel BLE8"

#define BT_LOCAL_NAME_DEFAULTa0		"NODEL BL05"
#define BT_LOCAL_NAME_DEFAULTa1		"NODEL BLE1"
#define BT_LOCAL_NAME_DEFAULTa2		"NODEL BLE2"
#define BT_LOCAL_NAME_DEFAULTa3		"NODEL BLE3"
#define BT_LOCAL_NAME_DEFAULTa4		"NODEL BLE4"
#define BT_LOCAL_NAME_DEFAULTa5		"NODEL BLE5"
#define BT_LOCAL_NAME_DEFAULTa6		"NODEL BLE6"
#define BT_LOCAL_NAME_DEFAULTa7		"NODEL BLE7"
#define BT_LOCAL_NAME_DEFAULTa8		"NODEL BLE8"
#define BT_LOCAL_NAME_DEFAULTa9		"NODEL BLE9"
#define BT_LOCAL_NAME_DEFAULTa10	"NODEL BL10"

#define BT_LOCAL_NAME_DEFAULTb0		"NODEL BS05"
#define BT_LOCAL_NAME_DEFAULTb1		"NODEL BSL1"
#define BT_LOCAL_NAME_DEFAULTb2		"NODEL BSL2"
#define BT_LOCAL_NAME_DEFAULTb3		"NODEL BSL3"
#define BT_LOCAL_NAME_DEFAULTb4		"NODEL BSL4"
#define BT_LOCAL_NAME_DEFAULTb5		"NODEL BSL5"
#define BT_LOCAL_NAME_DEFAULTb6		"NODEL BSL6"
#define BT_LOCAL_NAME_DEFAULTb7		"NODEL BSL7"
#define BT_LOCAL_NAME_DEFAULTb8		"NODEL BSL8"
#define BT_LOCAL_NAME_DEFAULTb9		"NODEL BSL9"
#define BT_LOCAL_NAME_DEFAULTb10	"NODEL BS10"

#define BT_LOCAL_NAME_DEFAULTT0		"NODEL VT05"
#define BT_LOCAL_NAME_DEFAULTT1		"NODEL VTE1"
#define BT_LOCAL_NAME_DEFAULTT2		"NODEL VTE2"
#define BT_LOCAL_NAME_DEFAULTT3		"NODEL VTE3"
#define BT_LOCAL_NAME_DEFAULTT4		"NODEL VTE4"
#define BT_LOCAL_NAME_DEFAULTT5		"NODEL VTE5"
#define BT_LOCAL_NAME_DEFAULTT6		"NODEL VTE6"
#define BT_LOCAL_NAME_DEFAULTT7		"NODEL VTE7"
#define BT_LOCAL_NAME_DEFAULTT8		"NODEL VTE8"
#define BT_LOCAL_NAME_DEFAULTT9		"NODEL VTE9"
#define BT_LOCAL_NAME_DEFAULTT10	"NODEL VT10"



/************************************************************************************************************************************
 *  Global Variables Declarations
 ***********************************************************************************************************************************/

uint8_t				Uart_u8TxBuffer[1];
uint8_t				Adc_value[1];

wiced_result_t		status12;
uint16_t			numbytes12;

volatile uint8_t*	Uart_pu8PutTx=&Uart_u8TxBuffer[0];
volatile uint8_t*	Uart_pu8GetTx=&Uart_u8TxBuffer[0];



/************************************************************************************************************************************
 *  Extern Variables
 ***********************************************************************************************************************************/

extern uint8_t				BT_LOCAL_NAME[64];

extern wiced_result_t		status1;

extern uint8_t				save_name;
extern uint8_t				save_MAC;
extern uint8_t				flag3;
extern uint8_t				flag11;

extern wiced_result_t		status1;
extern uint16_t				numbytes1;

extern wiced_result_t		status2;
extern uint16_t				numbytes2;

extern wiced_result_t		status3;
extern uint16_t				numbytes3;

extern wiced_result_t		status4;
extern uint16_t				numbytes4;

extern wiced_result_t		status5;
extern uint16_t				numbytes5;

extern wiced_result_t		status6;
extern uint16_t				numbytes6;

extern wiced_result_t		status11;
extern uint16_t				numbytes11;

wiced_result_t  status20;
uint16_t        numbytes20;

extern unsigned char		data_bn_save[10];
extern unsigned char		data_ma_save[6];



/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

void						init_config_logs(void);
void						init_mac_logs(void);

extern uint8_t generate_random_number(void);


#endif /* SF_APP_BLE_INIT_SYSTEM_CONFIG_LOGS_H_ */

