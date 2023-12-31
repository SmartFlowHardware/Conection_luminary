/*
 * processes_timer.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_PROCESSES_BLE_PROCESSES_TIMERS_H_
#define SF_APP_BLE_PROCESSES_BLE_PROCESSES_TIMERS_H_


#define TIMES_BLINKING_LED	5

/************************************************************************************************************************************
 *  Static/Global Variables Declarations
 ***********************************************************************************************************************************/

wiced_bool_t 			ctr_p6 = WICED_TRUE;
wiced_bool_t 			status_Online = WICED_TRUE;
wiced_bool_t 			value_inspection=WICED_TRUE;
uint16_t				app_timer_count;
uint8_t					app_timer_button;



/************************************************************************************************************************************
 *  Imported Variables Declarations
 ***********************************************************************************************************************************/

extern wiced_bool_t 	value_gap;
extern wiced_bool_t 	ctr_sm;
extern wiced_bool_t 	value_da;
extern wiced_bool_t 	value_sdb;

/** Variable extern that indicate if found a Lamp */
extern uint8_t find_lamp;

/** Variable extern that indicate if found a Tag */
extern uint8_t find_tag;

/** Variable extern that indicate if found a Tag */
extern uint8_t find_node;

/** Declare variable to start blinking led Timer */
extern wiced_timer_t	bled_timer;

extern wiced_bool_t		blinking_led_timer;

extern wiced_bool_t		acuse_pressed;
extern wiced_bool_t		onoff_pressed;




/************************************************************************************************************************************
 *  Imported Function Declarations
 ***********************************************************************************************************************************/

extern void				scanCallback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
extern void				start_TOnline(void);
extern void 			start_TOnline_long(void);
extern void 			prevention_status(void);
extern void 			prevention_status2(void);
extern void 			beacon_set_app_advertisement_data4(void);
extern void 			start_TOsm(void);
extern void 			process_rOTA(void);
extern void 			stop_rbdkst(void);

extern void				start_lamp_timer(void);
extern void				start_tag_timer(void);
extern void				start_node_timer(void);
extern void				start_bled_timer(void);
extern void 			stop_timer_succes(void);
extern void 			stop_blink(void);
extern void	 			STOP_LED_provisioner(void);


/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

void      				f_timer_SPI( void);
void					f_timer_Online( uint32_t data );
void					f_timer_st_Online( uint32_t data );
void					f_timer_inspection( uint32_t data );
void					f_timer_return( uint32_t data );
void					f_timer_sm( uint32_t data );
void					f_timer_da( void);
void					f_timer_clrspi( void);
void					f_app_main( TIMER_PARAM_TYPE arg );

void					f_timer_lamp( TIMER_PARAM_TYPE arg );
void					f_timer_tag( TIMER_PARAM_TYPE arg );
void					f_timer_node( TIMER_PARAM_TYPE arg );
void					f_timer_bled( TIMER_PARAM_TYPE arg );
void				    f_timer_succes_conection( TIMER_PARAM_TYPE arg );
void 					f_timer_blink( TIMER_PARAM_TYPE arg );
void 					f_timer_seePr( TIMER_PARAM_TYPE arg );

extern void beacon_set_eddystone_uid_advertisement_data_1(uint8_t addr1, uint8_t response, BD_ADDR  bdaddr_luminary[]);

#endif /* SF_APP_BLE_PROCESSES_BLE_PROCESSES_TIMERS_H_ */
