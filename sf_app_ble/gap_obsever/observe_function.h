/*
 * observe_function.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_GAP_OBSERVER_OBSERVE_FUNCTION_H_
#define SF_APP_BLE_GAP_OBSERVER_OBSERVE_FUNCTION_H_

/************************************************************************************************************************************
 *  Type  Definitions
 ***********************************************************************************************************************************/

      #define clk_radc 20
      #define clk_radc120 120
	  wiced_timer_t timer_radc;
	  void        f_timer_radc( uint32_t data );


/************************************************************************************************************************************
 *  Global/Static Variables Definitions
 ***********************************************************************************************************************************/

wiced_bool_t value_Adc=WICED_TRUE;
wiced_bool_t value_ac=WICED_TRUE;
wiced_bool_t value_gap=WICED_TRUE;
wiced_bool_t value_da=WICED_FALSE;

wiced_bool_t	blinking_led_timer;

/** Variable to indicate if the sensor was found by the scanner */
//uint8_t				found_sensor;

/** Variable to store the type of sensor found by the scanner */
//volatile uint8_t	sensor_type;

// Array of beacon identifier
//uint8_t			array_beacon_id[] = { 1, 2, 3, 4 };

// Filters to the Mesh
wiced_bool_t	find_lamp;
wiced_bool_t	find_tag;
wiced_bool_t	find_node;
wiced_bool_t	conn_node_mesh;

/** --- Variables to identify the type of Name --- */

/** Variable to find lamp device */
uint8_t		filter_lamp[] = 		{ 0x4c, 0x34, 0x53, 0x45, 0x43, 0x20, 0x42, 0x53, 0x4c };	// L4SEC BSL

/** Variable to find tag device */
uint8_t		filter_tag[] = 			{ 0x4c, 0x41, 0x49, 0x52, 0x44, 0x20, 0x42, 0x4c };			// LAIRD BL

/** Variable to find node device */
uint8_t		filter_node[5] = 		{ 0x4e, 0x4f, 0x44, 0x45, 0x4C};	// NODEL BSL
uint8_t NODEL_BSL1[5]  = {0x4E,0x4F,0x44,0x45,0x42};   //NODEB
uint8_t KEY[3]={0x4E, 0x45, 0x54};       // NET
uint8_t CN1[2]={0x43,0x4E};

/** Variable to find mesh connection */
uint8_t		filter_mesh_conn[] = 	{ 0x43, 0x4f, 0x4e };										// CON

/** Variable to find node response */
uint8_t		filter_node_rsp[] = 	{ 0x43, 0x4e };												// CN

//uint8_t		filter_node_lamp[] = { 0x01, 0x00, 0x00 };

//uint8_t	Filt_ID[2] = {0x42, 0x4e};											// Filter ( BN )

//uint8_t dataFilt[5];
//uint8_t dataFiltBC[5];

extern wiced_bool_t is_provisioned;
extern mesh_node_t node;
extern uint8_t copy_network[8];

/************************************************************************************************************************************
 *  Imported Function Declarations
 ***********************************************************************************************************************************/

extern void 		prevention_status(void);
extern void 		prevention_inspection(void);
extern void 		gap_transfer(void);
extern void			stop_timer_st_online(void);
extern void 		beacon_set_app_advertisement_data3(void);
extern void 		start_Treturn(void);
extern void			clear_da(void);

extern void 		start_rb1(void);
extern void 		start_rb2(void);
extern void 		start_rb3(void);
extern void 		start_rb4(void);
extern void 		start_rb5(void);

extern void 		start_multi_bc1(void);
extern void 		start_multi_bc2(void);
extern void 		start_multi_bc3(void);

extern void start_lamp_timer(void);
extern void start_tag_timer(void);
extern void start_node_timer(void);
extern void start_bled_timer(void);

extern void copy_info_net(uint8_t *p_info_net);
extern void Conect_process1(wiced_bt_ble_scan_results_t *p_scan_result);

//extern             void start_trOTA(uint32_t t_clk);



/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

static void        	observer_mesh_adv_report( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
void               	start_observe(void);
void               	init_event_ADC(void);
void               	init_event_RAC(void);
void               	init_event_gap(void);
void 				fill_data_base(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_uid_node);
#endif /* SF_APP_BLE_GAP_OBSERVER_OBSERVE_FUNCTION_H_ */

