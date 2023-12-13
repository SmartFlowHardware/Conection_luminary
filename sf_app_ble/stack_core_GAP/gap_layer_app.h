/*
 * gap_layer_app.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_STACK_CORE_GAP_GAP_LAYER_APP_H_
#define SF_APP_BLE_STACK_CORE_GAP_GAP_LAYER_APP_H_

/************************************************************************************************************************************
 *  Type  Definitions
 ***********************************************************************************************************************************/

#define EDDYSTONE_UID_FRAME_LEN			20
#define EDDYSTONE_UID_NAMESPACE_LEN		10
#define EDDYSTONE_UID_INSTANCE_ID_LEN	6

#define BEACON_EDDYSTONE_UID			1
#define BEACON_EDDYSTONE_URL 			2
#define BEACON_EDDYSTONE_EID 			3
#define BEACON_EDDYSTONE_TLM 			4
#define BEACON_IBEACON       			5

#define SIZE_DATA_SEND					27

/* User defined UUID for iBeacon */
#define UUID_IBEACON     0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f

#define EMBEDDED_PROV_NODE_ADDR_FIRST		WICED_NVRAM_VSID_START + 100
#define EMBEDDED_PROV_NODE_ADDR_LAST		(WICED_NVRAM_VSID_START + 100 + EMBEDDED_PROV_MAX_NODES)


/************************************************************************************************************************************
 *  Global Variables Declarations
 ***********************************************************************************************************************************/

uint8_t filt_SPI[3] = {0x53,0x50,0x49};
uint8_t filt_NULL[3] = {0x31,0x31,0x31};
uint8_t data_mcc[6];

uint8_t mesh_device[3];

wiced_bool_t	is_provisioned;

char data_txsf[16];	// Original uint8_t data_txsf[16];

char* data_name_node;

uint8_t mesh_beacon[]={' ', 'M', 'E', 'S', 'S', 'A', 'G', 'E', 'L', 'A'};

/** Filter to send message to share information about the Network */
uint8_t mesh_conn_class[] = {0x43, 0x4f, 0x4e}; // CON

uint8_t mesh_message[]={'H','E','L','L','O','\n'};;


/************************************************************************************************************************************
 *  Extern Variables
 ***********************************************************************************************************************************/

extern const wiced_bt_cfg_settings_t app_cfg_settings2;
extern const wiced_bt_cfg_buf_pool_t app_buf_pools2[];

extern mesh_node_t node;
extern mesh_info_t info_mesh;
extern uint8_t	inf_network[8];
extern wiced_bool_t	conn_node_mesh;

wiced_bool_t mode_send_info;

//// Arrays to send data
//// Array with characters of the introduction
//extern char							introduction_array[4];
//
//// Array with of the Bluetooth Device Address
//extern char 						array_bd_addr_tx[17];
//
//// Array with the name of the beacon name
//extern char							name_beacon_tx[6];

// Array with ID of Health Thermometer
uint8_t								device_id_HT[] = "HT";

// Array with ID of Heart Rate
uint8_t								device_id_HR[] = "HR";

// Array with the ID beacon
uint8_t								array_send[]	= "S1";

// Variable to print the bluetooth device address
extern uint8_t						array_bd_addr[7];

// Array to save the name of the beacon
extern uint8_t						array_name_beacon[17];

extern wiced_bool_t					save_URL;


/************************************************************************************************************************************
 *  												Function Declarations
 ***********************************************************************************************************************************/

void 							gap_stack_init(void);

static void                     beacon_init(void);
static wiced_result_t           beacon_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
extern wiced_bt_gatt_status_t 	beacon_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );
static void                     beacon_advertisement_stopped(void);
static void                     beacon_set_app_advertisement_data2();
void                            beacon_set_app_advertisement_data3(void);
void                            node_set_app_advertisement_data(void);
void							mesh_set_app_advertisement_data(void);
void							app_set_scan_response_data(void);

void                            stop_rbdkst(void);

void                            gap_rebroadcastLR(int8_t slt,uint8_t addr);
void							gap_rebroadcastURL(int8_t slt);
void 							gap_rebroadcastBIO(uint8_t t_sensor, uint16_t data_device);

static void 					beacon_set_eddystone_uid_advertisement_data(void);
static void 					beacon_set_eddystone_ibeacon_advertisement_data();
void                            beacon_set_eddystone_uid_advertisement_data_1(BD_ADDR mac_addres, uint8_t addr);

//void 					app_add_peer_info( uint16_t conn_id, uint8_t s_type, uint8_t* p_bd_addr );
//wiced_bt_gatt_status_t 	app_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
//wiced_bt_gatt_status_t 	app_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status );
//void 					app_remove_peer_info( uint16_t conn_id );
//sensor_device_info_t * app_get_peer_information( uint16_t conn_id );

/************************************************************************************************************************************
 *  											Imported Function Declarations
 ***********************************************************************************************************************************/

extern void	                    config_Transceiver(void);
extern void                     init_config_logs(void);
extern void                     init_mac_logs(void);
extern void                     start_observe(void);
extern void						start_scanner(void);
extern void						start_pwm( void );

extern void                     set_outPuts(void);
extern void                     set_intPuts(void);
extern void                     register_pin_interrupt(void);
extern void                     event_select_OTA(void);
extern void                     set_adc_p(void);

extern void                     config_clk_timers(void);
extern void                     event_recover_OTA(void);
//extern void                     start_BTimers(void);

//extern void 					blinking_mode_pwm( uint8_t led_mode, uint8_t pwm_num, uint8_t led, uint16_t pwm_frequency, uint16_t toggle_count, uint16_t init_count );

extern void 					separate_array_into_two( uint8_t *p_url_beacon, char *p_array_ten, uint8_t *p_array_six );

extern wiced_bt_gatt_status_t 	beacon_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );


extern char*					transmit_node_data(mesh_node_t node, char* user_prefix);

extern void 					prepare_network_info(const mesh_node_t *node, uint8_t *inf_network);


#endif /* SF_APP_BLE_STACK_CORE_GAP_GAP_LAYER_APP_H_ */
