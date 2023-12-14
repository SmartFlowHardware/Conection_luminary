/*
 * init_mesh.h
 *
 *  Created on: 28 nov 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_APP_MESH_INIT_MESH_H_
#define SF_APP_BLE_APP_MESH_INIT_MESH_H_

#define EMBEDDED_PROV_MAX_NODES             64
#define EMBEDDED_PROV_LOCAL_ADDR            1
#define EMBEDDED_PROV_HOPS					2
#define EMBEDDED_PROV_APP_KEY_IDX           0
#define EMBEDDED_PROV_NET_KEY_IDX           0
#define EMBEDDED_PROV_NODE_ADDR_FIRST		WICED_NVRAM_VSID_START + 100
#define EMBEDDED_PROV_NODE_ADDR_LAST		(WICED_NVRAM_VSID_START + 100 + EMBEDDED_PROV_MAX_NODES)
#define BBD_ADDR_LEN     		6  				/**< Bluetooth device address length */
#define advertisement_central   0
#define advertisement_nodo      1

/* ------------ Variables used in the connection -------------- */
#define SIZE 10
/* Multi advertisement instance ID */
#define BEACON_EDDYSTONE_UID 1

uint8_t safe_macs, array_mac[SIZE];  /* Used for safe the status of the data full */
uint8_t conection_status[10]={0,0,0,0,0,0,0,0,0,0};   /* Variable que se usará para el estatus de conexión */
uint8_t addr[10];
wiced_result_t         status_flag= WICED_FALSE;  /* Fla for only start one time the advertisement */

void Conect_process1(wiced_bt_ble_scan_results_t *p_scan_result);
extern void beacon_set_eddystone_uid_advertisement_data_1(uint8_t addr1, uint8_t response, BD_ADDR  bdaddr_luminary[]);

typedef struct
{
	uint8_t conection_status;
	uint8_t addr;
	BD_ADDR  bdaddr_luminary[6];
	uint8_t rssi;
}base_data;
///* ------------------------------------------------------------- */

//uint8_t cur_element_idx = 0;
//uint8_t cur_model_idx = 0;

//typedef struct
//{
////    uint8_t		uuid[16];
//    uint8_t 	addr;					/**< Local Node Address inside of the Network */
//    uint8_t		last_addr;				/**< Next Node Address inside of the Network */
//    uint8_t		max_dst;				/**< Destiny Node hops inside of the Network */
//    uint8_t		fil_key_node[2];		/**< Local Device Key */
//    uint8_t		net_key_node[3];		/**< Mesh Network Key */
////    uint8_t  	num_elements;
////    uint8_t  	is_relay;          // the node is currently set as a relay
////    uint8_t  	dev_key[16];
////    int8_t   	rssi_table[EMBEDDED_PROV_MAX_NODES];
//} mesh_node_t;


uint8_t	inf_network[7];
uint8_t copy_network[7];

mesh_node_t node;
mesh_info_t info_mesh;   /*  My information of the red */


extern wiced_bool_t	is_provisioned;

void		prepare_network_info(const mesh_node_t *node, uint8_t *inf_network);
void		create_network(void);
void		copy_info_net(uint8_t *p_info_net);
void		mesh_app_factory_reset(void);
uint8_t		generate_random_number(void);
char*		transmit_node_data(mesh_node_t node, char* user_prefix);
//char* transmit_node_data(mesh_node_t node);
void self_configure(uint16_t node_addr);

extern void 				fill_data_base(wiced_bt_ble_scan_results_t *p_scan_result,uint8_t *p_uid_node);
typedef struct
{
	uint8_t conection_status;
	uint8_t addr;
	BD_ADDR  bdaddr_luminary[6];
	uint8_t rssi;
}mesh_daba_base_t;

mesh_daba_base_t mesh_data[10];


extern void gap_rebroadcastLR(int8_t slt, uint8_t addr);

#endif /* SF_APP_BLE_APP_MESH_INIT_MESH_H_ */
