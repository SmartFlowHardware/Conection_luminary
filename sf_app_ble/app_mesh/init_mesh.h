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
#define EMBEDDED_PROV_APP_KEY_IDX           0
#define EMBEDDED_PROV_NET_KEY_IDX           0
#define EMBEDDED_PROV_NODE_ADDR_FIRST		WICED_NVRAM_VSID_START + 100
#define EMBEDDED_PROV_NODE_ADDR_LAST		(WICED_NVRAM_VSID_START + 100 + EMBEDDED_PROV_MAX_NODES)


//uint8_t cur_element_idx = 0;
//uint8_t cur_model_idx = 0;

/* This structure contains information sent from the provisioner application to provisioner library to setup local device */
typedef struct
{
    uint16_t addr;                      /**< Local Node Address */
    uint8_t  fil_key[2];               	/**< Local Device Key */
    uint8_t  net_key[2];				/**< Mesh Network Key */
} mesh_local_device_set_data_t;


typedef struct
{
//    uint8_t		uuid[16];
    uint16_t 	addr;
    uint16_t	last_addr;
    uint8_t		fil_key_node[2];
    uint8_t		net_key_node[2];
//    uint8_t  	num_elements;
//    uint8_t  	is_relay;          // the node is currently set as a relay
//    uint8_t  	dev_key[16];
//    int8_t   	rssi_table[EMBEDDED_PROV_MAX_NODES];
} mesh_node_t;

//typedef struct
//{
//    uint8_t  uuid[16];
//    uint16_t addr;
//    uint8_t  num_elements;
//    uint8_t  is_relay;          // the node is currently set as a relay
//    uint8_t  dev_key[16];
//    int8_t   rssi_table[EMBEDDED_PROV_MAX_NODES];
//} mesh_node_t;

mesh_node_t node;

void create_network(void);
void self_configure(uint16_t node_addr);

#endif /* SF_APP_BLE_APP_MESH_INIT_MESH_H_ */
