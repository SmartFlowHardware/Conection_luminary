/*
 * mesh_definitions.h
 *
 *  Created on: 7 dic 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_APP_MESH_MESH_DEFINITIONS_H_
#define SF_APP_BLE_APP_MESH_MESH_DEFINITIONS_H_


typedef struct
{
//    uint8_t		uuid[16];
    uint8_t 	addr;					/**< Local Node Address inside of the Network */
    uint8_t		last_addr;				/**< Next Node Address inside of the Network */
    uint8_t		max_dst;				/**< Destiny Node hops inside of the Network */
    uint8_t		fil_key_node[2];		/**< Local Device Key */
    uint8_t		net_key_node[3];		/**< Mesh Network Key */
//    uint8_t  	num_elements;
//    uint8_t  	is_relay;          // the node is currently set as a relay
//    uint8_t  	dev_key[16];
//    int8_t   	rssi_table[EMBEDDED_PROV_MAX_NODES];
} mesh_node_t;


typedef struct
{
	uint8_t		addr;
	uint8_t		max_dst;
	uint8_t		net_key[3];
	uint8_t		message_conn[10];
} mesh_info_t;


typedef struct
{
	uint8_t		addr_inf;
} inf_data;


enum state_device
{
	NONE_DEV,		/**< None Device is found */
	LAMP_DEV,		/**< The Lamp device is found */
	TAG_DEV,		/**< The Tag device is found */
	BOTH_DEV,		/**< Both devices are found (Lamp and Tag) */
	SEVERAL_TAGS,	/**< There are many Tags on the Network */
};

enum advertisiemnt_package
{
	NODE_ADV,
	MESH_ADV,
	BEACON_INFO_MESH_UID_ADV,
};

//typedef struct
//{
//    uint8_t  uuid[16];
//    uint16_t addr;
//    uint8_t  num_elements;
//    uint8_t  is_relay;          // the node is currently set as a relay
//    uint8_t  dev_key[16];
//    int8_t   rssi_table[EMBEDDED_PROV_MAX_NODES];
//} mesh_node_t;

/* This structure contains information sent from the provisioner application to provisioner library to setup local device */
//typedef struct
//{
//    uint16_t addr;                      /**< Local Node Address */
//    uint8_t  fil_key[2];               	/**< Local Device Key */
//    uint8_t  net_key[2];				/**< Mesh Network Key */
//} mesh_local_device_set_data_t;


#endif /* SF_APP_BLE_APP_MESH_MESH_DEFINITIONS_H_ */
