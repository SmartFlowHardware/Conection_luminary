/*
 * init_mesh.c
 *
 *  Created on: 28 nov 2023
 *      Author: Lasec
 */


#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_rand.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_model_utils.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_hal_wdog.h"
#include <stdlib.h>
#include <stdio.h>
#include "malloc.h"
#include "mesh_definitions.h"
#include "init_mesh.h"
#include <string.h>


void prepare_network_info(const mesh_node_t *node, uint8_t *inf_network)
{
    if (node == NULL || inf_network == NULL)
    {
        // Handle errors or debug messages
        return;
    }

    // Keep the information of the Network ready to send
    inf_network[0] = 0; // Something
    inf_network[1] = 0; // Something
    inf_network[2] = node->last_addr;
    inf_network[3] = node->max_dst;
    memcpy(inf_network + 4, node->net_key_node, sizeof(node->net_key_node));
}


/*
 * Create network and self provision to this network.
 */
void create_network(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	wiced_result_t result;						// Variable to save the result

	// If the Mesh has been created or node is provisioned return the function
    if(wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST, sizeof(node), (uint8_t*)&node, &result) == sizeof(node))
    {
    	// Node provisioned
    	WICED_BT_TRACE("Device are provisioned\r\n");
    	return;
    }

	// Information of the Net and local device
	//mesh_local_device_set_data_t set;			// Structure contains information of the provisioner application
    //mesh_node_t node;							// Structure contains information of the node

    memset(&node, 0, sizeof(node));				// Clean the structure

    node.addr = EMBEDDED_PROV_LOCAL_ADDR;			// Set the address provisioner application (Is the First Node)
    node.last_addr = EMBEDDED_PROV_LOCAL_ADDR + 1;	// Set the last address provisioner application (Is the Next Node)
    node.max_dst = EMBEDDED_PROV_HOPS;				// Set the hops through the Network to destiny, default is 2

    // Create the Network Key
    node.net_key_node[0] = 0x4e;//wiced_hal_rand_gen_num();
    node.net_key_node[1] = 0x45;//wiced_hal_rand_gen_num();
    node.net_key_node[2] = 0x54;//wiced_hal_rand_gen_num();

    // Create the Filter Key
    *(uint8_t*)&node.fil_key_node[0] = wiced_hal_rand_gen_num();
    *(uint8_t*)&node.fil_key_node[1] = wiced_hal_rand_gen_num();

    // Save the information about the 'node' in the NVRAM
    if (wiced_hal_write_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
    {
        WICED_BT_TRACE("write node failed id:%d result:%d\n", EMBEDDED_PROV_NODE_ADDR_FIRST, result);
    }
    else
    {
    	WICED_BT_TRACE("write node sucesfull id:%d result:%d\n", EMBEDDED_PROV_NODE_ADDR_FIRST, result);
    	WICED_BT_TRACE("Create net | addr:%x | NKey: %02X %02X %02X ", node.addr, node.net_key_node[0], node.net_key_node[1], node.net_key_node[2]);
    	WICED_BT_TRACE("| last addr:%x | max_dst:%x\n", node.last_addr, node.max_dst);

    	// Save the information about the Network to send
    	prepare_network_info(&node, inf_network);

    	// Change the advertisement
    	gap_rebroadcastLR(MESH_ADV, 0);  /*    ******* i add 0 */
    }
}


void copy_info_net(uint8_t *p_info_net)
{
	WICED_BT_TRACE_ARRAY(p_info_net, 14, "Info Net: ");

	// Save the information on a structure
	info_mesh.addr    = p_info_net[7];
	WICED_BT_TRACE("\n 1 Addres%d \n",info_mesh.addr);
	info_mesh.max_dst = EMBEDDED_PROV_HOPS;
	WICED_BT_TRACE("\n 2 Max_dist%d \n",info_mesh.max_dst);
	memcpy(info_mesh.net_key, p_info_net + 4, 3);
	WICED_BT_TRACE("\n KEY %B \n",info_mesh.net_key);

	// Filter to ask to connect
	info_mesh.message_conn[0] = ' ';	// Space empty
	info_mesh.message_conn[1] = 'C';	// C - Filter to ask to join at the Network
	info_mesh.message_conn[2] = 'N';	// N - Filter to ask to join at the Network
	info_mesh.message_conn[3] = info_mesh.addr; // Asigned the number of the node

	WICED_BT_TRACE("Info Net | addr:%x | max_dst:%x | NKey: %02X %02X %02X\r\n", info_mesh.addr, info_mesh.max_dst, info_mesh.net_key[0], info_mesh.net_key[1], info_mesh.net_key[2]);
//	WICED_BT_TRACE_ARRAY(info_mesh.message_conn, 8, "Message Connection: ");

	// Change the advertisement  /* Poneer hasta que se precione el boton */
	//gap_rebroadcastLR(NODE_ADV,info_mesh.addr);
}



/*
 * Mesh device is being factory reset. Clean up NVRAM used by the app.
 */
void mesh_app_factory_reset(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    int i;
    wiced_result_t result;

    // if it was a factory reset, make sure to clean up NVRAM (information about nodes)
    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        wiced_hal_delete_nvram(i, &result);
    }

    // Change the advertisement
    gap_rebroadcastLR(0,0);         /******* i add 0 */
}


//wiced_bool_t save_message_data(wiced_bt_ble_scan_results_t *p_scan_result)
//{
//	// Pointer to bluetooth device address
//	uint8_t				*p_mesh_adds;
//	p_mesh_adds = p_scan_result->remote_bd_addr;	// Assign the pointer to the variable of bluetooth device address
//
//	// Copy the bytes from the array pointer
//	for(int = 0; i< BBD_ADDR_LEN; i++)
//	{
//
//	}
//
//}



uint8_t generate_random_number(void)
{
	uint8_t random_value; // Variable to generate the random value

	// Loop to generate a Random Number that not be zero
    do
    {
    	random_value = wiced_hal_rand_gen_num();
    }while (random_value == 0);

    return random_value;
}



char* transmit_node_data(mesh_node_t node, char* user_prefix)
{
    // Get the value for number N
    uint8_t n_value = (node.addr % 100);

    // Calculate the total length of the resulting string
    // (prefix length + space + length of "Nxx")
    size_t total_length = strlen(user_prefix) + 1 + 4;

    // Allocates memory for the resulting string
    char* result = (char*)malloc(total_length);

    // Error Handling: Could not allocate memory
    if (result == NULL)
        return NULL;

    // Format the string to your specifications
    snprintf(result, total_length, "%s N%02d", user_prefix, n_value);

    // Returns the generated string
    return result;
}

/* Procces od conetion */
base_data data_base[SIZE];

void Conect_process1(wiced_bt_ble_scan_results_t *p_scan_result)
{

	static uint8_t addr1 = 2;
	beacon_set_eddystone_uid_advertisement_data_1(addr1, 0);
}
//	/* ************** Primer saber si ya tengo guardada la mac que he visto *************** */
//	safe_macs=0;
//	for(int i=0; i<SIZE;i++)
//	{
//		if(memcmp(data_base[i].bdaddr_luminary,p_scan_result->remote_bd_addr,6)==0 && p_scan_result->remote_bd_addr[0] != 0 && p_scan_result->remote_bd_addr[1] != 0 && p_scan_result->remote_bd_addr[2] != 0)     /* Primero saber si ya tengo la mac, si la tengo renuevo su RSSI y si su RSSI ha aumentado, si lo ha hecho  */
//		{
//			data_base[i].rssi= p_scan_result->rssi;   //Actualizo su vaor de RSSI
//			safe_macs=1;  /* flag to know if I already have this mac saved */
//		}
//	}
//	/* ********************************************************************************* */
//
//	/* ******************** Si no tengo la mac acomodar en un espacio disponible******** */
//	if(safe_macs == 0)	 /* Nunca entro, no tengo esta mac guardada en mi arreglo, ademas voy a ver si ya estoy lleno */
//	{
//			for(int i=0; i<SIZE;i++)
//			{
//				if(array_mac[i] == 0)  /* Si vale cero, aqui guardare la nueva mac  */
//				{
//					memcpy(data_base[i].bdaddr_luminary,p_scan_result->remote_bd_addr,6);
//					data_base[i].rssi= p_scan_result->rssi;
//					array_mac[i]=1;                                             /* Mac guaedada en la posicion i */
//					data_base[i].conection_status = 0;  /* init process conection */
//					break;
//				}
//			}
//	}
//
//
//	for(int i=0; i<SIZE;i++)   // 0 -5 0 -2 -3    quitadno espacios solos
//	{
//		if(array_mac[i] == 0 && array_mac[i+1] !=0)
//		{
//			memcpy(data_base[i].bdaddr_luminary,data_base[i+1].bdaddr_luminary,6);
//			memset(data_base[i+1].bdaddr_luminary,'\0',6); /* limpio */
//			data_base[i].rssi=data_base[i+1].rssi;
//			data_base[i+1].rssi=0;		                 /* limpio */
//
//			array_mac[i+1]=0;
//			array_mac[i]=1;                   /* En el actual ya tengo algo */
//		}
//	}
//	WICED_BT_TRACE("\n Macs vistas \n");  /* Esto no va necesariamente */
//	for(int i=0; i<SIZE;i++)
//	{
//		WICED_BT_TRACE("\n [%d] %B \n",i,data_base[i].bdaddr_luminary);   /* NODEL BSL */
//	}

	//static uint8_t addr1 = 2;

//	/* Inicio proceso de conexion, bandera para preguntar solo una vez una bandera */
//	for(int i= 0; i<SIZE;i++)
//	{	   /* Si hay algo */
//		if(array_mac[i]==1 && data_base[i].conection_status == 0 && status_flag==WICED_FALSE)   /* Tengo una MAC guardada en esta posicion, y ademas no tengo una conexion en ese lugar, nicia conexion */
//		{
//			status_flag=WICED_TRUE;  /* start advertisements ones */
//			data_base[i].addr=addr1;
//			beacon_set_eddystone_uid_advertisement_data_1(data_base[i].bdaddr_luminary, addr1);
//			addr1 ++;   /* Asigned addres */
//			/* Si se logra realizar la conexion conection_status valdra 1, si no se completa valdra 0 y despues de un rato se borrara para iniciar otra conección */
//			break;
//		}
//	}

//}
