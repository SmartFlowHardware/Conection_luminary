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
#include "init_mesh.h"


/*
 * Create network and self provision to this network.
 */
void create_network(void)
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	// Information of the Net and local device
	//mesh_local_device_set_data_t set;			// Structure contains information of the provisioner application
    //mesh_node_t node;							// Structure contains information of the node
    wiced_result_t result;						// Variable to save the result

    memset(&node, 0, sizeof(node));				// Clean the structure

    node.addr = EMBEDDED_PROV_LOCAL_ADDR;		// Set the address provisioner application (Is the First Node)
    node.last_addr = EMBEDDED_PROV_LOCAL_ADDR;

    // Create the Network Key
    *(uint8_t*)&node.fil_key_node[0] = wiced_hal_rand_gen_num();
    *(uint8_t*)&node.fil_key_node[1] = wiced_hal_rand_gen_num();

    // Create the Filter Key
    *(uint8_t*)&node.fil_key_node[0] = wiced_hal_rand_gen_num();
    *(uint8_t*)&node.fil_key_node[1] = wiced_hal_rand_gen_num();

    WICED_BT_TRACE("Create net | addr:%x\n", node.addr);

    // Save the information about the 'node' in the NVRAM
    if (wiced_hal_write_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
    {
        WICED_BT_TRACE("write node failed id:%d result:%d\n", EMBEDDED_PROV_NODE_ADDR_FIRST, result);
    }
    else
    {
    	WICED_BT_TRACE("write node sucesfull id:%d result:%d\n", EMBEDDED_PROV_NODE_ADDR_FIRST, result);

        // Reset the system using the watchdog timer will reset the system after setting "OTA_Mode_Enabled" to true
        wiced_hal_wdog_reset_system();
    }

    // Configure the local device with parameters defined in 'set' structure
    //wiced_bt_mesh_provision_local_device_set(&set);

    //node.addr = EMBEDDED_PROV_LOCAL_ADDR;						// Local Address for device
    //node.num_elements = mesh_config.elements_num;				// Number of element

    //memcpy(node.dev_key, set.dev_key, sizeof(node.dev_key));	// Device Key

    // Configure the key Application (Security and Net Communication)
    //configure_app_key_add(node.addr, EMBEDDED_PROV_NET_KEY_IDX, app_key, EMBEDDED_PROV_APP_KEY_IDX);

//#if 1
//    self_configure(EMBEDDED_PROV_LOCAL_ADDR);
//#endif
}




///*
// * Create network and self provision to this network.
// */
//void create_network(void)
//{
//	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
//
//	// Information of the Net and local device
//    wiced_bt_mesh_local_device_set_data_t set;	// Structure contains information of the provisioner application
//    mesh_node_t node;							// Structure contains information of the node
//    wiced_result_t result;						// Variable to save the result
//    uint8_t app_key[16];
//
//    memset(&set, 0, sizeof(set));				// Clean the structure
//
//    set.addr = EMBEDDED_PROV_LOCAL_ADDR;		// Set the address provisioner application
//
//    // Create the Device Key
//    *(uint32_t*)&set.dev_key[0] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&set.dev_key[4] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&set.dev_key[8] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&set.dev_key[12] = wiced_hal_rand_gen_num();
//
//    // Create the Network Key
//    *(uint32_t*)&set.network_key[0] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&set.network_key[4] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&set.network_key[8] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&set.network_key[12] = wiced_hal_rand_gen_num();
//
//    set.net_key_idx = EMBEDDED_PROV_NET_KEY_IDX;	// Index for the Network KeY
//
//    // Create the Application Key
//    *(uint32_t*)&app_key[0] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&app_key[4] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&app_key[8] = wiced_hal_rand_gen_num();
//    *(uint32_t*)&app_key[12] = wiced_hal_rand_gen_num();
//
//    WICED_BT_TRACE("-create net addr:%x net_key_idx:%x iv_idx:%x key_refresh:%d iv_upd:%d model_access:%d-\n", set.addr, set.net_key_idx, set.iv_idx, set.key_refresh, set.iv_update);
//
//    // Configure the local device with parameters defined in 'set' structure
//    wiced_bt_mesh_provision_local_device_set(&set);
//
//    node.addr = EMBEDDED_PROV_LOCAL_ADDR;						// Local Address for device
//    //node.num_elements = mesh_config.elements_num;				// Number of element
//
//    memcpy(node.dev_key, set.dev_key, sizeof(node.dev_key));	// Device Key
//
//    // Save the information about the 'node' in the NVRAM
//    if (wiced_hal_write_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
//    {
//        WICED_BT_TRACE("write node failed id:%d result:%d\n", EMBEDDED_PROV_NODE_ADDR_FIRST, result);
//    }
//
//    // Configure the key Application (Security and Net Communication)
//    //configure_app_key_add(node.addr, EMBEDDED_PROV_NET_KEY_IDX, app_key, EMBEDDED_PROV_APP_KEY_IDX);
//
//#if 1
//    self_configure(EMBEDDED_PROV_LOCAL_ADDR);
//#endif
//}
//
//
///*
// * Self provision to this network.
// */
//void self_configure(uint16_t node_addr)
//{
//	// Star with the self configure of the node in the Bluetooth Mesh
//	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);
//
//    cur_model_idx = 0;
//    cur_element_idx = 0;
//
//    // Initialize a timer to program and execute the next step in a after moment
//    wiced_init_timer(&self_config_timer, self_configure_next_op, (TIMER_PARAM_TYPE)node_addr, WICED_MILLI_SECONDS_TIMER);
//    self_configure_next_op((TIMER_PARAM_TYPE)node_addr);	// Call at the function for start the self configuration process
//}



