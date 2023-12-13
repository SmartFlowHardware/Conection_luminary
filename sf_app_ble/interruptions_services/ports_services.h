/*
 * ports_services.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_INTERRUPTIONS_SERVICES_PORTS_SERVICES_H_
#define SF_APP_BLE_INTERRUPTIONS_SERVICES_PORTS_SERVICES_H_


#define TIMES_BLINKING_LED	5

/************************************************************************************************************************************
 *  Variables Definitions
 ***********************************************************************************************************************************/

wiced_bool_t 	value_static=WICED_TRUE;
uint32_t      button_previous_value_acuse;
uint32_t      button_previous_value_onoff;

wiced_bool_t	acuse_pressed;
wiced_bool_t	onoff_pressed;

/** Variable to freeze the time when the button is pressed */
uint16_t				btn_push_tm_acuse;
uint16_t				btn_push_tm_onoff;

uint32_t debounce_time_ms = 50; 					// Debounce time in milliseconds
uint32_t lst_btn_evt_time_acuse = 0;
uint32_t lst_btn_evt_time_onoff = 0;



/************************************************************************************************************************************
 *  Imported Variables Definitions
 ***********************************************************************************************************************************/

extern mesh_node_t node;
extern uint16_t app_timer_count;
extern uint8_t	app_timer_button;
extern wiced_bool_t	is_provisioned;
extern wiced_bool_t find_node;
extern wiced_bool_t	mode_send_info;
extern mesh_info_t info_mesh;
extern wiced_bool_t conn_node_mesh;


/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

void			button_cback_0( void *data, uint8_t port_pin );

void			button_cback_acuse( void *data, uint8_t port_pin );
void			button_cback_on_off( void *data, uint8_t port_pin );



/************************************************************************************************************************************
 * 	Imported Function Declarations
 ***********************************************************************************************************************************/

extern void 	init_event_gap(void);
extern void 	init_event_btn(void);

extern void		create_network(void);
extern void		copy_info_net(uint8_t *p_info_net);
extern void		mesh_app_factory_reset(void);

#endif /* SF_APP_BLE_INTERRUPTIONS_SERVICES_PORTS_SERVICES_H_ */

