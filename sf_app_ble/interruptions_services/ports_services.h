/*
 * ports_services.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_INTERRUPTIONS_SERVICES_PORTS_SERVICES_H_
#define SF_APP_BLE_INTERRUPTIONS_SERVICES_PORTS_SERVICES_H_

/************************************************************************************************************************************
 *  Variables Definitions
 ***********************************************************************************************************************************/

wiced_bool_t 	value_static=WICED_TRUE;
uint32_t      button_previous_value_acuse;
uint32_t      button_previous_value_onoff;


/************************************************************************************************************************************
 *  Imported Variables Definitions
 ***********************************************************************************************************************************/

extern uint16_t app_timer_count;
extern wiced_bool_t	is_provisioned;



/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

void			button_cback_26( void *data, uint8_t port_pin );
void			button_cback_4( void *data, uint8_t port_pin );
void			button_cback_0( void *data, uint8_t port_pin );

void			button_cback_acuse( void *data, uint8_t port_pin );
void			button_cback_on_off( void *data, uint8_t port_pin );



/************************************************************************************************************************************
 * 	Imported Function Declarations
 ***********************************************************************************************************************************/

extern void 	init_event_gap(void);
extern void 	init_event_btn(void);

extern void		create_network(void);

#endif /* SF_APP_BLE_INTERRUPTIONS_SERVICES_PORTS_SERVICES_H_ */

