/*
 * config_ports.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_INIT_SYSTEM_CONFIG_PORTS_H_
#define SF_APP_BLE_INIT_SYSTEM_CONFIG_PORTS_H_

/** Buttons for device */
#define PORT_INT0			WICED_P26
#define PORT_INT_ACUSE		WICED_P07
#define PORT_INT_ON_OFF		WICED_P10

/** LEDs for device */
#define LED_PIN_GREEN		WICED_P00
#define LED_PIN_BLUE		WICED_P01
#define LED_PIN_RED			WICED_P02
//#define LED_CHARGE			WICED_P26
#define LED_CONECTION		WICED_P28
#define LED_WARNING			WICED_P29
#define LED_VEHICLE			WICED_P34
#define LED_PERSON			WICED_P38


/** Pin state for when a button is pressed. **/
#ifndef BTN_PRESSED
#define BTN_PRESSED								(GPIO_PIN_OUTPUT_LOW)
#endif



void set_outPuts(void);
void set_inttPuts(void);

extern void button_cback_26( void *data, uint8_t port_pin );
extern void button_cback_4( void *data, uint8_t port_pin );
extern void button_cback_0( void *data, uint8_t port_pin );
extern void reset_values(void);

extern void button_cback_acuse( void *data, uint8_t port_pin );
extern void button_cback_on_off( void *data, uint8_t port_pin );


#endif /* SF_APP_BLE_INIT_SYSTEM_CONFIG_PORTS_H_ */

