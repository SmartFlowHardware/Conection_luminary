/************************************************************************************************************************************
 * File: config_ports.c
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Description:
 *	This source file focuses on configuring GPIO pins for both input and output purposes, setting initial states, and enabling
 *	interrupts. These functionalities are crucial for interactions involving buttons, LEDs, and other devices connected to the GPIO
 *	pins of a Bluetooth-enabled device.
 *
 *
 * Author:
 *   Your Name
 *
 * Date:
 *   4 ago 2023
 *
 * Version:
 *   Version number of the code
 ***********************************************************************************************************************************/

#include "wiced_bt_dev.h"
#include "sparcommon.h"

#include "wiced_hal_gpio.h"
#include "wiced_hal_mia.h"
#include "wiced_gki.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "config_ports.h"



/************************************************************************************************************************************
 *  												Function Definitions
 ***********************************************************************************************************************************/

/************************************************************************************************************************************
 * Function Name: set_outPuts(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Set the pins of the device like Outputs with the function "wiced_hal_gpio_configure_pin()" the parameter that receives the function
 * 	are the follows:
 * 		wiced_hal_gpio_configure_pin(PIN, INPUT/OUTPUT, HIGH/LOW);
 ***********************************************************************************************************************************/
void set_outPuts( void )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

	/* Configure LED Luminary PIN as input (all PINs are Turn Off) */
	//wiced_hal_gpio_configure_pin(LED_PIN_GREEN, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);  descomentar
	wiced_hal_gpio_configure_pin(LED_PIN_BLUE, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
	wiced_hal_gpio_configure_pin(LED_PIN_RED, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);

	/* Configure LED Indication PIN as input (all PINs are Turn Off) */
	//wiced_hal_gpio_configure_pin(LED_CHARGE, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
	//wiced_hal_gpio_configure_pin(LED_CONECTION, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
	wiced_hal_gpio_configure_pin(LED_WARNING, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
	wiced_hal_gpio_configure_pin(LED_VEHICLE, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
	wiced_hal_gpio_configure_pin(LED_PERSON, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);

	/* CYW920819EVB-02 */
	wiced_hal_gpio_configure_pin(LED1, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
	wiced_hal_gpio_configure_pin(LED2, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
}


/************************************************************************************************************************************
 * Function Name: set_intPuts(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	Set the pins of the device like InPuts with the function "wiced_hal_gpio_configure_pin()" the parameter that receives the function
 * 	are the follows:
 * 		wiced_hal_gpio_configure_pin(PIN, CONFIGURATION, HIGH/LOW);
 ***********************************************************************************************************************************/
void set_intPuts( void )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

    /* Activate the PIN from Button */
	//wiced_hal_gpio_configure_pin(PORT_INT0, (GPIO_INPUT_ENABLE|GPIO_PULL_UP_DOWN_NONE|GPIO_EN_INT_BOTH_EDGE), GPIO_PIN_OUTPUT_HIGH);

	wiced_hal_gpio_configure_pin(PORT_INT_ACUSE, (GPIO_INPUT_ENABLE|GPIO_PULL_UP_DOWN_NONE|GPIO_EN_INT_BOTH_EDGE), GPIO_PIN_OUTPUT_HIGH);
	wiced_hal_gpio_configure_pin(PORT_INT_ON_OFF, (GPIO_INPUT_ENABLE|GPIO_PULL_UP_DOWN_NONE|GPIO_EN_INT_BOTH_EDGE), GPIO_PIN_OUTPUT_HIGH);

	//wiced_hal_gpio_configure_pin(PORT_INT_ACUSE, (GPIO_INPUT_ENABLE|GPIO_PULL_UP|GPIO_EN_INT_BOTH_EDGE), GPIO_PIN_OUTPUT_HIGH);  // CYW920819EVB-02
}


/************************************************************************************************************************************
 * Function Name: register_pin_interrupt(void)
 * ----------------------------------------------------------------------------------------------------------------------------------
 * Summary:
 * 	GPIO interrupts are enabled or disabled during pin configuration. For pins with interrupts enabled, the interrupt callback function
 * 	(i.e. interrupt service routine or interrupt handler) is registered using wiced_hal_gpio_register_pin_for_interrupt. Enable
 * 	interrupts that depends at a button for example are the follows:
 * 		wiced_hal_gpio_register_pin_for_interrupt( PIN, Function, user_data(optional) );
 ***********************************************************************************************************************************/
void register_pin_interrupt( void )
{
	WICED_BT_TRACE("[%s]\r\n", __FUNCTION__);

//	wiced_hal_gpio_register_pin_for_interrupt(PORT_INT_ACUSE, button_cback_acuse, NULL);  /* Aqui respuesta de conexion */
	wiced_hal_gpio_register_pin_for_interrupt(PORT_INT_ACUSE, button_cback_acuse, NULL);  /* Aqui respuesta de conexion */
	wiced_hal_gpio_register_pin_for_interrupt(PORT_INT_ON_OFF, button_cback_on_off, NULL);
}
