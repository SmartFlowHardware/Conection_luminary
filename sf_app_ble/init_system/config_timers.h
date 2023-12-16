/*
 * config_timers.h
 *
 *  Created on: 4 ago 2023
 *      Author: Lasec
 */

#ifndef SF_APP_BLE_INIT_SYSTEM_CONFIG_TIMERS_H_
#define SF_APP_BLE_INIT_SYSTEM_CONFIG_TIMERS_H_


/************************************************************************************************************************************
 *  Type  Definitions
 ***********************************************************************************************************************************/

#define clock_Online 			1000
#define clock_st_Online 		8000
#define clock_Online_long 		10000
#define clock_inspection   		20
#define clock_return5      		5
#define APP_TIMEOUT_IN_SECONDS	1
#define APP_TIMEOUT_MILISECONDS 500
#define TIMEOUT_IN_SECONDS		5



/************************************************************************************************************************************
 *  Global/Static Variables Definitions
 ***********************************************************************************************************************************/

/** Declare variable to start the Main Timer */
wiced_timer_t	app_main_timer;

/** Declare variable to start the Lamp Timer */
wiced_timer_t	lamp_timer;

/** Declare variable to start the Tag Timer */
wiced_timer_t	tag_timer;

/** Declare variable to start the Tag Timer */
wiced_timer_t	node_timer;

/** Declare variable to start blinking led Timer */
wiced_timer_t	bled_timer;

/** Declare variable to start blinking led Timer */
wiced_timer_t	timer_responce;

/** LED BLINK to show the response of the conection, when the provisioner save your mac  */
wiced_timer_t	timer_blink;

wiced_timer_t timer_Online;
wiced_timer_t timer_st_Online;
wiced_timer_t timer_inspection;
wiced_timer_t timer_return;
wiced_timer_t timer_sm;
wiced_timer_t timer_rOTA;
wiced_timer_t timer_SPI;
wiced_timer_t timer_da;
wiced_timer_t timer_clrspi;

wiced_timer_t timer_cback1;



/************************************************************************************************************************************
 *  Function Declarations
 ***********************************************************************************************************************************/

void config_clk_timers(void);
void start_BTimers(void);
void start_TOnline(void);
void start_TOnline_long(void);
void stop_TOnline(void);
void prevention_inspection(void);
void stop_timer_st_online(void);
void start_Treturn(void);
void start_Tsm(void);
void start_TOsm(void);
void start_trOTA_stop(void);
void start_TSPI(void);
void clear_da(void);
void clr_spi(void);

void			start_lamp_timer(void);
void			start_tag_timer(void);
void			stop_timers(void);
void 			start_timer(void);
void 			stop_timer_succes(void);
void 			start_blink(void);
void 			stop_blink(void);

void start_trOTA(uint32_t t_clk);



/************************************************************************************************************************************
 *	Imported Function Declarations
 ***********************************************************************************************************************************/

extern void		f_timer_Online( uint32_t data );
extern void		f_timer_st_Online( uint32_t data );
extern void		f_timer_inspection( uint32_t data );
extern void		f_timer_return( uint32_t data );
extern void		f_timer_sm( uint32_t data );
extern void		f_timer_rOTA( uint32_t data );
extern void		f_timer_SPI( void);
extern void		f_timer_da( void);
extern void		f_timer_clrspi( void);

extern void		f_timer_cback1( void );

extern void		f_app_main( TIMER_PARAM_TYPE arg );
extern void 	f_timer_lamp( TIMER_PARAM_TYPE arg );
extern void 	f_timer_tag( TIMER_PARAM_TYPE arg );
extern void 	f_timer_node( TIMER_PARAM_TYPE arg );
extern void		f_timer_bled( TIMER_PARAM_TYPE arg );
extern void     f_timer_succes_conection( TIMER_PARAM_TYPE arg );
extern void     f_timer_blink( TIMER_PARAM_TYPE arg );


#endif /* SF_APP_BLE_INIT_SYSTEM_CONFIG_TIMERS_H_ */

