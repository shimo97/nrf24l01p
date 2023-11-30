  
/*
 *  nrf24l01_plus.h
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 *
 */

/*
 *  Major modifications on 2023.05.28
 * 	Author: shimo97
 *
 *  The library has been almost completely rewritten (only some low level defines and
 *  functions has been preserved) to be simpler to use and be more compatible to an
 *  half duplex rather than simplex usage as it was intended before.
 *
 *  Other than that, now the library can support multiple transceivers on the same microcontroller.
 *
 *  Now the library will only pass through the recommended operating modes from the datasheet
 *  (Power Down, Standby-I, Rx and TX) rather than resorting to Standby-II mode like typically
 *  done for simplex-oriented libraries (Standby-II mode is easier to use for simplex transmission
 *  because it doesn't require a specific timing of CE pin but it doesn't give the possibility to pass
 *  directly on RX mode without a dummy transmission or a power down)
 *
 *  INSTRUCTIONS:
 *  To use the library, the user has to:
 *  -define the get_tick_us() function (i.e. with an hardware timer)
 *  -place the nrf24l01p_tx_irq() inside the EXTI interrupt handler of the IRQ pin
 *  -define the _weak functions nrf24l01p_tx_callback(), nrf24l01p_rx_callback() and nrf24l01p_max_callback()
 *  with the code to execute after correct transmission, reception and failed transmission, respectively
 *
 *  The transceiver can be set up to idle on RX mode or Standby(-I) mode and it will go on TX mode
 *  only when a transmission is requested, Standby(-I) idle mode is recommended for a simplex setup
 *  because have a much lower power consumption w.r.t. RX mode
 *  This way it can be used for both simplex or half-duplex communication.
 *
 *  TRANSCEIVER SETUP:
 *  All the user has to do to set up the transceiver is to pass an initialization struct nrf24l01p_init_struct
 *  to the nrf24l01p_add() function, this function will return an integer (the transceiver ID) thet identifies
 *  the transceiver and must be pssed to the other library functions
 *
 */

#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include "spi.h"
#include "tim.h"
#include <stdbool.h>
#include <stdint.h>

/* User Configurations */
#define NRF24L01P_MAX_XCVRS					2	//maximum number of transceivers the library can manage
#define SPI_TIMEOUT							10	//SPI transaction timeout (ms)

/* nRF24L01+ typedefs */
typedef enum
{
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
} air_data_rate;

typedef enum
{
	_3bytes = 1,
	_4bytes = 2,
	_5bytes = 3
} address_width;

typedef enum
{
    _0dBm  = 3,
    _6dBm  = 2,
    _12dBm = 1,
    _18dBm = 0
} output_power;

typedef enum{
	_powerdown,
	_standby,
	_rx,
	_tx
} xcvr_state;

//policy in case of full buffer (throw older or newer packet)
typedef enum
{
	throw_old,
	throw_new
} buffer_policy;

typedef struct {
	SPI_HandleTypeDef* 	hspi; //spi port the xcvr is connected to

	//gpio definitions
	GPIO_TypeDef* 		csn_port;
	uint16_t			csn_pin;
	GPIO_TypeDef* 		ce_port;
	uint16_t 			ce_pin;
	uint16_t 			irq_pin; 	//irq pin, this is a number that must be passed to nrf24l01p_tx_irq() on the external interrupt
									//isr, technically this can be an arbitrary "interrupt number" since it's only used to search for
									//the transceiver that triggered the interrupt and not necessarily the interrupt pin number
									//but the latter is the easier way to go
	IRQn_Type			irqn;		//NVIC interrupt number associated with the transceiver EXTI interrupt

    //transceiver functionality setting
    uint8_t        		rf_channel;				//rf channel (as MHz above 2400 MHz)
    uint8_t* 			address; 				//address (same for TX and RX),
    											//must have the right number of bytes as set on address_w
    address_width		address_w;				//address width (N.B. must be address_width enum type, not pure integer)
    air_data_rate  		data_rate;				//data rate (must be air_data_rate enum type)
    uint8_t		        retransmit_delay_250us;	//retransmit delay , the resulting delay is given by
    											// 250us + (250us * value) , must be <=15
    uint8_t        		retransmit_number;		//number of retransmits (must be <=15)
    output_power     	tx_power;				//trasmit power (must be output_power enum type)
    xcvr_state			idle_state;				//transceiver idle state, must be _rx or _standby,
    											//if this is set as _tx or _powerdown the idle state is set to _standby
} nrf24l01p_init_struct;

void funnyprint();

/* User callbacks, defined as _weak empty functions, to be replaced by user with its code
 * payload is a buffer of size equal to the nrf24l01p_init_struct's payload_l containing the packet
 * that was transmitted/received/lost
 * NB. those callbacks are called inside EXTI ISR so the user must implement its own locking mechanism
 * in order to share resources with the callbacks
 */
void nrf24l01p_tx_callback(uint32_t ID, uint8_t* payload,uint32_t len); 	//called when a packet has been correctly transmitted
void nrf24l01p_rx_callback(uint32_t ID, uint8_t* payload,uint32_t len);	//called when a packet has been received
void nrf24l01p_max_callback(uint32_t ID, uint8_t* payload,uint32_t len);	//called when a packet has been transmitted with no ack received (maximum retry reached)

//function that must be placed inside EXTI interrupt handler, the pin number that
//caused the interrupt must be passed
void nrf24l01p_irq(uint16_t irq_pin);

/* Main Functions */
//adds a new transceiver to the library
//returns NRF24L01P_MAX_XCVRS in case of error(too much xcvrs added), otherwise returns the added transceiver ID
uint32_t nrf24l01p_add(nrf24l01p_init_struct* nrf_struct);

void nrf24l01p_powerup(uint32_t ID);
void nrf24l01p_powerdown(uint32_t ID);

xcvr_state nrf24l01p_getState(uint32_t ID); //returns current transceiver state

//start transmission of a new packet,  the payload buffer must have at least the number of element as passed
//to nrf24l01p_add as payload_l field, or can be a NULL pointer, in this case no new payload will be loaded
//this can be used in case of MAX_RT event (max retransmissions with no ack) because the transceiver holds
//the failed packet and will resend it with the same PID (the receiver will discard it if it was received and
//was instead the acknowledge that was lost)
//ack flag to 1 will request ack (and eventually retransmit depending on settings, otherwise no ack is requested)
//returns 0 in case of success, 0 otherwise
uint8_t nrf24l01p_transmit(uint32_t ID, uint8_t* payload,uint32_t len, uint8_t ack);

//ISR enable/disable functions for communication with callbacks
//stops ISR for a given xcvr and returns the ISR state before stopping
uint32_t nrf24l01p_stopAndGetIRQ(uint32_t ID);
//sets ISR to the state given as argument, you must save the return state when using nrf24l01p_stopAndGetISR() and
//pass it when calling this function to avoid problems due to nested functions enabling ISR when not wanted
void nrf24l01p_setIRQ(uint32_t ID,uint32_t state);

#endif /* __NRF24L01P_H__ */
