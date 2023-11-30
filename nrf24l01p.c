/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 *
 *  Major modifications on 2023.05.28
 * 	Author: shimo97
 */


#include "nrf24l01p.h"
#include <string.h>
//#include <stdio.h> //only used if debug printfs are used

//bitwise set reset macros
#define NRF24L01P_SET_BIT(byte,mask) 		byte=byte|(mask)
#define NRF24L01P_RESET_BIT(byte,mask) 		byte=byte&(~(mask))

/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER                  0b00000000
#define NRF24L01P_CMD_W_REGISTER                  0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD                0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
#define NRF24L01P_CMD_FLUSH_TX                    0b11100001
#define NRF24L01P_CMD_FLUSH_RX                    0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL                 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID                 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD               0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK          0b10110000
#define NRF24L01P_CMD_NOP                         0b11111111

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08    // Read-Only
#define NRF24L01P_REG_RPD               0x09    // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D

/* Registers bit and multi bit masks definitions (only for 1 byte registers)
 * Single bit fields are defined with the same name
 * of the datasheet, while multi bit values are defined
 * with a macro function that masks and shifts the numerical value
 * passed into position (this is also true for data rate field regardless of
 * the fact that in datasheet it is divided into RF_DR_LOW and RF_DR_HIGH
 * single bits, the macro in this case is named RF_DR() )
 * */
#define NRF24L01P_BIT_MASK_RX_DR		0x40
#define NRF24L01P_BIT_MASK_TX_DS		0x20
#define NRF24L01P_BIT_MASK_MAX_RT		0x10
#define NRF24L01P_BIT_EN_CRC			0x08
#define NRF24L01P_BIT_CRCO				0x04
#define NRF24L01P_BIT_PWR_UP			0x02
#define NRF24L01P_BIT_PRIM_RX			0x01
#define NRF24L01P_BIT_ENAA_P5			0x20
#define NRF24L01P_BIT_ENAA_P4			0x10
#define NRF24L01P_BIT_ENAA_P3			0x08
#define NRF24L01P_BIT_ENAA_P2			0x04
#define NRF24L01P_BIT_ENAA_P1			0x02
#define NRF24L01P_BIT_ENAA_P0			0x01
#define NRF24L01P_BIT_ERX_P5			0x20
#define NRF24L01P_BIT_ERX_P4			0x10
#define NRF24L01P_BIT_ERX_P3			0x08
#define NRF24L01P_BIT_ERX_P2			0x04
#define NRF24L01P_BIT_ERX_P1			0x02
#define NRF24L01P_BIT_ERX_P0			0x01
#define NRF24L01P_BIT_AW(val) 			((val) & 0x03)
#define NRF24L01P_BIT_ARD(val)			(((val) & 0xF0)<<4)
#define NRF24L01P_BIT_ARC(val)			((val) & 0x0F)
#define NRF24L01P_BIT_RF_CH(val)		((val) & 0x7F)
#define NRF24L01P_BIT_CONT_WAVE			0x80
#define NRF24L01P_BIT_RF_DR(val)		((((val) & 2)<<5) | ((val & 1)<<3)) //this places both RF_DR_LOW and RF_DR_HIGH bits
#define NRF24L01P_BIT_PLL_LOCK			0x10
#define NRF24L01P_BIT_RF_PWR(val)		(((val) & 0x06)<<1)
#define NRF24L01P_BIT_RX_DR				0x40
#define NRF24L01P_BIT_TX_DS				0x20
#define NRF24L01P_BIT_MAX_RT			0x10
#define NRF24L01P_BIT_RX_P_NO(val)		(((val) & 0x0E)<<1)
#define NRF24L01P_BIT_TX_FULL_STATUS	0x01	//TX_FULL bit inside STATUS register
#define NRF24L01P_BIT_PLOS_CNT(val)		(((val) & 0xF0)<<4)
#define NRF24L01P_BIT_ARC_CNT(val)		((val) & 0x0F)
#define NRF24L01P_BIT_RPD				0X01
#define NRF24L01P_BIT_RX_PW_P0(val)		((val) & 0x7F)
#define NRF24L01P_BIT_RX_PW_P1(val)		((val) & 0x7F)
#define NRF24L01P_BIT_RX_PW_P2(val)		((val) & 0x7F)
#define NRF24L01P_BIT_RX_PW_P3(val)		((val) & 0x7F)
#define NRF24L01P_BIT_RX_PW_P4(val)		((val) & 0x7F)
#define NRF24L01P_BIT_RX_PW_P5(val)		((val) & 0x7F)
#define NRF24L01P_BIT_TX_REUSE			0x40
#define NRF24L01P_BIT_TX_FULL_FIFO		0x20	//TX_FULL bit inside FIFO_STATUS register
#define NRF24L01P_BIT_TX_EMPTY			0x10
#define NRF24L01P_BIT_RX_FULL			0x02
#define NRF24L01P_BIT_RX_EMPTY			0x01
#define NRF24L01P_BIT_DPL_P5			0x20
#define NRF24L01P_BIT_DPL_P4			0x10
#define NRF24L01P_BIT_DPL_P3			0x08
#define NRF24L01P_BIT_DPL_P2			0x04
#define NRF24L01P_BIT_DPL_P1			0x02
#define NRF24L01P_BIT_DPL_P0			0x01
#define NRF24L01P_BIT_EN_DPL			0x04
#define NRF24L01P_BIT_EN_ACK_PAY		0x02
#define NRF24L01P_BIT_EN_DYN_ACK		0x01

//BEGIN TRANSCEIVERS MANAGEMENT STRUCTURES AND BUFFERS

#define NRF24L01P_ADDRESS_WITH_TO_INT(val)	((val)+2) //conversion from address_width enum type to int


typedef struct{
	nrf24l01p_init_struct nrf_struct;	//transceiver handle

	volatile xcvr_state state;			//transceiver state
	uint8_t txBuffer[32]; 				//transmitted payload
	uint8_t txLen;						//length of tx payload
	uint8_t rxBuffer[32]; 				//received payload
	uint8_t rxLen;						//length of rx payload

} xcvr;
xcvr xcvrs_buff[NRF24L01P_MAX_XCVRS]; //transceivers buffers
uint32_t xcvrs_num=0;	//number of added transceivers

//PRIVATE FUNCTIONS

void cs_high(nrf24l01p_init_struct* nrf_struct)
{
    HAL_GPIO_WritePin(nrf_struct->csn_port,nrf_struct->csn_pin, GPIO_PIN_SET);
}

void cs_low(nrf24l01p_init_struct* nrf_struct)
{
    HAL_GPIO_WritePin(nrf_struct->csn_port,nrf_struct->csn_pin, GPIO_PIN_RESET);
}

void ce_high(nrf24l01p_init_struct* nrf_struct)
{
    HAL_GPIO_WritePin(nrf_struct->ce_port,nrf_struct->ce_pin, GPIO_PIN_SET);
}

void ce_low(nrf24l01p_init_struct* nrf_struct)
{
    HAL_GPIO_WritePin(nrf_struct->ce_port,nrf_struct->ce_pin, GPIO_PIN_RESET);
}

//function to send a command, returns the status register content
uint8_t send_command(nrf24l01p_init_struct* nrf_struct, uint8_t command){
	uint8_t status;
	cs_low(nrf_struct);
	HAL_SPI_TransmitReceive(nrf_struct->hspi, &command, &status, 1, SPI_TIMEOUT);
	cs_high(nrf_struct);

	return status;
}

//function to perform a write transaction (sending a command first and then  sending len bytes of data)
uint8_t write_transaction(nrf24l01p_init_struct* nrf_struct, uint8_t command, uint8_t* data, uint32_t len){
	uint8_t status;
	cs_low(nrf_struct);
	HAL_SPI_TransmitReceive(nrf_struct->hspi, &command, &status, 1, SPI_TIMEOUT);
	HAL_SPI_Transmit(nrf_struct->hspi, data, len, SPI_TIMEOUT);
	cs_high(nrf_struct);

	return status;
}

//function to perform a read transaction (sending a command first and then receiving len bytes of data)
uint8_t read_transaction(nrf24l01p_init_struct* nrf_struct, uint8_t command, uint8_t* data, uint32_t len){
	uint8_t status;
	cs_low(nrf_struct);
	HAL_SPI_TransmitReceive(nrf_struct->hspi, &command, &status, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(nrf_struct->hspi, data, len, SPI_TIMEOUT);
	cs_high(nrf_struct);

	return status;
}

//reads 8 bit register content
uint8_t read_register(nrf24l01p_init_struct* nrf_struct, uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t read_val;

    read_transaction(nrf_struct, command, &read_val, 1);

    return read_val;
}

//writes new content inside 8 bit register
uint8_t write_register(nrf24l01p_init_struct* nrf_struct,uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t write_val = value;

    write_transaction(nrf_struct, command, &write_val, 1);

    return write_val;
}

//reads content from multiple bytes register, value is where data will be written and must have size len
uint8_t * read_multibyte_register(nrf24l01p_init_struct* nrf_struct, uint8_t reg, uint8_t* value, uint8_t len){
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;

    read_transaction(nrf_struct, command, value, len);

    return value;
}

//writes new content inside multiple bytes register, value must be a buffer of size len
uint8_t * write_multibyte_register(nrf24l01p_init_struct* nrf_struct, uint8_t reg, uint8_t* value, uint8_t len){
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;

    write_transaction(nrf_struct, command, value, len);

    return value;
}


//sets the value of a specific register bit (or group of bits) given by 1s on the "bit" argument
uint8_t set_register_bit(nrf24l01p_init_struct* nrf_struct, uint8_t reg, uint8_t bit)
{
	uint8_t content=read_register(nrf_struct,reg);
	NRF24L01P_SET_BIT(content,bit);
	return write_register(nrf_struct,reg,content);
}

//resets the value of a specific register bit (or group of bits) given by 1s on the "bit" argument
uint8_t reset_register_bit(nrf24l01p_init_struct* nrf_struct, uint8_t reg, uint8_t bit)
{
	uint8_t content=read_register(nrf_struct,reg);
	NRF24L01P_RESET_BIT(content,bit);
	return write_register(nrf_struct,reg,content);
}

//replaces the bits of a specific register, meaning that the bits given by the mask (the
//ones that are 1 on the mask) are replaced by the corresponding bits in the "bit" argument
//no matter if they're 0s or 1s, this is useful for writing groups of bits for multiple-bit field
uint8_t replace_register_bit(nrf24l01p_init_struct* nrf_struct, uint8_t reg, uint8_t bit, uint8_t mask){
	uint8_t content=read_register(nrf_struct,reg);
	NRF24L01P_RESET_BIT(content,mask);
	NRF24L01P_SET_BIT(content,mask & bit);
	return write_register(nrf_struct,reg,content);
}

uint32_t nrf24l01p_stopAndGetIRQ(uint32_t ID){
	if(ID>=xcvrs_num) return 0;
	else{
		uint32_t state=NVIC_GetEnableIRQ(xcvrs_buff[ID].nrf_struct.irqn);
		NVIC_DisableIRQ(xcvrs_buff[ID].nrf_struct.irqn);
		return state;
	}
}

void nrf24l01p_setIRQ(uint32_t ID,uint32_t state){
	if(ID>=xcvrs_num) return;
	else{
		if(state){
			NVIC_EnableIRQ(xcvrs_buff[ID].nrf_struct.irqn);
		}
	}
	return;
}

void nrf24l01p_init(nrf24l01p_init_struct* nrf_struct)
{
	//setting address width
	write_register(nrf_struct, NRF24L01P_REG_SETUP_AW, NRF24L01P_BIT_AW(nrf_struct->address_w));
	//setting retransmit delay and count
	write_register(nrf_struct, NRF24L01P_REG_SETUP_RETR, NRF24L01P_BIT_ARD(nrf_struct->retransmit_delay_250us) | NRF24L01P_BIT_ARC(nrf_struct->retransmit_number));
	//setting RF channel
	write_register(nrf_struct, NRF24L01P_REG_RF_CH, NRF24L01P_BIT_RF_CH(nrf_struct->rf_channel));
	//setting RF data rate
	replace_register_bit(nrf_struct, NRF24L01P_REG_RF_SETUP, NRF24L01P_BIT_RF_DR(nrf_struct->data_rate) | NRF24L01P_BIT_RF_PWR(nrf_struct->tx_power), NRF24L01P_BIT_RF_DR(0xFF) | NRF24L01P_BIT_RF_PWR(0xFF));
    //setting address
	write_multibyte_register(nrf_struct, NRF24L01P_REG_RX_ADDR_P0, nrf_struct->address, NRF24L01P_ADDRESS_WITH_TO_INT(nrf_struct->address_w));
	write_multibyte_register(nrf_struct, NRF24L01P_REG_TX_ADDR, nrf_struct->address, NRF24L01P_ADDRESS_WITH_TO_INT(nrf_struct->address_w));

	return;
}

void nrf24l01p_reset(nrf24l01p_init_struct* nrf_struct)
{
    // Reset pins
    cs_high(nrf_struct);
    ce_low(nrf_struct);

    // registers default state
    // Flushing fifos
    send_command(nrf_struct, NRF24L01P_CMD_FLUSH_TX);
    send_command(nrf_struct, NRF24L01P_CMD_FLUSH_RX);
    //2 bytes CRC, set state as PRX, xcvr powered off
    write_register(nrf_struct,NRF24L01P_REG_CONFIG, NRF24L01P_BIT_EN_CRC | NRF24L01P_BIT_CRCO | NRF24L01P_BIT_PRIM_RX);
    //enable autoacknowledge on pipe 0
    write_register(nrf_struct,NRF24L01P_REG_EN_AA, NRF24L01P_BIT_ENAA_P0);
    //enable pipe 0
    write_register(nrf_struct,NRF24L01P_REG_EN_RXADDR, NRF24L01P_BIT_ERX_P0);
    //250us retransmit time, 3 retries
    write_register(nrf_struct,NRF24L01P_REG_SETUP_RETR, NRF24L01P_BIT_ARD(0) | NRF24L01P_BIT_ARC(3));
    //setting frequency to 2400 MHz
    write_register(nrf_struct,NRF24L01P_REG_RF_CH, NRF24L01P_BIT_RF_CH(0));
    //1Mbps data rate
    write_register(nrf_struct,NRF24L01P_REG_RF_SETUP, NRF24L01P_BIT_RF_DR(_1Mbps) | NRF24L01P_BIT_RF_PWR(_18dBm));
    //clearing all interrupt flags
    write_register(nrf_struct,NRF24L01P_REG_STATUS, NRF24L01P_BIT_RX_DR | NRF24L01P_BIT_TX_DS | NRF24L01P_BIT_MAX_RT);
    //setting rx pipe 0 payload to maximum payload length
    write_register(nrf_struct,NRF24L01P_REG_RX_PW_P0, NRF24L01P_BIT_RX_PW_P0(32));
    //setting other pipes as not used
    write_register(nrf_struct,NRF24L01P_REG_RX_PW_P1, 0);
    write_register(nrf_struct,NRF24L01P_REG_RX_PW_P2, 0);
    write_register(nrf_struct,NRF24L01P_REG_RX_PW_P3, 0);
    write_register(nrf_struct,NRF24L01P_REG_RX_PW_P4, 0);
    write_register(nrf_struct,NRF24L01P_REG_RX_PW_P5, 0);
    //no dynamic payload on pipe 0
    write_register(nrf_struct,NRF24L01P_REG_DYNPD, NRF24L01P_BIT_DPL_P0);
    //no additional features
    write_register(nrf_struct,NRF24L01P_REG_FEATURE, NRF24L01P_BIT_EN_DYN_ACK | NRF24L01P_BIT_EN_DPL);

}

//PUBLIC FUNCTIONS
uint32_t nrf24l01p_add(nrf24l01p_init_struct* nrf_struct)
{
	if(xcvrs_num>=NRF24L01P_MAX_XCVRS) return NRF24L01P_MAX_XCVRS;

	while(HAL_GetTick()<100);	//waiting for power on reset time to elapse

	uint32_t irqState=NVIC_GetEnableIRQ(nrf_struct->irqn);
	NVIC_DisableIRQ(nrf_struct->irqn);

	memcpy(&xcvrs_buff[xcvrs_num].nrf_struct,nrf_struct,sizeof(nrf24l01p_init_struct));
	xcvrs_buff[xcvrs_num].state=_powerdown;
	xcvrs_num++;

	nrf24l01p_reset(nrf_struct); //reset all transceiver registers to default configurations and power it off

	nrf24l01p_init(nrf_struct);	//init the transceiver with the informations passed with the handle structure


	uint8_t config=read_register(nrf_struct, NRF24L01P_REG_CONFIG);
	uint8_t feature=read_register(nrf_struct, NRF24L01P_REG_FEATURE);
	uint8_t status=send_command(nrf_struct, NRF24L01P_CMD_NOP);
	uint8_t dynpld=read_register(nrf_struct, NRF24L01P_REG_DYNPD);
	printf("#config %x status %x feature %x dynpld %x\n",config,status,feature,dynpld);

	if(irqState){
		NVIC_EnableIRQ(nrf_struct->irqn);
	}

	return (xcvrs_num-1);
}

void nrf24l01p_powerup(uint32_t ID){

	if(xcvrs_buff[ID].state!=_powerdown) return;	//if already powered up do nothing

	//disabling interrupts for atomicity
	uint32_t irqState=nrf24l01p_stopAndGetIRQ(ID);

	ce_low(&xcvrs_buff[ID].nrf_struct);	//CE=0 to remain on standby-I mode

	//flush RX fifo
	nrf24l01p_reset(&xcvrs_buff[ID].nrf_struct); //reset all transceiver registers to default configurations and power it off
	nrf24l01p_init(&xcvrs_buff[ID].nrf_struct);	//init the transceiver with the informations passed with the handle structure

	if(xcvrs_buff[ID].nrf_struct.idle_state != _rx){
    	xcvrs_buff[ID].state=_standby;
    }else{
		//set mode to prx
		set_register_bit(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_CONFIG, NRF24L01P_BIT_PRIM_RX);

		xcvrs_buff[ID].state=_rx;

		ce_high(&xcvrs_buff[ID].nrf_struct);
    }

    set_register_bit(&(xcvrs_buff[ID].nrf_struct), NRF24L01P_REG_CONFIG, NRF24L01P_BIT_PWR_UP);

    nrf24l01p_setIRQ(ID,irqState);

	return;
}

void nrf24l01p_powerdown(uint32_t ID){
	//disabling interrupts for atomicity
	uint32_t irqState=nrf24l01p_stopAndGetIRQ(ID);

	ce_low(&(xcvrs_buff[ID].nrf_struct));

	reset_register_bit(&(xcvrs_buff[ID].nrf_struct), NRF24L01P_REG_CONFIG, NRF24L01P_BIT_PWR_UP);

	xcvrs_buff[ID].state=_powerdown;

	nrf24l01p_setIRQ(ID,irqState);

	return;
}

xcvr_state nrf24l01p_getState(uint32_t ID){

	return xcvrs_buff[ID].state;

}

uint8_t nrf24l01p_transmit(uint32_t ID, uint8_t* payload,uint32_t len, uint8_t ack)
{
	if(len==0 || payload==NULL) return 1;

	//disabling interrupts for atomicity
	uint32_t irqState=nrf24l01p_stopAndGetIRQ(ID);

    //checking if transmission is already ongoing
	if(xcvrs_buff[ID].state == _tx){
		HAL_NVIC_EnableIRQ(xcvrs_buff[ID].nrf_struct.irqn);
		return 1;
	}

	//otherwise start a new transmission
	ce_low(&xcvrs_buff[ID].nrf_struct); //eventually exit from _rx

	//set mode to ptx
	reset_register_bit(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_CONFIG, NRF24L01P_BIT_PRIM_RX);

	if(payload != NULL){
		//flush TX fifo
	    send_command(&xcvrs_buff[ID].nrf_struct, NRF24L01P_CMD_FLUSH_TX);
		//load new TX payload
	    if(ack){
	    	write_transaction(&xcvrs_buff[ID].nrf_struct, NRF24L01P_CMD_W_TX_PAYLOAD, payload, len);
	    }else{
	    	write_transaction(&xcvrs_buff[ID].nrf_struct, NRF24L01P_CMD_W_TX_PAYLOAD_NOACK, payload, len);
	    }
		//also saving payload
		memcpy(&xcvrs_buff[ID].txBuffer,payload,len);
		xcvrs_buff[ID].txLen=len;
	}
	xcvrs_buff[ID].state=_tx;

	//CE high to sart transmission
	ce_high(&xcvrs_buff[ID].nrf_struct);

	nrf24l01p_setIRQ(ID,irqState);

	return 0;
}

void nrf24l01p_irq(uint16_t irq_pin)
{
	//searching for the corresponding xcvr
	uint32_t ID=0;
	for(;ID<xcvrs_num;ID++){
		if(xcvrs_buff[ID].nrf_struct.irq_pin == irq_pin) break;
	}
	if(ID>=xcvrs_num) return; //if transceiver not found, return

	//otherwise searching for the cause of interrupt
	//reading status register
	uint8_t status = send_command(&xcvrs_buff[ID].nrf_struct, NRF24L01P_CMD_NOP);

	if(status & NRF24L01P_BIT_RX_DR){
		//reading received packet length
		read_transaction(&xcvrs_buff[ID].nrf_struct, NRF24L01P_CMD_R_RX_PL_WID, &xcvrs_buff[ID].rxLen, 1);

		//resetting irq pin
		set_register_bit(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_STATUS, NRF24L01P_BIT_RX_DR);

		//emptying fifo (multiple packets could have arrived in case of overrun)
		while(!(read_register(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_FIFO_STATUS) & NRF24L01P_BIT_RX_EMPTY)){
			//reading received packet
			read_transaction(&xcvrs_buff[ID].nrf_struct, NRF24L01P_CMD_R_RX_PAYLOAD, xcvrs_buff[ID].rxBuffer, xcvrs_buff[ID].rxLen);

			//calling callback
			nrf24l01p_rx_callback(ID, xcvrs_buff[ID].rxBuffer, xcvrs_buff[ID].rxLen);
		}
		//if a new packet arrived while reading, another IRQ is pending, if we already read the packet that triggered
		//the IRQ there's no problem because the isq will be called again but will find the FIFO empty
	}

	if(status & NRF24L01P_BIT_TX_DS){
		//setting state to _standby
		//if the state was not changed by another call i.e. of powerdown
		if(xcvrs_buff[ID].state==_tx) xcvrs_buff[ID].state=_standby;

		//calling callback
		nrf24l01p_tx_callback(ID, xcvrs_buff[ID].txBuffer,xcvrs_buff[ID].txLen);

		//clearing interrupt bit
		set_register_bit(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_STATUS, NRF24L01P_BIT_TX_DS);

	}

	if(status & NRF24L01P_BIT_MAX_RT){
		//lower ce to avoid retransmitting packet
		ce_low(&xcvrs_buff[ID].nrf_struct);

		//setting state to _standby
		//if the state was not changed by another call i.e. of powerdown
		if(xcvrs_buff[ID].state==_tx) xcvrs_buff[ID].state=_standby;

		//calling callback
		nrf24l01p_max_callback(ID, xcvrs_buff[ID].txBuffer,xcvrs_buff[ID].txLen);

		//clearing interrupt bit
		set_register_bit(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_STATUS, NRF24L01P_BIT_MAX_RT);
	}

	//if no new transmission started on callback and idle state is rx, go to rx state
	if(xcvrs_buff[ID].state==_standby && xcvrs_buff[ID].nrf_struct.idle_state == _rx){
		//CE low to exit stanby/tx mode)
		ce_low(&xcvrs_buff[ID].nrf_struct);

		//set mode to prx
		set_register_bit(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_CONFIG, NRF24L01P_BIT_PRIM_RX);

		xcvrs_buff[ID].state = _rx;

		//flush RX fifo and irq
		set_register_bit(&xcvrs_buff[ID].nrf_struct, NRF24L01P_REG_STATUS, NRF24L01P_BIT_RX_DR);
		send_command(&xcvrs_buff[ID].nrf_struct, NRF24L01P_CMD_FLUSH_RX);

		ce_high(&xcvrs_buff[ID].nrf_struct);

	}
}
