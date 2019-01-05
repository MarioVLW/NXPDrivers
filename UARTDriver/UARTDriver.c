/*
 * UARTDriver.c
 *
 *  Created on: Dec 22, 2018
 *      Author: MarioVL
 */

#include "derivative.h"
#include "UARTDriver.h"

void UART3_Status_IRQHandler()
{	
	if(is_TX_ready())
	{
		if(END_TEXT_ASCII != *(UART_data.tx_data + UART_data.tx_index))
		{
			UART3_D = *(UART_data.tx_data + UART_data.tx_index);
			UART_data.tx_index++;
		} else {
			UART_stop_sending();
		}
	}
	
	if(is_RX_ready())
	{
		uint8_t temp_data = UART3_D;
		
		UART_status.RX_EN = 1u;
		
		if(END_TEXT_ASCII_RX != temp_data)
		{
			*(UART_data.rx_data + UART_data.rx_index) = temp_data;
			UART_data.rx_index++;
		} else {
			UART_stop_receiving();
		}
	}
}

void set_RX_buffer(uint8_t *buffer)
{
	UART_data.rx_data = buffer;
}

void UART_clock_enable()
{
	SIM_SCGC4 |= SIM_SCGC4_UART3_MASK; //Enable the clock for the UART1
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //Enable the clock for the PORTC
}

void UART_init()
{
	UART_clock_enable();
	
	PORTC_PCR16 |= PORT_PCR_MUX(3); //UART3_RX mux
	PORTC_PCR17 |= PORT_PCR_MUX(3); //UART3_TX mux

	//Only for baud rate of 9600
	UART3_BDH = HIGH_BYTE_BAUDRATE;
	UART3_BDL = LOW_BYTE_BAUDRATE;
}

void UART_NVIC_enable()
{
	UART3_BDH |= 0x40;
	UART3_C2 |= UART_C2_RIE_MASK; //Enable RX interrupt
	NVICISER1 |= 1 << UART3_NVIC_POSITION; //Enables the interrupt (Set-enable register)
	NVICIP8 = 0 << UART3_NVICIPR_POSITION; //Set higher priority 
}

void UART_NVIC_disable()
{
	NVICICER1 = 1 << UART3_NVICIPR_POSITION;//Disables the interrupt (Clear-enable register)
	UART3_C2 ^= UART_C2_RIE_MASK; //Disable RX interrupt
}

void UART_begin()
{
	UART3_C2 = UART_C2_TE_MASK | UART_C2_RE_MASK; //Enable Rx and Tx
}

void UART_send(uint8_t *buffer)
{
	UART_data.tx_data = buffer;
	UART_status.TX_EN = 1;
	UART3_C2 |= UART_C2_TCIE_MASK;
}

void UART_stop_sending()
{
	UART3_C2 ^= UART_C2_TCIE_MASK;
	UART_data.tx_index = 0;
	UART_status.TX_EN = 0;
	UART3_D = 0;
}

void UART_stop_receiving()
{
	UART_data.rx_index = 0;
	UART_status.RX_CMPL = 1u;
	UART_status.RX_EN = 0;
	UART3_D = 0;
}

uint8_t is_TX_ready()
{
	return (UART_S1_TC_MASK == (UART3_S1&UART_S1_TC_MASK) && 1 == UART_status.TX_EN) ? 1u : 0u;
}

uint8_t is_RX_ready()
{
	return (UART_S1_RDRF_MASK == (UART3_S1&UART_S1_RDRF_MASK))? 1u : 0u;
}
