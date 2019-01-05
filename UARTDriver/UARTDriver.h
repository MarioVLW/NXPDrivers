/*
 * UARTDriver.h
 *
 *  Created on: Dec 22, 2018
 *      Author: MarioVL
 */

#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#define MAX_PRIORITY 16u
#define UART3_NVIC_POSITION 5
#define UART3_NVICIPR_POSITION 15
#define HIGH_BYTE_BAUDRATE 0u
#define LOW_BYTE_BAUDRATE 130u

#define END_TEXT_ASCII 0x03u
#define END_TEXT_ASCII_RX 126

typedef uint8_t* pointer_t;

typedef struct {
	pointer_t tx_data;
	pointer_t rx_data;
	uint8_t tx_index;
	uint8_t rx_index;
}UART_t;

typedef struct {
	uint8_t TX_EN : 1;
	uint8_t RX_EN : 1;
	uint8_t RX_CMPL : 1;
}UART_flags;

void UART_init();
void UART_NVIC_enable();
void UART_NVIC_disable();
void UART_begin();
void UART_clock_enable();
uint8_t is_TX_ready();
uint8_t is_RX_ready();
void UART_stop_sending();
void UART_stop_receiving();
void set_RX_buffer(uint8_t *buffer);
void UART_send(uint8_t *buffer);
		
/* Global variables */
UART_flags UART_status;
UART_t UART_data;

#endif /* UARTDRIVER_H_ */
