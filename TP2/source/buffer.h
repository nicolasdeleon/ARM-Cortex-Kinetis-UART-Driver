/*
 * buffer.h
 *
 *  Created on: 2 oct. 2019
 *      Author: Tomas
 */

#ifndef BUFFER_H_
#define BUFFER_H_

//includes
#include <stdint.h>
#include <stdbool.h>


//defines
#define MAX_BUFFERS 10
#define BUFFER_SIZE	128
#define BUFFER_INVALID_ID 255
#define TRUE 1
#define FALSE 0

//typedefs
typedef uint8_t buff_id_t;

typedef struct {
	bool enable;							//inicializacion
	uint8_t  data_buf[BUFFER_SIZE]; 		// FIFO buffer
	uint16_t i_first;                    	// index of oldest data byte in buffer
	uint16_t i_last;                     	// index of newest data byte in buffer
	uint16_t num_bytes;                  	// number of bytes currently in buffer
	bool fifo_full_flag;      				// this flag is automatically set and cleared by the software buffer
	bool fifo_ovf_flag;       				// this flag is not automatically cleared by the software buffer
	bool fifo_not_empty_flag; 				// this flag is automatically set and cleared by the software buffer

}sw_buffer;


// UART data transmit function
//  - checks if there's room in the transmit sw buffer
//  - if there's room, it transfers data byte to sw buffer
//  - automatically handles "uart_tx_buffer_full_flag"
//  - sets the overflow flag upon software buffer overflow (doesn't overwrite existing data)
//  - if this is the first data byte in the buffer, it enables the "hw buffer empty" interrupt
void push_buffer(uint8_t byte, sw_buffer * tx_fifo);


// UART data receive function
//  - checks if data exists in the receive sw buffer
//  - if data exists, it returns the oldest element contained in the buffer
//  - automatically handles "uart_rx_buffer_full_flag"
//  - if no data exists, it clears the uart_rx_flag
uint8_t pop_buffer(sw_buffer * rx_fifo);


//Indicates if buffer is complete
bool buffer_is_full(sw_buffer * buff);
//indicates if buffer is empty
bool buffer_is_empty(sw_buffer * buff);


#endif /* BUFFER_H_ */
