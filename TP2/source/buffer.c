/*
 * buffer.c
 *
 *  Created on: 2 oct. 2019
 *      Author: Tomas
 */

////////////////////////////////////////////////////////////////////////////////////////
/* enter necessary header files for proper interrupt vector and UART/USART visibility */
////////////////////////////////////////////////////////////////////////////////////////

#include "buffer.h"

/*
static sw_buffer buffers_array[MAX_BUFFERS];
static uint8_t buffers_cant = 0;		//cantidad de buffers inicializados

sw_buffer rx_fifo = { {0}, 0, 0, 0 }; // declare a receive software buffer
sw_buffer tx_fifo = { {0}, 0, 0, 0 }; // declare a transmit software buffer
*/

//funciones propias del archivo (por ahora no las uso)
buff_id_t bufferGetId(void);

void initBuffer(sw_buffer * buffer){
	buffer->enable=TRUE;
	buffer->i_first=0;
	buffer->i_last=0;
	buffer->num_bytes=0;
	buffer->fifo_full_flag=0;
	buffer->fifo_not_empty_flag=0;
	buffer->fifo_ovf_flag=0;
}



/***************************************************************************************************************/
// UART data transmit function
//  - checks if there's room in the transmit sw buffer
//  - if there's room, it transfers data byte to sw buffer
//  - automatically handles "uart_tx_buffer_full_flag"
//  - sets the overflow flag upon software buffer overflow (doesn't overwrite existing data)
//  - if this is the first data byte in the buffer, it enables the "hw buffer empty" interrupt
void push_buffer(uint8_t byte, sw_buffer * tx_fifo) {


  ///////////////////////////////////////////////////////////
  /* disable interrupts while manipulating buffer pointers */
  ///////////////////////////////////////////////////////////

	if(tx_fifo->num_bytes == BUFFER_SIZE) {      // no room in the sw buffer
		tx_fifo->fifo_ovf_flag = 1;                     // set the overflow flag
	}else if(tx_fifo->num_bytes < BUFFER_SIZE) { // if there's room in the sw buffer
		tx_fifo->data_buf[tx_fifo->i_last] = byte;       // transfer data byte to sw buffer
		tx_fifo->i_last++;                              // increment the index of the most recently added element
		tx_fifo->num_bytes++;                           // increment the bytes counter
	}
	if(tx_fifo->num_bytes == BUFFER_SIZE) {      // if sw buffer is full
		tx_fifo->fifo_full_flag = 1;                    // set the TX FIFO full flag
	}
	if(tx_fifo->i_last == BUFFER_SIZE) {         // if the "new data" index has reached the end of the buffer,
		tx_fifo->i_last = 0;                            // roll over the index counter
	}

  ///////////////////////
  /* enable interrupts */
  ///////////////////////


}
/***************************************************************************************************************/


/***************************************************************************************************************/
// UART data receive function
//  - checks if data exists in the receive sw buffer
//  - if data exists, it returns the oldest element contained in the buffer
//  - automatically handles "uart_rx_buffer_full_flag"
//  - if no data exists, it clears the uart_rx_flag
uint8_t pop_buffer(sw_buffer * rx_fifo) {

  ///////////////////////////////////////////////////////////
  /* disable interrupts while manipulating buffer pointers */
  ///////////////////////////////////////////////////////////

	uint8_t byte = 0;
	if(rx_fifo->num_bytes == BUFFER_SIZE) { // if the sw buffer is full
		rx_fifo->fifo_full_flag = 0;               // clear the buffer full flag because we are about to make room
	}
	if(rx_fifo->num_bytes > 0) {                 // if data exists in the sw buffer
		byte = rx_fifo->data_buf[rx_fifo->i_first]; // grab the oldest element in the buffer
		rx_fifo->i_first++;                        // increment the index of the oldest element
		rx_fifo->num_bytes--;                      // decrement the bytes counter
	}else{                                      // RX sw buffer is empty
		rx_fifo->fifo_not_empty_flag = 0;          // clear the rx flag
	}
	if(rx_fifo->i_first == BUFFER_SIZE) {   // if the index has reached the end of the buffer,
		rx_fifo->i_first = 0;                      // roll over the index counter
	}

  ///////////////////////
  /* enable interrupts */
  ///////////////////////

	return byte;                                // return the data byte
}

bool buffer_is_full(sw_buffer * buff){
	return buff->fifo_full_flag;
}

bool buffer_is_empty(sw_buffer * buff){
	return !(buff->fifo_not_empty_flag);
}

/*
buff_id_t bufferGetId(void)
{

    if (buffers_cant > MAX_BUFFERS)
    {
        return BUFFER_INVALID_ID;
    }
    else
    {
    	buffers_array[buffers_cant].enable=FALSE;	//todavia no esta habilitado
        buffers_cant = buffers_cant + 1;			//hay un buffer mas
    	return (buffers_cant - 1);					//devuelvo el id correspondiente
    }
}
*/
