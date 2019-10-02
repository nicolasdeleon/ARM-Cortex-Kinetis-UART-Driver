/*
 * uart.c
 *
 *  Created on: 30 sep. 2019
 *      Author: Tomas
 */

//includes
#include "uart.h"
#include "hardware.h"
#include "MK64F12.h"
#include "board.h"
#include "buffer.h"

static UART_Type *  uarts_ptr[]= UART_BASE_PTRS; //{URT0,URT1,URT2,URT3,URT4,URT5} donde cada UART apunta UARTX base address
static PORT_Type * ports_p[]=PORT_BASE_PTRS;
static SIM_Type *  sim=SIM;
#define UART_HAL_DEFAULT_BAUDRATE (9600)

//static variables for the file
static void UART_SetBaudRate(UART_Type *uart, uint32_t baudrate);
static void uart_putchar (uint8_t id, char ch);

//DEFINE STATIC BUFFER
static sw_buffer UartTxBuffer = {TRUE,BUFFER_SIZE,0,0,0,FALSE,FALSE,TRUE};
static sw_buffer UartRxBuffer = {TRUE,BUFFER_SIZE,0,0,0,FALSE,FALSE,TRUE};


static void uartMode(pin_t pin){
//Nada mas funciona o esta probada para pota 14 y 15

	uint8_t port=PIN2PORT(pin);
	uint8_t num=PIN2NUM(pin);
	if(port==PA)//Hago Clock gating con el puerto deseado
	{
		sim->SCGC5 |= SIM_SCGC5_PORTA(num<<1);
	}
	else if(port==PB)
	{
		sim->SCGC5 |= SIM_SCGC5_PORTB(num<<1);
	}
	else if(port==PC)
	{
		sim->SCGC5 |= SIM_SCGC5_PORTC(num<<1);
	}
	else if(port==PD)
	{
		sim->SCGC5 |= SIM_SCGC5_PORTD(num<<1);
	}
	else if(port==PE)
	{
		sim->SCGC5 |= SIM_SCGC5_PORTE(num<<1);
	}

	ports_p[port]->PCR[num]&=~PORT_PCR_MUX(7);
	ports_p[port]->PCR[num]|=PORT_PCR_MUX(3);//Configuro el mux del pin para que apunte al UART0
	//                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          ports_p[port]->PCR[num]|=PORT_PCR_IRQC(0);//DESHABILITO INTERRUPCIONES

}

void uartInit (uint8_t id, uart_cfg_t config){

	uartMode(PIN_UART0_TX); //pa14
	uartMode(PIN_UART0_RX); //pa15

	//id ENTRE 0 y 5 uarts number. creo que esto no es el puerto sino el pin
	//uint8_t port=PIN2PORT(pin);
	//uint8_t num=PIN2NUM(pin);
	if (id == 0){
		sim->SCGC4 |= SIM_SCGC4_UART0_MASK;
	}else if(id == 1){
		sim->SCGC4 |= SIM_SCGC4_UART1_MASK;
	} else if(id == 2){
		sim->SCGC4 |= SIM_SCGC4_UART2_MASK;
	}else if (id == 3){
		sim->SCGC4 |= SIM_SCGC4_UART3_MASK;
	}else if (id == 4){
		sim->SCGC1 |= SIM_SCGC1_UART4_MASK;
	}else if (id == 5){
		sim->SCGC1 |= SIM_SCGC1_UART5_MASK;
	}else {
		return;
	}




	//desabilito transmiter y reciever antse de hacer cualquier tipo de cambio,
	//TEnable y REenable en 0
	uarts_ptr[id]->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

	uarts_ptr[id]->D = 0;		//necesario para configurar FIFO
	/**habilito FIFO**/


	//enable fifo receive and transmit
//	uarts_ptr[id]->PFIFO |= (UART_PFIFO_RXFE_MASK | UART_PFIFO_TXFE_MASK);



	/* Configure the UART for 8-bit mode, no parity */
	/* We need all default settings, so entire register is cleared */
	uarts_ptr[id]->C1 = 0x01;


	//habilito interrupciones
	//usar el handler correspondiente
	__NVIC_EnableIRQ(UART0_RX_TX_IRQn);
	//habilito las interrupciones de transmicion completa y buffer de recibir lleno
	//uarts_ptr[id]->C2 |= (UART_C2_TCIE_MASK | UART_C2_RIE_MASK);

	//baudrate configuration
	UART_SetBaudRate(uarts_ptr[id],config.baudrate);
	/* Enable receiver and transmitter */
	uarts_ptr[id]->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );

}

__ISR__ UART0_RX_TX_IRQHandler(void){
	if(uarts_ptr[0]->S1 | UART_S1_TDRE_MASK & UartTxBuffer.fifo_not_empty_flag){	//el buffer esta vacio
		uart_Tx_Interrupt();
	}
}



void UART_SetBaudRate(UART_Type *uart, uint32_t baudrate){

	uint16_t sbr, brfa;
	uint32_t clock;

	//configuro el clock del baudrate que tiene que ser 16 veces menor al de la kinetis o 32 veces menor si
	//viene del bus.
	//seteo el clock correspondiente
	clock = ((uart==UART0) || (uart == UART1))?(__CORE_CLOCK__ ):(__CORE_CLOCK__ >>1);

	//seteamos el baudrate entre los limites admisibles, numeros negativos no se tienen en cuenta por el tipo de dato
	baudrate = ((baudrate == 0)?(UART_HAL_DEFAULT_BAUDRATE):((baudrate > 0x1FFF)?(UART_HAL_DEFAULT_BAUDRATE):(baudrate)));

	sbr = clock / (baudrate << 4); //seteo el baudrate correspondiente con el clokc correspondiente bajo la formula, dividiendolo por 16
	brfa = (clock << 1) / baudrate - (sbr << 5);

	uart->BDH = UART_BDH_SBR(sbr>>8); //shifteo para poner la info de los primeros 8bits, siempre los primeros 3 bits van a quedar en 0 por limite de 1FFF
	uart->BDL = UART_BDL_SBR(sbr); //coloco los 8 ultimos bits de sbr en BDL
	//Tiene que ver con un ajuste fino con demoras fraccionarias para que coincida la velocicad de transmision del sistema
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}

/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param id UART's number
 * @param msg Buffer with the bytes to be transfered
 * @param cant Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/



uint8_t uartWriteMsg(uint8_t id,char* msg, uint8_t cant){
	int i =0;
	for(i=0;i<cant;i++){
		uart_send_byte(id,msg[i]);
	}
}

void uart_send_byte(uint8_t id,uint8_t byte) {

  ///////////////////////////////////////////////////////////
  /* disable interrupts while manipulating buffer pointers */
  ///////////////////////////////////////////////////////////
	uarts_ptr[id]->C2 &= ~(UART_C2_TIE_MASK | UART_C2_RIE_MASK | UART_C2_TCIE_MASK);
	//lleno mi buffer con 1 byte
	push_buffer(byte, &UartTxBuffer);
	//habilito interrupcion
  ///////////////////////
  /* enable interrupts */
  ///////////////////////
	uarts_ptr[id]->C2 |= (UART_C2_TIE_MASK | UART_C2_RIE_MASK | UART_C2_TCIE_MASK);

	if(UartTxBuffer.num_bytes > 0) {                      // if there is data in the buffer

		UartTxBuffer.fifo_not_empty_flag = 1;               // set flag

    ///////////////////////////////////////////////////////////////////////////
    /* if using shared RX/TX hardware buffer, disable RX data interrupt here */
    /* enable UART "TX hw buffer empty" interrupt here                       */
    ///////////////////////////////////////////////////////////////////////////
	}



}

/***************************************************************************************************************/
// UART transmit interrupt sub-routine
//  - interrupts when the tx hardware buffer is empty
//  - checks if data exists in the tx software buffer
//  - if data exists, it places the oldest element of the sw buffer into the tx hardware buffer
//  - if the sw buffer is emptied, it disables the "hw buffer empty" interrupt
//  - automatically handles "uart_tx_buffer_full_flag"
//////////////////////////////////////////////
 void uart_Tx_Interrupt(){
//////////////////////////////////////////////

  /* Explicitly clear the source of interrupt if necessary */

  if(UartTxBuffer.num_bytes == BUFFER_SIZE) { // if the sw buffer is full
	  UartTxBuffer.fifo_full_flag = 0;               // clear the buffer full flag because we are about to make room
  }
  if(UartTxBuffer.num_bytes > 0) {                 // if data exists in the sw buffer

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	uarts_ptr[0]->D = UartTxBuffer.data_buf[UartTxBuffer.i_first]; // place oldest data element in the TX hardware buffer
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	UartTxBuffer.i_first++;                        // increment the index of the oldest element
	UartTxBuffer.num_bytes--;                      // decrement the bytes counter
  }
  if(UartTxBuffer.i_first == BUFFER_SIZE) {   // if the index has reached the end of the buffer,
	  UartTxBuffer.i_first = 0;                      // roll over the index counter
  }
  if(UartTxBuffer.num_bytes == 0) {                // if no more data exists

	  UartTxBuffer.fifo_not_empty_flag = 0;          // clear flag

    //////////////////////////////////////////////////////////////////////////
    /* disable UART "TX hw buffer empty" interrupt here                     */
    /* if using shared RX/TX hardware buffer, enable RX data interrupt here */
    //////////////////////////////////////////////////////////////////////////

  }

	if(!UartTxBuffer.fifo_not_empty_flag){
		uarts_ptr[0]->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

	}
}// end UART TX IRQ handler
/***************************************************************************************************************/



static void uart_putchar (uint8_t id, char ch){

 /* Wait until space is available in the FIFO */
 while(!(uarts_ptr[id]->S1 & UART_S1_TDRE_MASK)){};

 /* Send the character */
 //UART_D_REG(channel) = (uint8)ch;
 uarts_ptr[id]->D = (uint8_t) ch;
}


/**
 * @brief Read a received message. Non-Blocking
 * @param id UART's number
 * @param msg Buffer to paste the received bytes
 * @param cant Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg(uint8_t id, char* msg, uint8_t cant){

	/* Wait until character has been received */
	while (!(uarts_ptr[id]->S1 & UART_S1_RDRF_MASK));

	*msg = uarts_ptr[id]->D;
	/* Return the 8-bit data from the receiver */
	return cant;
}
