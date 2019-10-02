/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "buffer.h"	//este deberia estar solo en uart.h


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void delayLoop(uint32_t veces);


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

sw_buffer rx_buff;
sw_buffer tx_buff;



/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	//revillero
	uart_cfg_t var;
	var.parity=0;
	var.baudrate=9600;
	uartInit(0, var);
	/*
	initBuffer(&rx_buff);
	initBuffer(&tx_buff);
	*/

}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	//char msgg[5];
	//uartReadMsg(0, msgg, 0);
	//uartWriteMsg(0, "hola", 0);
	/*push_buffer('h', &tx_buff);
	push_buffer('o', &tx_buff);
	char receive = pop_buffer(&tx_buff);
	receive = pop_buffer(&tx_buff);
//	uartWriteMsg(0, pop_buffer(&tx_buff), 0);
 * uartWriteMsg
*/
	uartWriteMsg(0,"hola",4);


}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void delayLoop(uint32_t veces)
{
    while (veces--);
}


/*******************************************************************************
 ******************************************************************************/
