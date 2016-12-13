/*
 * portserial.c
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: frost
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* ----------------------- libopencm3 STM32F includes -------------------------------*/
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/usart.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "port.h"
#include "mbport.h"

void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	/* If xRXEnable enable serial receive interrupts. If xTxENable enable
	 * transmitter empty interrupts.
	 */
	if( xRxEnable )
	{
		usart_enable_rx_interrupt(USART3);
	}
	else
	{
		usart_disable_rx_interrupt(USART3);
	}

	if( xTxEnable )
	{
		usart_enable_tx_interrupt(USART3);
	}
	else
	{
		usart_disable_tx_interrupt(USART3);
	}
}


BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	#warning "to be implemented"
}

/* ----------------------- Close Serial Port ----------------------------------*/

void vMBPortSerialClose( void )
{
	nvic_disable_irq(NVIC_USART3_4_IRQ);
	usart_disable(USART3);
}

/* -----------------------Send character  ----------------------------------*/

BOOL xMBPortSerialPutByte( CHAR ucByte )
{
	/* Put a byte in the UARTs transmit buffer. This function is called
	 * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
	 * called. */

	usart_send(USART3, ucByte);
	return TRUE;
}

/* ----------------------- Get character ----------------------------------*/

BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
	/* Return the byte in the UARTs receive buffer. This function is called
	 * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
	 */

	*pucByte = (CHAR) usart_recv(USART3);
	return TRUE;
}

/* ----------------------- ModBus USART ISR handler ----------------------------------*/

void usart3_4_isr(void)
{
	//If overrun happened
	if(usart_get_flag(USART3, USART_ISR_ORE))
	{
		//Clear overrun flag
		USART3_ICR |= USART_ICR_ORECF;
	}

	/* Check if we were called because of RXNE. */
	if (usart_get_interrupt_source(USART3,USART_ISR_RXNE))
	{
		//Callback for receive data
	    pxMBFrameCBByteReceived(  );
	}

	/* Check if we were called because of TXE. */
	if (usart_get_interrupt_source(USART3,USART_ISR_TXE))
	{
		//Callback for transmit data
	    pxMBFrameCBTransmitterEmpty(  );
	}
}



