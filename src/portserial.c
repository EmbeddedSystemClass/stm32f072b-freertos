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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

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


/* ----------------------- Initialize USART ----------------------------------*/

BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	BOOL bStatus;

	// ---- WTF? ---- Oddity of STM32F series: word length includes parity. 7 bits no parity not possible
	CHAR wordLength;

	/* Enable clock for USART3. */
	rcc_periph_clock_enable(RCC_USART3);

	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_4_IRQ);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, ulBaudRate);
	usart_set_stopbits(USART3, USART_CR2_STOP_1_0BIT);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART3, USART_MODE_TX_RX);

	bStatus = TRUE;

	switch ( eParity )
	{
		case MB_PAR_NONE:
			usart_set_parity(USART3, USART_PARITY_NONE);
			break;
		case MB_PAR_ODD:
			usart_set_parity(USART3, USART_PARITY_ODD);
			break;
		case MB_PAR_EVEN:
			usart_set_parity(USART3, USART_PARITY_EVEN);
			break;
		default:
			bStatus = FALSE;
			break;
	}

	switch ( ucDataBits )
	{
		case 8:
			if (eParity == MB_PAR_NONE)
				wordLength = 8;
			else
				wordLength = 9;
			usart_set_databits(USART3, wordLength);
			break;
		case 7:
			if (eParity == MB_PAR_NONE)
				{
					wordLength = 8;
					//bStatus = FALSE;
					usart_set_databits(USART3, 8);
				}
			else
				usart_set_databits(USART3, 8);
			break;
		default:
			bStatus = FALSE;
	}

	if( bStatus == TRUE )
	{
		/* Finally enable the USART. */
		usart_disable_rx_interrupt(USART3);
		usart_disable_tx_interrupt(USART3);
		usart_enable(USART3);
	}
	return bStatus;
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



