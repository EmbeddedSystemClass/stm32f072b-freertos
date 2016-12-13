/*
 * portevent.c
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: frost
 */

/* ----------------------- System includes ----------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static xQueueHandle xQueueHdl;


/* ----------------------- Start implementation -----------------------------*/

BOOL xMBPortEventInit( void )
{
    BOOL bStatus = FALSE;
    if( 0 != ( xQueueHdl = xQueueCreate( 1, sizeof( eMBEventType ) ) ) )
    {
        bStatus = TRUE;
    }
    return bStatus;
}

void vMBPortEventClose( void )
{
    if( 0 != xQueueHdl )
    {
        vQueueDelete( xQueueHdl );
        xQueueHdl = 0;
    }
}


BOOL xMBPortEventPost( eMBEventType eEvent )
{
    BOOL bStatus = TRUE;
    if( bMBPortIsWithinException(  ) )
    {
        ( void )xQueueSendFromISR( xQueueHdl, ( const void * )&eEvent, pdFALSE );
    }
    else
    {
        ( void )xQueueSend( xQueueHdl, ( const void * )&eEvent, pdFALSE );
    }

    return bStatus;
}


BOOL xMBPortEventGet( eMBEventType * peEvent )
{
    BOOL xEventHappened = FALSE;

    if( pdTRUE == xQueueReceive( xQueueHdl, peEvent, portTICK_RATE_MS * 50 ) )
    {
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

