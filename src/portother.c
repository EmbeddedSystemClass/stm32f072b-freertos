/*
 * portother.c
 *
 *  Created on: 14 дек. 2016 г.
 *      Author: frost
 */

/* ----------------------- System includes ----------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "mb.h"
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
//#include <intrinsics.h>

/* ----------------------- Variables ----------------------------------------*/
static BOOL     bIsWithinException = FALSE;

/* ----------------------- Start implementation -----------------------------*/

void vMBPortSetWithinException( BOOL bInException )
{
    bIsWithinException = bInException;
}

BOOL bMBPortIsWithinException( void )
{
    return bIsWithinException;
}

void vMBPortEnterCritical( void )
{
    taskENTER_CRITICAL(  );
}

void vMBPortExitCritical( void )
{
    taskEXIT_CRITICAL(  );
}

void vMBPortClose( void )
{
    extern void     vMBPortSerialClose( void );
    extern void     vMBPortTimerClose( void );
    extern void     vMBPortEventClose( void );
    vMBPortSerialClose(  );
    vMBPortTimerClose(  );
    vMBPortEventClose(  );
}
