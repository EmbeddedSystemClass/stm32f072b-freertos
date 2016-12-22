/*
 * modbus.c
 *
 *  Created on: 13 дек. 2016 г.
 *      Author: frost
 */

#include "port.h"

#include "mb.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

//Modbus related defines
#define REG_INPUT_START                 ( 1000 )
#define REG_INPUT_NREGS                 ( 64 )

#define REG_HOLDING_START               ( 1 )
#define REG_HOLDING_NREGS               ( 32 )

//Modbus register/coil arrays
volatile USHORT usRegInputStart = REG_INPUT_START;
volatile USHORT usRegInputBuf[REG_INPUT_NREGS];
volatile USHORT usRegHoldingStart = REG_HOLDING_START;
volatile USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

/* ----------------------- Callbacks -----------------------------*/

eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}


eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;

                if((iRegIndex >= MB_HOLDINGREGS_DAYOFWEEK) && (iRegIndex <= MB_HOLDINGREGS_SECOND))
                {
					//Dirty hack to write to ModBus holding registers
					//Disable scheduler to prevent task switching
					taskENTER_CRITICAL();

					//Unlock RTC here
					rtc_unlock();

					//Enter RTC Init mode
					rtc_enter_init_mode();

					switch(iRegIndex)
					{
						case MB_HOLDINGREGS_DAYOFWEEK:
							rtc_set_datetime_dayofweek(usRegHoldingBuf[iRegIndex]);
							break;

						case MB_HOLDINGREGS_YEAR:
							rtc_set_datetime_year(usRegHoldingBuf[iRegIndex]);
							break;

						case MB_HOLDINGREGS_MONTH:
							rtc_set_datetime_month(usRegHoldingBuf[iRegIndex]);
							break;

						case MB_HOLDINGREGS_DAY:
							rtc_set_datetime_day(usRegHoldingBuf[iRegIndex]);
							break;

						case MB_HOLDINGREGS_HOUR:
							rtc_set_datetime_hours(usRegHoldingBuf[iRegIndex]);
							break;

						case MB_HOLDINGREGS_MINUTE:
							rtc_set_datetime_minutes(usRegHoldingBuf[iRegIndex]);
							break;

						case MB_HOLDINGREGS_SECOND:
							rtc_set_datetime_seconds(usRegHoldingBuf[iRegIndex]);
							break;

						default:
							break;

					};

					//Exit Init mode
					rtc_exit_init_mode();

					//Lock RTC
					rtc_lock();

					//Enable scheduler
					taskEXIT_CRITICAL();
                }

                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    return MB_ENOREG;
}


eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}


