/*
 * porttimer.c
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

/* ----------------------- libopencm3 STM32F includes -------------------------------*/
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>


static CHAR count = 0;

/* ----------------------- Initialize Timer -----------------------------*/
BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);
 	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_reset(TIM2);

	/* Timer global mode: - Divider 4, Alignment edge, Direction up */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_continuous_mode(TIM2);
	timer_set_prescaler(TIM2, 4799); /* 48MHz to 50 microseconds period */
	timer_set_period(TIM2, usTim1Timerout50us);

    return TRUE;
}

void vMBPortTimerClose( void )
{
	timer_disable_irq(TIM2, TIM_DIER_UIE);
	timer_disable_counter(TIM2);
}

/* ----------------------- Enable Timer -----------------------------*/
void vMBPortTimersEnable(  )
{
	/* Restart the timer with the period value set in xMBPortTimersInit( ) */
	TIM2_CNT = 0;
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	timer_enable_counter(TIM2);
}

/* ----------------------- Disable timer -----------------------------*/
void vMBPortTimersDisable(  )
{
	timer_disable_irq(TIM2, TIM_DIER_UIE);
	timer_disable_counter(TIM2);
}

void vMBPortTimersDelay( USHORT usTimeOutMS )
{
	vTaskDelay( usTimeOutMS / portTICK_RATE_MS );
}

/* ----------------------- Timer ISR -----------------------------*/
/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */

void tim2_isr(void)
{
	count++;

	if (timer_interrupt_source(TIM2, TIM_SR_UIF))
	{
		timer_clear_flag(TIM2, TIM_SR_UIF); /* Clear interrrupt flag. */
	}

	timer_get_flag(TIM2, TIM_SR_UIF);	/* Reread to force the previous (buffered) write before leaving */

	//Callback
	pxMBPortCBTimerExpired();
}

