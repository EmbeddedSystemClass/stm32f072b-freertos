//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "diag/Trace.h"

#include <cdcacm.h>

#include "hardware.h"

//FreeRTOS kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define DAY_OF_WEEK_NAME_SIZE		3
const char* const DayOfWeekName[] = {"---", "MON", "TUE" , "WED", "THU", "FRI", "SAT", "SUN"};

#define ADC_RESULT_APPROX_COUNT		8	//Constant for moving average method

#define partstMAX_LEDS		4


//Task priorities
#define UART_GATEKEEPER_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define TEMPERATURE_CALCULATION_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	RTC_WAKEUP_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )


#define UART_QUEUE_LENGTH					( 64 )
#define ADC_QUEUE_LENGTH					( 1 )
#define TEMPERATURE_QUEUE_LENGTH				( 1 )

/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

static QueueHandle_t xUARTQueue = NULL;
static QueueHandle_t xADCQueue = NULL;
static QueueHandle_t xTemperatureQueue = NULL;

//RTC Wakeup semaphore
static SemaphoreHandle_t xSemaphoreRTCWakeup = NULL;

/*-----------------------------------------------------------*/

static void prvTemperatureCalculationTask( void *pvParameters );
static void prvRTCWakeupTask( void *pvParameters );

static void prvUARTGatekeeperTask( void *pvParameters );

//ADC1 interrupt service routine (EOC)
void adc_comp_isr(void)
{
	int16_t adc_result = 0;

	BaseType_t xHigherPriorityTaskWoken;

	// We have not woken a task at the start of the ISR.
	xHigherPriorityTaskWoken = pdFALSE;

	adc_result = adc_read_regular(ADC1);

	xQueueSendFromISR( xADCQueue, &adc_result, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken )
	{
		portYIELD_FROM_ISR ( xHigherPriorityTaskWoken );
	}
}

//RTC interrupt service routine (Periodic wakeup)
void rtc_isr(void)
{
	static BaseType_t xHigherPriorityTaskWoken;

	rtc_clear_wakeup_flag();
	exti_reset_request(EXTI20);

	xHigherPriorityTaskWoken = pdFALSE;

	//Give semaphore here to periodic task
	xSemaphoreGiveFromISR( xSemaphoreRTCWakeup, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// ----- main() ---------------------------------------------------------------


int main(int argc, char* argv[])
{
	xSemaphoreRTCWakeup = xSemaphoreCreateBinary();

	xADCQueue = xQueueCreate( ADC_QUEUE_LENGTH, sizeof( uint16_t ) );

	//Create UART queue for writing to UART gatekeeper task
	xUARTQueue = xQueueCreate( UART_QUEUE_LENGTH, sizeof( char ) );

	xTemperatureQueue = xQueueCreate( TEMPERATURE_QUEUE_LENGTH, sizeof( uint16_t ) );

	//Initialize board hardware
	vInitHardware();

	if( (xSemaphoreRTCWakeup != NULL) && (xUARTQueue != NULL) && (xADCQueue != NULL) && (xTemperatureQueue != NULL))
	{
		//If all semaphores and queues were successfully created, then create tasks

		xTaskCreate( prvUARTGatekeeperTask,					/* The function that implements the task. */
					"UGK", 									/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 				/* The size of the stack to allocate to the task. */
					NULL, 									/* The parameter passed to the task - if present. */
					TEMPERATURE_CALCULATION_TASK_PRIORITY, 		/* The priority assigned to the task. */
					NULL );									/* The task handle is not required, so NULL is passed. */

		xTaskCreate( prvTemperatureCalculationTask, "ADC", configMINIMAL_STACK_SIZE, NULL, TEMPERATURE_CALCULATION_TASK_PRIORITY, NULL );

		xTaskCreate( prvRTCWakeupTask, "RTC", configMINIMAL_STACK_SIZE, NULL, RTC_WAKEUP_TASK_PRIORITY, NULL );

		//Start FreeRTOS scheduler
		vTaskStartScheduler();
	}


	while(1)
	{

	}

}

// ----------------------------------------------------------------------------

static void prvTemperatureCalculationTask( void *pvParameters )
{
	int16_t adc_data;
	int16_t adc_fifo[ADC_RESULT_APPROX_COUNT];
	uint16_t adc_fifo_current_position = 0;
	int16_t adc_fifo_sum = 0;	//Considering that ADC is 12 bit and right-aligned, so uint16_t is enough to hold sum of 12-bit values.

	uint16_t adc_average = 0;
	int16_t temp_celsius10 = 0;

	uint8_t first_run = 1;	//Fifo is uninitialized, so we need tofill it with first read value.

	for( ;; )
	{
		//Wait for data written to queue from ADC ISR
		xQueueReceive( xADCQueue, &adc_data, portMAX_DELAY );

		if(first_run)
		{
			for(first_run=0; first_run<ADC_RESULT_APPROX_COUNT; first_run++)
			{
				adc_fifo[first_run] = adc_data;
			}

			first_run = 0;
		}

		if(adc_fifo_current_position < ADC_RESULT_APPROX_COUNT)
		{
			adc_fifo[adc_fifo_current_position] = adc_data;
			adc_fifo_current_position++;

		} else
		{
			adc_fifo_current_position = 0;
			adc_fifo[adc_fifo_current_position] = adc_data;
		}

		//Calculate average of all array elements

		adc_fifo_sum = 0;

		for(adc_data=0; adc_data < ADC_RESULT_APPROX_COUNT; adc_data++)
		{
			adc_fifo_sum += adc_fifo[adc_data];
		}

		adc_average = adc_fifo_sum / ADC_RESULT_APPROX_COUNT;

		//Calculate actual temperature, in *C (Formula from ST RM0091 manual)
		temp_celsius10 = (((int32_t) adc_average * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR );
		temp_celsius10 = temp_celsius10 * (int32_t)(1100 - 300);
		temp_celsius10 = temp_celsius10 / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
		temp_celsius10 = temp_celsius10 + 300;

		//Send to queue
		xQueueSend( xTemperatureQueue, &temp_celsius10, 0U );
	}
}

/*-----------------------------------------------------------*/

//Unblock this task every 1 sec., via semaphore from RTC wakeup ISR
static void prvRTCWakeupTask( void *pvParameters )
{
	uint8_t i;
	char message[UART_QUEUE_LENGTH];

	DateTime_t datetime;

	uint8_t temperature_output_disabled = 0;
	uint16_t temperature = 0;

	for( ;; )
	{
		if( xSemaphoreTake( xSemaphoreRTCWakeup, portMAX_DELAY ) == pdTRUE )
		{
			//gpio_toggle(GPIOC, (GPIO6) << 1);

			rtc_get_datetime_struct(&datetime);

			if(exti_get_flag_status(EXTI0))
			{
				//Invert flag which allows temperature output
				temperature_output_disabled = ~temperature_output_disabled;

				exti_reset_request(EXTI0);
			}

			if(temperature_output_disabled)
			{
				sprintf(message + DAY_OF_WEEK_NAME_SIZE, " %d, %d, %d, %d, %d, %d\r\n", datetime.year, datetime.months, datetime.days, datetime.hours, datetime.minutes, datetime.seconds);
			} else
			{
				xQueueReceive( xTemperatureQueue, &temperature, portMAX_DELAY );

				sprintf(message + DAY_OF_WEEK_NAME_SIZE, " %d, %d, %d, %d, %d, %d, %d.%d\r\n", datetime.year, datetime.months, datetime.days, datetime.hours, datetime.minutes, datetime.seconds, temperature/10, temperature%10);
			}

			//Insert day of week name (3 characters) to start of the message
			strncpy(message, DayOfWeekName[datetime.dayOfWeek], DAY_OF_WEEK_NAME_SIZE);

			for(i=0; i<strlen(message); i++)
			{
				xQueueSend( xUARTQueue, &message[i], 0U );
			}

			//Manually start next ADC conversion
			adc_start_conversion_regular(ADC1);
		}
	}

}


/*-----------------------------------------------------------*/

static void prvUARTGatekeeperTask( void *pvParameters )
{
	char new_char;

	for( ;; )
	{
		xQueueReceive( xUARTQueue, &new_char, portMAX_DELAY );

		usart_send_blocking(USART1, new_char);
	}
}
/*-----------------------------------------------------------*/


//Various useful, but not used here hooks from FreeRTOS :)

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/



