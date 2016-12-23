

#ifndef HARDWARE_H
#define HARDWARE_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>

#define GPIOC_LEDS_SHIFT	6
#define GPIOC_LEDS_MASK		0xF

#define LED_RED 	GPIO6
#define LED_BLUE 	GPIO7
#define LED_AMBER 	GPIO8
#define LED_GREEN 	GPIO9

#define LED1 	(1 << 0)
#define LED2 	(1 << 1)
#define LED3 	(1 << 2)
#define LED4 	(1 << 3)

#define USART1_PORT 		GPIOA
#define USART1_PORT_TX		GPIO9

#define USART2_PORT 		GPIOA
#define USART2_PORT_RX		GPIO15

#define USART3_PORT 		GPIOB
#define USART3_PORT_TX		GPIO10
#define USART3_PORT_RX		GPIO11


//Priority levels for STM32F0. The less numeric value for more urgent interrupt, e.g. nvic_pr_0 = 0x00 for ultimate urgency.
typedef enum tCM0_NVIC_Priority
{
	nvic_pr_0 = 0x00,
	nvic_pr_1 = 0x40,
	nvic_pr_2 = 0x80,
	nvic_pr_3 = 0xC0,
} CM0_NVIC_Priority_t;

#define RCC_BDCR_RTCSEL_MASK		(0x3)

#define RCC_RTCSEL_CLOCK_NONE		(0x0)
#define RCC_RTCSEL_CLOCK_LSE		(0x1)
#define RCC_RTCSEL_CLOCK_LSI		(0x2)
#define RCC_RTCSEL_CLOCK_HSE		(0x3)

#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define VDD_CALIB ((uint16_t) (3300))
#define VDD_APPLI ((uint16_t) (3000))

typedef enum tDayOfWeek
{
	Unknown = 0,
	Monday = 1,
	Tuesday,
	Wednesday,
	Thursday,
	Friday,
	Saturday,
	Sunday
} DayOfWeek_t;

typedef struct tDateTimeStruct
{
	//Date
	uint8_t year;	//Only 2 digits
	uint8_t months;
	uint8_t days;
	uint8_t dayOfWeek;

	//Time
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} DateTime_t;

//void putchar(char c);

void vInitHardware(void);

void rtc_get_datetime_struct(DateTime_t *datetime);

void rtc_set_datetime_dayofweek(const uint8_t value);
void rtc_set_datetime_year(const uint8_t value);
void rtc_set_datetime_month(const uint8_t value);
void rtc_set_datetime_day(const uint8_t value);
void rtc_set_datetime_hours(const uint8_t value);

void rtc_set_datetime_minutes(const uint8_t value);
void rtc_set_datetime_seconds(const uint8_t value);
void rcc_rtc_select_clock(uint32_t clock);
void rtc_set_datetime(const DateTime_t *datetime);
void rcc_rtc_select_clock(uint32_t clock);
void rtc_enter_init_mode(void);
uint8_t rtc_is_init_mode_on(void);
void rtc_exit_init_mode(void);

void rcc_clock_setup_in_hse_out_48mhz(void);

void vSetLEDS(const uint8_t mask, const uint8_t value);

#endif /* HARDWARE_H */
