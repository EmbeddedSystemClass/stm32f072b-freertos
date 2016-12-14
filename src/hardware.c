/*
 * hardware.c
 *
 *  Created on: 9 дек. 2016 г.
 *      Author:
 */

#include "hardware.h"

extern uint32_t rcc_ahb_frequency;
extern uint32_t rcc_apb1_frequency;

extern usbd_device *usbd_dev;

volatile uint32_t SystemCoreClock = 0;

void putchar(char c)
{
	usart_send_blocking(USART1, c);
}

//USB interrupt service routine
void usb_isr(void)
{
	usbd_poll(usbd_dev);
}


//Hardware initialization routine
void vInitHardware(void)
{
	uint16_t i;
	uint8_t adc_channel_array[16];	//todo: reduce  array size

	DateTime_t datetime;

	datetime.year = 16;
	datetime.months = 12;
	datetime.days = 11;
	datetime.dayOfWeek = Sunday;
	datetime.hours = 10;
	datetime.minutes = 20;
	datetime.seconds = 0;

	//Reset RTC and backup domain
	RCC_BDCR |= RCC_BDCR_BDRST;

	//simple delay
	for(i=0;i<10000;i++) {};

	RCC_BDCR &= ~RCC_BDCR_BDRST;

	//Configure clocks and enable peripherials
	rcc_clock_setup_in_hse_out_48mhz();

	//nvic_set_priority(NVIC_SYSTICK_IRQ, nvic_pr_1);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);

	//Enable PWR domain
	rcc_periph_clock_enable(RCC_PWR);

	//Disable RTC domain write protection
	pwr_disable_backup_domain_write_protect();

	rcc_periph_clock_enable(RCC_RTC);

	//nvic_enable_irq(NVIC_USB_IRQ);
	//rcc_periph_clock_enable(RCC_USB);

	//User button
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

	//LEDs
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6 | GPIO7 | GPIO8 | GPIO9);

	//USART1 TX on PA9
	gpio_mode_setup(USART1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART1_PORT_TX);
	gpio_set_af(USART1_PORT, GPIO_AF1, USART1_PORT_TX);

	//USART2 RX on PA15
	gpio_mode_setup(USART2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART2_PORT_RX);
	gpio_set_af(USART2_PORT, GPIO_AF1, USART2_PORT_RX);

	//USART3 TX & RX on PB10, PB11
	gpio_mode_setup(USART3_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART3_PORT_TX | USART3_PORT_RX);
	gpio_set_af(USART3_PORT, GPIO_AF4, USART3_PORT_TX | USART3_PORT_RX);


	//**************** EXTI initialization ****************

	//Configure external interrupt for User Button on PA0.

	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_reset_request(EXTI0);
	exti_enable_request(EXTI0);

	//We do not require enabling ISR for button

	exti_enable_request(EXTI20);
	exti_set_trigger(EXTI20, EXTI_TRIGGER_RISING);

	//**************** End of EXTI initialization ****************


	//**************** ADC initialization ****************

	//Internal temperature monitor channel
	adc_channel_array[0] = ADC_CHANNEL_TEMP;

	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV4);

	//Perform ADC calibration
	adc_calibrate(ADC1);

	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);

	//Enable ADC End of conversion interrupt
	adc_enable_eoc_interrupt(ADC1);

	//Enable ADC interrupt handling in NVIC and set it's priority
	nvic_enable_irq(NVIC_ADC_COMP_IRQ);
	nvic_set_priority(NVIC_ADC_COMP_IRQ, nvic_pr_1);

	//Now enable ADC
	adc_power_on(ADC1);

	adc_set_regular_sequence(ADC1, 1, adc_channel_array);

	adc_start_conversion_regular(ADC1);

	//**************** End of ADC initialization ****************


	//**************** RTC initialization ****************

	//Clock RTC from HSE / 32
	rcc_rtc_select_clock(RCC_RTCSEL_CLOCK_HSE);


	nvic_enable_irq(NVIC_RTC_IRQ);
	nvic_set_priority(NVIC_RTC_IRQ, nvic_pr_1);

	rtc_unlock();

	//wait for synchro
	RTC_ISR &= ~(RTC_ISR_RSF);
	while (!(RTC_ISR & RTC_ISR_RSF));

	//Enter RTC Init mode
	RTC_ISR |= RTC_ISR_INIT;
	while ((RTC_ISR & RTC_ISR_INITF) == 0) {};

	//RTC prescaler settings for 12 MHz HSE crystal
	rtc_set_prescaler(3749, 99);

	rtc_set_datetime(&datetime);

	rtc_set_wakeup_time(0, RTC_CR_WUCLKSEL_SPRE);

	rtc_clear_wakeup_flag();

	//Enable wakeup timer interrupt
	RTC_CR |= RTC_CR_WUTIE;


	//Exit Init mode
	RTC_ISR &= ~(RTC_ISR_INIT);

	//Enable RTC registers write protection
	rtc_lock();

	//**************** End of RTC initialization ****************



	//**************** UART ports initialization ****************

	//USART1
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX);						//Transmit only :)
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_enable(USART1);

	//todo: Send data to USART1 using DMA

	//USART2
	usart_set_baudrate(USART2, 9600 /*115200*/);
	usart_set_databits(USART2, 8);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_stopbits(USART2, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART2, USART_MODE_RX);						//Receive only :)
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	//USART2 RX inverted to conform LV-ProxSonar-EZ TX polarity.
	USART2_CR2 |= USART_CR2_RXINV;

	usart_enable(USART2);

	//Flush RX queue
	USART2_RQR |= USART_RQR_RXFRQ;

	usart_recv(USART2);

	//Clear pending IRQ request if it present
	nvic_clear_pending_irq(NVIC_USART2_IRQ);

	//Enable USART2 interrupt handling in NVIC
	nvic_enable_irq(NVIC_USART2_IRQ);

	//Enable USART2 RX interrupt request
	usart_enable_rx_interrupt(USART2);





	//**************** End of UART ports initialization ****************

	//usb_cdc_hw_config();
}

void rtc_get_datetime_struct(DateTime_t *datetime)
{
	uint32_t rtc_tr = RTC_TR;
	uint32_t rtc_dr = RTC_DR;

	datetime->seconds = (rtc_tr >> RTC_TR_SU_SHIFT) & RTC_TR_SU_MASK;
	datetime->seconds += 10*((rtc_tr >> RTC_TR_ST_SHIFT) & RTC_TR_ST_MASK);

	datetime->minutes = (rtc_tr >> RTC_TR_MNU_SHIFT) & RTC_TR_MNU_MASK;
	datetime->minutes += 10*((rtc_tr >> RTC_TR_MNT_SHIFT) & RTC_TR_MNT_MASK);

	datetime->hours = (rtc_tr >> RTC_TR_HU_SHIFT) & RTC_TR_HU_MASK;
	datetime->hours += 10*((rtc_tr >> RTC_TR_HT_SHIFT) & RTC_TR_HT_MASK);

	datetime->year = (rtc_dr >> RTC_DR_YU_SHIFT) & RTC_DR_YU_MASK;
	datetime->year += 10*((rtc_dr >> RTC_DR_YT_SHIFT) & RTC_DR_YT_MASK);

	datetime->dayOfWeek = (rtc_dr >> RTC_DR_WDU_SHIFT) & RTC_DR_WDU_MASK;

	datetime->months = (rtc_dr >> RTC_DR_MU_SHIFT) & RTC_DR_MU_MASK;
	datetime->months += 10*((rtc_dr >> RTC_DR_MT_SHIFT) & 0x01);

	datetime->days = (rtc_dr >> RTC_DR_DU_SHIFT) & RTC_DR_DU_MASK;
	datetime->days += 10*((rtc_dr >> RTC_DR_DT_SHIFT) & RTC_DR_DT_MASK);
}

void rtc_set_datetime(const DateTime_t *datetime)
{
	uint32_t rtc_tr_new = 0;
	uint32_t rtc_dr_new = 0;

	rtc_tr_new |= ((datetime->seconds / 10) & RTC_TR_ST_MASK) << RTC_TR_ST_SHIFT;	//Tens of seconds
	rtc_tr_new |= ((datetime->seconds % 10) & RTC_TR_SU_MASK) << RTC_TR_SU_SHIFT;	//Units of seconds

	rtc_tr_new |= ((datetime->minutes / 10) & RTC_TR_MNT_MASK) << RTC_TR_MNT_SHIFT;	//Tens of minutes
	rtc_tr_new |= ((datetime->minutes % 10) & RTC_TR_MNU_MASK) << RTC_TR_MNU_SHIFT;	//Units of minutes

	rtc_tr_new |= ((datetime->hours / 10) & RTC_TR_HT_MASK) << RTC_TR_HT_SHIFT;		//..
	rtc_tr_new |= ((datetime->hours % 10) & RTC_TR_HU_MASK) << RTC_TR_HU_SHIFT;

	rtc_dr_new |= ((datetime->year / 10) & RTC_DR_YT_MASK) << RTC_DR_YT_SHIFT;
	rtc_dr_new |= ((datetime->year % 10) & RTC_DR_YU_MASK) << RTC_DR_YU_SHIFT;

	rtc_dr_new |= (datetime->dayOfWeek & RTC_DR_WDU_MASK) << RTC_DR_WDU_SHIFT;

	rtc_dr_new |= ((datetime->months / 10) & 0x01) << RTC_DR_MT_SHIFT;
	rtc_dr_new |= ((datetime->months % 10) & RTC_DR_MU_MASK) << RTC_DR_MU_SHIFT;

	rtc_dr_new |= ((datetime->days / 10) & RTC_DR_DT_MASK) << RTC_DR_DT_SHIFT;
	rtc_dr_new |= ((datetime->days % 10) & RTC_DR_DU_MASK) << RTC_DR_DU_SHIFT;

	RTC_TR = rtc_tr_new;
	RTC_DR = rtc_dr_new;
}

void rcc_rtc_select_clock(uint32_t clock)
{
	RCC_BDCR &= ~(RCC_BDCR_RTCSEL_MASK << RCC_BDCR_RTCSEL_SHIFT);
	RCC_BDCR |= (clock << RCC_BDCR_RTCSEL_SHIFT);
}

void rcc_clock_setup_in_hse_out_48mhz(void)
{
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);

	//Set HSE as SYSCLK source
	rcc_set_sysclk_source(RCC_HSE);

	//AHB prescaler
	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);

	//APB prescaler
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

	//12 * 4 = 48 MHz
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL4);

	//Select PLL clock source
	RCC_CFGR &= ~RCC_CFGR_PLLSRC;
	RCC_CFGR &= ~RCC_CFGR_PLLSRC0;

	//PLL clock source - HSE
	RCC_CFGR |= RCC_CFGR_PLLSRC;

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	//Switch SYSCLK to be clocked by PLL
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 48000000;
	rcc_ahb_frequency = 48000000;

	SystemCoreClock = 48000000;
}

void vSetLEDS(uint8_t mask, uint8_t value)
{
	GPIOC_BRR = (uint16_t)((mask & GPIOC_LEDS_MASK) << GPIOC_LEDS_SHIFT);
	GPIOC_BSRR = (uint32_t)((value & GPIOC_LEDS_MASK) << GPIOC_LEDS_SHIFT);
}


