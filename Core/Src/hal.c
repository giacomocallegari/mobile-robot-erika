/* ###*B*###
 * Erika Enterprise, version 3
 * 
 * Copyright (C) 2017 - 2018 Evidence s.r.l.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License, version 2, for more details.
 * 
 * You should have received a copy of the GNU General Public License,
 * version 2, along with this program; if not, see
 * < www.gnu.org/licenses/old-licenses/gpl-2.0.html >.
 * 
 * This program is distributed to you subject to the following
 * clarifications and special exceptions to the GNU General Public
 * License, version 2.
 * 
 * THIRD PARTIES' MATERIALS
 * 
 * Certain materials included in this library are provided by third
 * parties under licenses other than the GNU General Public License. You
 * may only use, copy, link to, modify and redistribute this library
 * following the terms of license indicated below for third parties'
 * materials.
 * 
 * In case you make modified versions of this library which still include
 * said third parties' materials, you are obligated to grant this special
 * exception.
 * 
 * The complete list of Third party materials allowed with ERIKA
 * Enterprise version 3, together with the terms and conditions of each
 * license, is present in the file THIRDPARTY.TXT in the root of the
 * project.
 * ###*E*### */

/** \file	hal.c
 *  \brief	Hardware Abstraction Layer Sources.
 *
 *  This file contains the sources of the HAL for Erika Enterprise.
 *
 *  \author	Giuseppe Serano
 *  \date	2018
 */

/* Header */
#include "hal.h"

#ifdef	OS_EE_LIB_STM32_CUBE_F4
/* STM32F4 DISCOVERY BSP. */
#include "stm32f4_discovery.h"
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */

#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
#include "stm32f4xx.h"
#define	HAL_TIMER_PERIOD_MS	1U
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */

#ifdef	OS_EE_LIB_STM32_CUBE_F4
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static FUNC(void, APPL_CODE) SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	/* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
	if (HAL_GetREVID() == 0x1001)
	{
		/* Enable the Flash prefetch */
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
	}
}	/* SystemClock_Config() */
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */

FUNC(void, APPL_CODE) DemoHAL_Init( void )
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	HAL_Init();

	SystemClock_Config();
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */

#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
	/* Update SystemCoreClock variable according to Clock Register Values. */
	SystemCoreClockUpdate();

	SysTick_Config(HAL_TIMER_PERIOD_MS * (SystemCoreClock / 1000U));
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */

#if (defined(OSEE_CORTEX_M_TIM3_ISR) && defined(OSEE_SYSTEM_TIMER))
	DemoHAL_TimerInit(HAL_TIMER_PERIOD_MS);
#endif
}

#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
static volatile VAR(MemSize, AUTOMATIC) TimingDelay;

/**
  * @brief  Inserts a delay time.
  * @param  interval: specifies the delay time length, in milliseconds.
  * @retval None
  */
static FUNC(void, APPL_CODE) HAL_Delay(
	VAR(MemSize, AUTOMATIC)	interval
)
{
	  TimingDelay = interval;

	  while(TimingDelay != 0U);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
static FUNC(void, APPL_CODE) TimingDelay_Decrement(void)
{
  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
}
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */

#if (defined(OSEE_CORTEX_M_TIM3_ISR) && defined(OSEE_SYSTEM_TIMER))
ISR1(HALTickISR)
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	HAL_IncTick();
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
	TimingDelay_Decrement();
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
	DemoHAL_TimerAck();
}

FUNC(void, APPL_CODE) DemoHAL_Delay(
	VAR(MemSize, AUTOMATIC)	interval
)
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	HAL_Delay(interval);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
}
#else
#if (defined(OSEE_SYSTEM_TIMER) && defined(OSEE_SYSTEM_TIMER_DEVICE) && (OSEE_SYSTEM_TIMER_DEVICE == OSEE_CORTEX_M_SYSTICK))
FUNC(void, APPL_CODE) DemoHAL_Delay(
	VAR(MemSize, AUTOMATIC)	interval
)
{
	TickType	start, tmp, elapsed;

	GetCounterValue(OSEE_SYSTEM_TIMER, &start);

	do {
		tmp = start;
		GetElapsedValue(OSEE_SYSTEM_TIMER, &tmp, &elapsed);
	} while (elapsed < interval);
}
#else
#if	(								\
		!(							\
			defined(OSEE_CORTEX_M_SYSTICK_ISR_TID) &&	\
			(OSEE_CORTEX_M_SYSTICK_ISR_CAT == 2)		\
		) && !(							\
			defined(OSEE_CORTEX_M_SYSTICK_ISR) &&		\
			(OSEE_CORTEX_M_SYSTICK_ISR_CAT == 1)		\
		)							\
)
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
FUNC(void, APPL_CODE) SysTick_Handler(void)
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	HAL_IncTick();
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
	TimingDelay_Decrement();
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
}
#endif	/* OSEE_CORTEX_M_SYSTICK_ISR */

FUNC(void, APPL_CODE) DemoHAL_Delay(
	VAR(MemSize, AUTOMATIC)	interval
)
{
	HAL_Delay(interval);
}
#endif
#endif

FUNC(void, APPL_CODE) DemoHAL_MainFunction( void ) {}

/* Leds HAL */
#define	HAL_LCD_LED1_STR	(uint8_t *)"1\r\n"
#define	HAL_LCD_LED2_STR	(uint8_t *)"2\r\n"
#define	HAL_LCD_LED3_STR	(uint8_t *)"3\r\n"
#define	HAL_LCD_LED4_STR	(uint8_t *)"4\r\n"
#define	HAL_LCD_LED5_STR	(uint8_t *)"5\r\n"
#define	HAL_LCD_LED6_STR	(uint8_t *)"6\r\n"
#define	HAL_LCD_LED7_STR	(uint8_t *)"7\r\n"
#define	HAL_LCD_LED8_STR	(uint8_t *)"8\r\n"
#define HAL_LCD_LED_STR_SZ	3U

static CONSTP2VAR(uint8_t, OS_APPL_DATA, OS_APPL_DATA) halLedArray[DEMO_HAL_BUTTON_NUM] = {
	HAL_LCD_LED1_STR, HAL_LCD_LED2_STR, HAL_LCD_LED3_STR, HAL_LCD_LED4_STR,
	HAL_LCD_LED5_STR, HAL_LCD_LED6_STR, HAL_LCD_LED7_STR, HAL_LCD_LED8_STR
};

#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
#define	LEDS_PORT	GPIOD

#define	LED3_PIN	GPIO_Pin_13
#define	LED4_PIN	GPIO_Pin_12
#define	LED5_PIN	GPIO_Pin_14
#define	LED6_PIN	GPIO_Pin_15
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */

FUNC(void, APPL_CODE) DemoHAL_LedInit( void )
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);

	DemoHAL_SerialInit();
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* GPIOG Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PG6 and PG8 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LEDS_PORT, &GPIO_InitStructure);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
}

FUNC(void, APPL_CODE) DemoHAL_LedOn(
	VAR(DemoHAL_Led, AUTOMATIC)				led
)
{
	DemoHAL_SerialWrite(halLedArray[led], HAL_LCD_LED_STR_SZ);

	switch (led) {
	case DEMO_HAL_LED_0:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED3);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED3_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_1:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED3);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED3_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_2:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED4);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED4_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_3:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED4);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED4_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_4:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED5);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED5_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_5:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED5);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED5_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_6:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED6);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED6_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_7:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_On(LED6);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED6_PIN, Bit_SET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	default:
		break;
	}
}

FUNC(void, APPL_CODE) DemoHAL_LedOff(
	VAR(DemoHAL_Led, AUTOMATIC)	led
)
{
	switch (led) {
	case DEMO_HAL_LED_0:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED3);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED3_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_1:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED3);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED3_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_2:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED4);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED4_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_3:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED4);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED4_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_4:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED5);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED5_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_5:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED5);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED5_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_6:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED6);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED6_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_7:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Off(LED6);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_WriteBit(LEDS_PORT, LED6_PIN, Bit_RESET);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	default:
		break;
	}
}

FUNC(void, APPL_CODE) DemoHAL_LedToggle(
	VAR(DemoHAL_Led, AUTOMATIC)				led
)
{
	switch (led) {
	case DEMO_HAL_LED_0:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED3);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED3_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_1:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED3);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED3_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_2:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED4);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED4_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_3:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED4);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED4_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_4:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED5);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED5_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_5:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED5);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED5_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_6:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED6);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED6_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	case DEMO_HAL_LED_7:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		BSP_LED_Toggle(LED6);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
		GPIO_ToggleBits(LEDS_PORT, LED6_PIN);
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
		break;
	default:
		break;
	}
}

/* Buttons HAL */
#ifdef	OS_EE_LIB_S32_SDK

#endif	/* OS_EE_LIB_S32_SDK */

FUNC(void, APPL_CODE) DemoHAL_ButtonInit( void )
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	/* Configure USER Button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
}

FUNC(OsEE_bool, APPL_CODE) DemoHAL_ButtonRead(
	VAR(DemoHAL_Button, AUTOMATIC)	button
) {
	VAR(OsEE_bool, AUTOMATIC)		value = 0U;
	switch (button) {
	case DEMO_HAL_BUTTON_0:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		value = (BSP_PB_GetState(BUTTON_KEY) == RESET);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_1:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		value = (BSP_PB_GetState(BUTTON_KEY) == RESET);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_2:
	case DEMO_HAL_BUTTON_3:
	case DEMO_HAL_BUTTON_4:
	case DEMO_HAL_BUTTON_5:
	case DEMO_HAL_BUTTON_6:
	case DEMO_HAL_BUTTON_7:
	default:
		break;
	}
	return !value;
}

FUNC(void, APPL_CODE) DemoHAL_ButtonInterruptEnable(
	VAR(DemoHAL_Button, AUTOMATIC)	button
) {
	switch (button) {
	case DEMO_HAL_BUTTON_0:
#ifdef	OS_EE_LIB_STM32_CUBE_F4

#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_1:
#ifdef	OS_EE_LIB_STM32_CUBE_F4

#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_2:
	case DEMO_HAL_BUTTON_3:
	case DEMO_HAL_BUTTON_4:
	case DEMO_HAL_BUTTON_5:
	case DEMO_HAL_BUTTON_6:
	case DEMO_HAL_BUTTON_7:
	default:
		break;
	}
}

FUNC(void, APPL_CODE) DemoHAL_ButtonInterruptDisable(
	VAR(DemoHAL_Button, AUTOMATIC)	button
) {
	switch (button) {
	case DEMO_HAL_BUTTON_0:
#ifdef	OS_EE_LIB_STM32_CUBE_F4

#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_1:
#ifdef	OS_EE_LIB_STM32_CUBE_F4

#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_2:
	case DEMO_HAL_BUTTON_3:
	case DEMO_HAL_BUTTON_4:
	case DEMO_HAL_BUTTON_5:
	case DEMO_HAL_BUTTON_6:
	case DEMO_HAL_BUTTON_7:
	default:
		break;
	}
}

FUNC(void, APPL_CODE) DemoHAL_ButtonInterruptAck(
	VAR(DemoHAL_Button, AUTOMATIC)	button
) {
	switch (button) {
	case DEMO_HAL_BUTTON_0:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_1:
#ifdef	OS_EE_LIB_STM32_CUBE_F4
		HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
		break;
	case DEMO_HAL_BUTTON_2:
	case DEMO_HAL_BUTTON_3:
	case DEMO_HAL_BUTTON_4:
	case DEMO_HAL_BUTTON_5:
	case DEMO_HAL_BUTTON_6:
	case DEMO_HAL_BUTTON_7:
	default:
		break;
	}
}

/* Timer HAL */
#ifdef	OS_EE_LIB_STM32_CUBE_F4
/* User can use this section to tailor TIMx instance used and associated
   resources */
/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM3_CLK_ENABLE

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler

static VAR(TIM_HandleTypeDef, OS_APPL_DATA)	TimHandle;
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
static VAR(TIM_TimeBaseInitTypeDef, OS_APPL_DATA)	TIM_TimeBaseStructure;
static VAR(TIM_OCInitTypeDef, OS_APPL_DATA)			TIM_OCInitStructure;
static __IO VAR(uint16_t, OS_APPL_DATA)				CCR1_Val = 40961;
static __IO VAR(uint16_t, OS_APPL_DATA)				CCR2_Val = 27309;
static __IO VAR(uint16_t, OS_APPL_DATA)				CCR3_Val = 13654;
static __IO VAR(uint16_t, OS_APPL_DATA)				CCR4_Val = 6826;
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */

FUNC(void, APPL_CODE) DemoHAL_TimerInit(
	VAR(MemSize, AUTOMATIC)					period
) {
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	VAR(uint16_t, AUTOMATIC)				uwPrescalerValue = 0;

	/*##-0- Enable peripherals and GPIO Clocks #################################*/
	/* TIMx Peripheral clock enable */
	TIMx_CLK_ENABLE();

	/*##-1- Configure the TIM peripheral #######################################*/
	/* -----------------------------------------------------------------------
	   In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	   since APB1 prescaler is different from 1.
	     TIM3CLK = 2 * PCLK1
	     PCLK1 = HCLK / 4
	     => TIM3CLK = HCLK / 2 = SystemCoreClock /2
	   To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
	   Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	   Prescaler = ((SystemCoreClock /2) /10 KHz) - 1

	   Note:
	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	     variable value. Otherwise, any configuration based on this variable will be incorrect.
	     This variable is updated in three ways:
	      1) by calling CMSIS function SystemCoreClockUpdate()
	      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
	      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
	  ----------------------------------------------------------------------- */

	/* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / (period * 1000000)) - 1;

	/* Set TIMx instance */
	TimHandle.Instance = TIMx;

	/* Initialize TIM3 peripheral as follows:
	   + Period = 10000 - 1
	   + Prescaler = ((SystemCoreClock/2)/10000) - 1
	   + ClockDivision = 0
	   + Counter direction = Up
	 */
	TimHandle.Init.Period = (period * 1000) - 1;
	TimHandle.Init.Prescaler = uwPrescalerValue;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	{
		/* Initialization Error */
		DemoHAL_LedOn(DEMO_HAL_LED_1);
		for(;;);
	}

	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
	{
		/* Starting Error */
		DemoHAL_LedOn(DEMO_HAL_LED_1);
		for(;;);
	}
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
#if	0
	SysTick_Config(period*(SystemCoreClock / 1000));
#else
	VAR(uint16_t, AUTOMATIC)	PrescalerValue = 0;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* -----------------------------------------------------------------------
	    TIM3 Configuration: Output Compare Timing Mode:

	    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	    since APB1 prescaler is different from 1.
	      TIM3CLK = 2 * PCLK1
	      PCLK1 = HCLK / 4
	      => TIM3CLK = HCLK / 2 = SystemCoreClock /2

	    To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
	       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	       Prescaler = ((SystemCoreClock /2) /6 MHz) - 1

	    CC1 update rate = TIM3 counter clock / CCR1_Val = 146.48 Hz
	    ==> Toggling frequency = 73.24 Hz

	    C2 update rate = TIM3 counter clock / CCR2_Val = 219.7 Hz
	    ==> Toggling frequency = 109.8 Hz

	    CC3 update rate = TIM3 counter clock / CCR3_Val = 439.4 Hz
	    ==> Toggling frequency = 219.7 Hz

	    CC4 update rate = TIM3 counter clock / CCR4_Val = 878.9 Hz
	    ==> Toggling frequency = 439.4 Hz

	    Note:
	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	     function to update SystemCoreClock variable value. Otherwise, any configuration
	     based on this variable will be incorrect.
	  ----------------------------------------------------------------------- */

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 6000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
#endif
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
}	/* DemoHAL_TimerInit() */

FUNC(void, APPL_CODE) DemoHAL_TimerDelay(
	VAR(MemSize, AUTOMATIC)	interval
) { (void)interval; }

FUNC(void, APPL_CODE) DemoHAL_TimerAck( void )
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	HAL_TIM_IRQHandler(&TimHandle);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
	VAR(uint16_t, AUTOMATIC)	capture = 0;

	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		capture = TIM_GetCapture1(TIM3);
		TIM_SetCompare1(TIM3, capture + CCR1_Val);
	}
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		capture = TIM_GetCapture2(TIM3);
		TIM_SetCompare2(TIM3, capture + CCR2_Val);
	}
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		capture = TIM_GetCapture3(TIM3);
		TIM_SetCompare3(TIM3, capture + CCR3_Val);
	}
	else
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		capture = TIM_GetCapture4(TIM3);
		TIM_SetCompare4(TIM3, capture + CCR4_Val);
	}
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
}	/* DemoHAL_TimerAck() */

/* Serial HAL */
#ifdef	OS_EE_LIB_STM32_CUBE_F4
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

static VAR(UART_HandleTypeDef, OS_APPL_DATA)	UartHandle;
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */

ISR1(SerialISR)
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	HAL_UART_IRQHandler(&UartHandle);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
}

FUNC(void, APPL_CODE) DemoHAL_SerialInit( void )
{
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*##-0- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	/* Enable USART1 clock */
	USARTx_CLK_ENABLE();

	/*##-1- Configure peripheral GPIO ##########################################*/
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USARTx_TX_AF;

	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;

	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

	/*##-2- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
	   - Word Length = 8 Bits
	   - Stop Bit = One Stop bit
	   - Parity = None
	   - BaudRate = 9600 baud
	   - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance          = USARTx;

	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		DemoHAL_LedOn(DEMO_HAL_LED_1);
		for(;;);
	}
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
}

FUNC(void, APPL_CODE) DemoHAL_SerialWrite(
	P2CONST(uint8_t, AUTOMATIC, OS_APPL_DATA)		buffer,
	VAR(MemSize, AUTOMATIC)					length
) {
#ifdef	OS_EE_LIB_STM32F4XX_DSP_SPL
	(void)buffer;
	(void)length;
#endif	/* OS_EE_LIB_STM32F4XX_DSP_SPL */
#ifdef	OS_EE_LIB_STM32_CUBE_F4
	HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)buffer, length);
#endif	/* OS_EE_LIB_STM32_CUBE_F4 */
}
