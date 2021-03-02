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

/** \file hal.h
 *  \brief  Main header.
 *
 *  This file contains the header of the main source file.
 *
 *  \author Giacomo Callegari
 *  \date 2021
 */

#ifndef __MAIN_H
#define __MAIN_H

#if (defined(__cplusplus))
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

#define CHANNEL_A_M1_Pin GPIO_PIN_0
#define CHANNEL_A_M1_GPIO_Port GPIOC
#define CHANNEL_A_M1_EXTI_IRQn EXTI0_IRQn
#define CHANNEL_B_M1_Pin GPIO_PIN_1
#define CHANNEL_B_M1_GPIO_Port GPIOC
#define CHANNEL_B_M1_EXTI_IRQn EXTI1_IRQn
#define CHANNEL_B_M2_Pin GPIO_PIN_2
#define CHANNEL_B_M2_GPIO_Port GPIOC
#define CHANNEL_B_M2_EXTI_IRQn EXTI2_IRQn
#define RESET_LASER3_Pin GPIO_PIN_4
#define RESET_LASER3_GPIO_Port GPIOA
#define RESET_LASER4_Pin GPIO_PIN_5
#define RESET_LASER4_GPIO_Port GPIOA
#define B_IN_1_Pin GPIO_PIN_6
#define B_IN_1_GPIO_Port GPIOA
#define B_IN_2_Pin GPIO_PIN_7
#define B_IN_2_GPIO_Port GPIOA
#define CHANNEL_A_M2_Pin GPIO_PIN_4
#define CHANNEL_A_M2_GPIO_Port GPIOC
#define CHANNEL_A_M2_EXTI_IRQn EXTI4_IRQn
#define A_IN_1_Pin GPIO_PIN_0
#define A_IN_1_GPIO_Port GPIOB
#define A_IN_2_Pin GPIO_PIN_1
#define A_IN_2_GPIO_Port GPIOB
#define RESET_LASER1_Pin GPIO_PIN_11
#define RESET_LASER1_GPIO_Port GPIOF
#define RESET_LASER2_Pin GPIO_PIN_12
#define RESET_LASER2_GPIO_Port GPIOF
#define IMU_RESET_Pin GPIO_PIN_15
#define IMU_RESET_GPIO_Port GPIOE
#define WIFI_RESET_Pin GPIO_PIN_12
#define WIFI_RESET_GPIO_Port GPIOB
#define ENABLE_MOTOR_Pin GPIO_PIN_13
#define ENABLE_MOTOR_GPIO_Port GPIOB
#define LED_WHITE_Pin GPIO_PIN_8
#define LED_WHITE_GPIO_Port GPIOD
#define LED_STATUS_Pin GPIO_PIN_9
#define LED_STATUS_GPIO_Port GPIOD
#define GND_DETECT_Pin GPIO_PIN_8
#define GND_DETECT_GPIO_Port GPIOA

#if (defined(__cplusplus))
}
#endif

#endif  /* __MAIN_H */
