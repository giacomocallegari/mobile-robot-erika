/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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
#define RESET_LASER1_GPIO_Port GPIOE
#define RESET_LASER2_Pin GPIO_PIN_12
#define RESET_LASER2_GPIO_Port GPIOE
#define IMU_RESET_Pin GPIO_PIN_15
#define IMU_RESET_GPIO_Port GPIOE
#define WIFI_RESET_Pin GPIO_PIN_12
#define WIFI_RESET_GPIO_Port GPIOB
#define ENABLE_MOTOR_Pin GPIO_PIN_13
#define ENABLE_MOTOR_GPIO_Port GPIOB
#define LED_WHITE_Pin GPIO_PIN_12
#define LED_WHITE_GPIO_Port GPIOD
#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOD
#define GND_DETECT_Pin GPIO_PIN_8
#define GND_DETECT_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
