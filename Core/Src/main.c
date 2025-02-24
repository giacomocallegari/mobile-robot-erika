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

/** \file	code.c
 *  \brief	Main application.
 *
 *  This file contains the code of main application for Erika Enterprise.
 *
 *  \author	Giacomo Callegari
 *  \date	2021
 */

#include "ee.h"

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ROBOT VARIABLES --------------------------------------------------------- */
int counterRelativeEncoder_M1 = 0;
int counterRelativeEncoder_M1Old = 0;
int counterRelativeEncoder_M2 = 0;
int counterRelativeEncoder_M2Old = 0;
int counterDiff_M1 = 0;
int counterDiff_M2 = 0;
GPIO_PinState channelA_M1_old = 0;
GPIO_PinState channelB_M1_old = 0;
GPIO_PinState channelA_M2_old = 0;
GPIO_PinState channelB_M2_old = 0;

int RTOS_dTime = 0;
float lowerMotorLimit = 2000;
float upperMotorLimit = 10000;
float estimatedSpeed_M1 = 0.0;
float estimatedSpeed_M2 = 0.0;
float value_M1 = 0.0;
float value_M2 = 0.0;
float value_old_M1 = 0.0;
float value_old_M2 = 0.0;
float error_M1 = 0.0;
float error_M2 = 0.0;
float error_old_M1 = 0.0;
float error_old_M2 = 0.0;
float integral_M1 = 0.0;
float integral_M2 = 0.0;
float derivative_M1 = 0.0;
float derivative_M2 = 0.0;
/* ------------------------------------------------------------------------- */

/* USART VARIABLES --------------------------------------------------------- */
UART_HandleTypeDef * huartOP = &huart1;
UART_HandleTypeDef * huartUSER = &huart2;

uint8_t temp_serialChar;
uint8_t temp_char;
uint8_t message_usartUSER[512];
uint8_t message_received_usartUSER = 0;
uint8_t message_length_usartUSER = 0;
uint8_t count_usartUSER = 0;
uint8_t receivedCR_usartUSER = 0;
uint8_t data_buffer[128];

uint8_t temp_usartOP;
uint8_t message_usartOP[512];
uint8_t message_received_usartOP = 0;
uint8_t message_length_usartOP = 0;

char usart_buffer[128];
/* ------------------------------------------------------------------------- */

/* ROBOT PARAMETERS -------------------------------------------------------- */
float omegaDes = 0.3;  // Desired angular velocity of the robot
float velDes = 0.05;  // Desired linear velocity of the robot

float velDes_M1 = 0.0;  // Desired angular velocity of motor 1
float velDes_M2 = 0.0;  // Desired angular velocity of motor 2

float tickToTheta = 360.0 / (26);  // Angular distance between two ticks
float Rr = 210;  // Gear ratio
float borderVel = 0.01;
float targetVel = 0.04;
float wheelRadius = 0.023 / 2.0;  // Radius of each wheel; 0.22 works
float rearTrack = 0.144;  // Distance between the wheels; 0.1455 works
float theta_des = 0.0;  // Desired angle of the robot
float vehicleLength = 0.11;  // Length of the robot
float vehicleWidth = 0.15;  // Width of the robot

float fromDegSToRPM = 60.0 / 360.0;  // Conversion from degrees to RPM
float fromRPMtoRads = 2.0 * M_PI / 60.0;  // Conversion from RPM to radians
/* ------------------------------------------------------------------------- */

/* HELPER FUNCTIONS -------------------------------------------------------- */
void PWM_Set(uint32_t value, uint32_t Channel);
void resetPWM();
char DEBUG_usart_getchar(void);
int DEBUG_usart_print(char* buffer);
/* ------------------------------------------------------------------------- */

/* ERIKA FUNCTIONS AND VARIABLES ------------------------------------------- */
OsEE_bool volatile stk_wrong = OSEE_FALSE;
OsEE_addr volatile old_sp;
uint32_t volatile idle_cnt;
uint16_t volatile TaskControl_count;

DeclareTask(TaskReceiveCMD);
DeclareTask(TaskSendInfo);
DeclareTask(TaskControl);
extern void idle_hook(void);
void SystemClock_Config(void);

extern SemType SemaphoreVariabiliIngresso;
extern SemType SemaphoreVariabiliUscita;
extern SemType SemaphoreDataBuffer;

#define	IDLE_CNT_MAX	1000000U
#define TASK_RECEIVECMD_PERIOD 200U   // Default: 200
#define TASK_SENDINFO_PERIOD 200U     // Default: 200
#define TASK_CONTROL_PERIOD 20U       // Default: 20

#define OSEE_BREAK_POINT()  do {                                    \
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);                         \
    DisableAllInterrupts();                                         \
    while ( 1 ) {                                                   \
      ;                                                             \
    }                                                               \
  } while ( 0 )

// Currently disabled
void idle_hook(void) {
  if (!stk_wrong) {
    if (!old_sp) {
      old_sp = osEE_get_SP();
    } else if (old_sp != osEE_get_SP()) {
      stk_wrong = OSEE_TRUE;
      OSEE_BREAK_POINT()
      ;
    }
  }

  ++idle_cnt;
  if (idle_cnt >= IDLE_CNT_MAX) {
    idle_cnt = 0;
    /*ActivateTask(TaskReceiveCMD);
    ActivateTask(TaskSendInfo);
    ActivateTask(TaskControl);*/
  }
}
/* ------------------------------------------------------------------------- */

/**
 * Main function.
 *
 * @return
 */
int main(void) {
  // Initialize the system.
  HAL_Init();
  SystemClock_Config();

  // Initialize the peripherals.
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  // Start the PWM generation.
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // Enable the motors and turn on the LED.
  HAL_GPIO_WritePin(ENABLE_MOTOR_GPIO_Port, ENABLE_MOTOR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);

  // Set all PWM channels to zero.
  PWM_Set(0, TIM_CHANNEL_1);
  PWM_Set(0, TIM_CHANNEL_2);
  PWM_Set(0, TIM_CHANNEL_3);
  PWM_Set(0, TIM_CHANNEL_4);

  HAL_Delay(1000);

  // Receive data from UART in non-blocking mode.
  HAL_UART_Receive_IT(huartUSER, &temp_serialChar, 1);

  // Start the RTOS.
  StartOS(OSDEFAULTAPPMODE);

  return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE()
  ;
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // Find the channel and update the counter depending on the direction.
  if (GPIO_Pin == CHANNEL_A_M1_Pin) {
    if (channelA_M1_old == channelB_M1_old) {
      counterRelativeEncoder_M1++;
    } else {
      counterRelativeEncoder_M1--;
    }
    channelA_M1_old = HAL_GPIO_ReadPin(CHANNEL_A_M1_GPIO_Port, CHANNEL_A_M1_Pin);
  } else if (GPIO_Pin == CHANNEL_B_M1_Pin) {
    if (channelA_M1_old != channelB_M1_old) {
      counterRelativeEncoder_M1++;
    } else {
      counterRelativeEncoder_M1--;
    }
    channelB_M1_old = HAL_GPIO_ReadPin(CHANNEL_B_M1_GPIO_Port, CHANNEL_B_M1_Pin);
  } else if (GPIO_Pin == CHANNEL_A_M2_Pin) {
    if (channelA_M2_old == channelB_M2_old) {
      counterRelativeEncoder_M2++;
    } else {
      counterRelativeEncoder_M2--;
    }
    channelA_M2_old = HAL_GPIO_ReadPin(CHANNEL_A_M2_GPIO_Port, CHANNEL_A_M2_Pin);
  } else if (GPIO_Pin == CHANNEL_B_M2_Pin) {
    if (channelA_M2_old != channelB_M2_old) {
      counterRelativeEncoder_M2++;
    } else {
      counterRelativeEncoder_M2--;
    }
    channelB_M2_old = HAL_GPIO_ReadPin(CHANNEL_B_M2_GPIO_Port, CHANNEL_B_M2_Pin);
  }
  return;
}

/**
 * Task for the reception of commands.
 */
TASK(TaskReceiveCMD) {
  char temp;
  char buffer[50];
  char stringVel[10];
  char stringOmega[10];
  float vel = 0.0;
  float omega = 0.0;

  //for (;;) {
    /*HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
    HAL_Delay(50);*/

    // Read the command from the buffer.
    WaitSem(&SemaphoreDataBuffer);
    memcpy(buffer, data_buffer, 50 * sizeof(char));
    PostSem(&SemaphoreDataBuffer);

    // Copy the message into the local buffer.
    sscanf(buffer, "%s %s", stringVel, stringOmega);

    // Extract the values from the buffer.
    vel = atof(stringVel);
    omega = atof(stringOmega);

    // Set the desired linear and angular velocity for the robot.
    WaitSem(&SemaphoreVariabiliIngresso);
    velDes = vel;
    omegaDes = omega;
    PostSem(&SemaphoreVariabiliIngresso);

    /*HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);*/
    HAL_Delay(TASK_RECEIVECMD_PERIOD);

    ActivateTask(TaskSendInfo);
  //}

  TerminateTask();
}

/**
 *  Task for the transmission of information.
 */
TASK(TaskSendInfo) {
  //for (;;) {
    /*HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_Delay(50);*/

    WaitSem(&SemaphoreVariabiliUscita);
    sprintf(usart_buffer, "rpm vr %d - vl %d\r\n", (int) estimatedSpeed_M1, (int) estimatedSpeed_M2);
    DEBUG_usart_print(usart_buffer);
    PostSem(&SemaphoreVariabiliUscita);

    /*HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);*/
    HAL_Delay(TASK_SENDINFO_PERIOD);

    ActivateTask(TaskControl);
  //}

  TerminateTask();
}

/**
 * Task for the control of the robot.
 */
TASK(TaskControl) {
  float dt = 0.02;

  // Set the parameters for the PID controller.
  float kp = 0.06;
  float ki = 0.00;
  float kd = 0.01;

  // Get the current time.
  int initialTime = HAL_GetTick();

  int decimator = 0; // Cycle counter

  // Terms for PID control
  float integral_M1 = 0.0;
  float derivative_M1 = 0.0;

  // Desired velocity for each motor
  float omegaR = 0.0;
  float omegaL = 0.0;

  //for (;;) {
    /*HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);
    HAL_Delay(50);*/

    // Toggle the LED every 10 iterations.
    if (decimator++ % 10 == 0) {
      HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);
    }

    // Compute the tick difference from the previous iteration.
    counterDiff_M1 = counterRelativeEncoder_M1 - counterRelativeEncoder_M1Old;
    counterDiff_M2 = counterRelativeEncoder_M2 - counterRelativeEncoder_M2Old;
    counterRelativeEncoder_M1Old = counterRelativeEncoder_M1;
    counterRelativeEncoder_M2Old = counterRelativeEncoder_M2;

    // Estimate the speed in RPM of each motor.
    WaitSem(&SemaphoreVariabiliUscita);
    estimatedSpeed_M1 = ((float) counterDiff_M1 / dt) * tickToTheta * fromDegSToRPM;
    estimatedSpeed_M2 = -((float) counterDiff_M2 / dt) * tickToTheta * fromDegSToRPM;
    PostSem(&SemaphoreVariabiliUscita);

    // Find the desired velocity for each wheel.
    WaitSem(&SemaphoreVariabiliIngresso);
    omegaR = 0.5 * (omegaDes * rearTrack + 2.0 * velDes) / wheelRadius;
    omegaL = 0.5 * (-omegaDes * rearTrack + 2.0 * velDes) / wheelRadius;
    PostSem(&SemaphoreVariabiliIngresso);

    // Convert the desired velocities to RPM.
    velDes_M2 = omegaL * Rr * 60.0 / (2.0 * 3.14);
    velDes_M1 = omegaR * Rr * 60.0 / (2.0 * 3.14);

    // Use PID to get the control value for motor 1.
    error_M1 = velDes_M1 - estimatedSpeed_M1;
    integral_M1 = integral_M1 + error_M1 * dt;
    derivative_M1 = (error_M1 - error_old_M1) / dt;
    value_M1 = value_old_M1 + kp * (error_M1) + ki * integral_M1 + kd * derivative_M1;

    // Use PID to get the control value for motor 2.
    error_M2 = velDes_M2 - estimatedSpeed_M2;
    integral_M2 = integral_M2 + error_M2 * dt;
    derivative_M2 = (error_M2 - error_old_M2) / dt;
    value_M2 = value_old_M2 + kp * (error_M2) + ki * integral_M2 + kd * derivative_M2;

    // Find the sign for the velocities.
    float signM1 = 1.0;
    float signM2 = 1.0;
    if (velDes_M1 > 0) {
      signM1 = 1.0;
    } else {
      signM1 = -1.0;
    }
    if (velDes_M2 > 0) {
      signM2 = 1.0;
    } else {
      signM2 = -1.0;
    }

    // Saturate the PWM values within the limits.
    if (fabs(value_M1) > upperMotorLimit) {
      value_M1 = upperMotorLimit;
    } else if (fabs(value_M1) < lowerMotorLimit) {
      value_M1 = lowerMotorLimit;
    }
    if (fabs(value_M2) > upperMotorLimit) {
      value_M2 = upperMotorLimit;
    } else if (fabs(value_M2) < lowerMotorLimit) {
      value_M2 = lowerMotorLimit;
    }
    value_M1 = fabs(value_M1) * signM1;
    value_M2 = fabs(value_M2) * signM2;

    // Set the PWM for motor 1.
    if (value_M1 > 0) {
      PWM_Set((uint32_t) (fabs(value_M1)), TIM_CHANNEL_4);
      PWM_Set(0, TIM_CHANNEL_3);
    } else {
      PWM_Set(0, TIM_CHANNEL_4);
      PWM_Set((uint32_t) (fabs(value_M1)), TIM_CHANNEL_3);
    }

    // Set the PWM for motor 2.
    if (value_M2 > 0) {
      PWM_Set((uint32_t) (fabs(value_M2)), TIM_CHANNEL_1);
      PWM_Set(0, TIM_CHANNEL_2);
    } else {
      PWM_Set(0, TIM_CHANNEL_1);
      PWM_Set((uint32_t) (fabs(value_M2)), TIM_CHANNEL_2);
    }

    // Update the variables for motor 1.
    error_old_M1 = error_M1;
    value_old_M1 = value_M1;

    // Update the variables for motor 2.
    error_old_M2 = error_M2;
    value_old_M2 = value_M2;

    // Sleep until the next activation.
    /*HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_RESET);*/
    HAL_Delay(TASK_CONTROL_PERIOD);

    ActivateTask(TaskReceiveCMD);
  //}

  TerminateTask();
}

/**
 * Resets the PWM variables.
 */
void resetPWM() {
  error_M1 = 0;
  error_M2 = 0;
  integral_M1 = 0;
  integral_M2 = 0;
  derivative_M1 = 0;
  derivative_M2 = 0;
  error_old_M1 = 0;
  error_old_M2 = 0;
  value_M1 = 0;
  value_M2 = 0;
  value_old_M1 = 0;
  value_old_M2 = 0;
}

/**
 * Sets the PWM value on the selected channel.
 *
 * @param value     The PWM value
 * @param Channel   The output channel
 */
void PWM_Set(uint32_t value, uint32_t Channel) {
  // Apply PWM to the selected channel.
  switch (Channel) {
  case TIM_CHANNEL_1: {
    htim3.Instance->CCR1 = value;
    break;
  }
  case TIM_CHANNEL_2: {
    htim3.Instance->CCR2 = value;
    break;
  }
  case TIM_CHANNEL_3: {
    htim3.Instance->CCR3 = value;
    break;
  }
  case TIM_CHANNEL_4: {
    htim3.Instance->CCR4 = value;
    break;
  }
  }
}

/**
 * Callback that is invoked at the completion of the UART reception.
 *
 * @param huart   The UART handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == huartUSER) {
    if (temp_serialChar != 0x0d && temp_serialChar != 0x0a) {  // Normal character received
      // Add a new character to the buffer.
      message_usartUSER[count_usartUSER++] = temp_serialChar;
    } else if (receivedCR_usartUSER == 0 && temp_serialChar == 0x0d) {  // CR received
      // Set the flag for the Carriage Return.
      receivedCR_usartUSER = 1;
    } else if (receivedCR_usartUSER == 1 && temp_serialChar == 0x0a) {  // LF received
      // Set the flag for the message reception.
      message_length_usartUSER = count_usartUSER;
      message_received_usartUSER = 1;

      // Write the command to the buffer.
      WaitSem(&SemaphoreDataBuffer);
      memcpy(data_buffer, message_usartUSER, count_usartUSER * sizeof(uint8_t));
      PostSem(&SemaphoreDataBuffer);

      // Reset the variables.
      count_usartUSER = 0;
      receivedCR_usartUSER = 0;
    } else {  // Invalid character received
      // Reset the variables.
      count_usartUSER = 0;
      receivedCR_usartUSER = 0;
      message_received_usartUSER = 0;
    }

    // Receive data from UART in non-blocking mode.
    HAL_UART_Receive_IT(huartUSER, &temp_serialChar, 1);
  } else if (huart == huartOP) {
  }
}

/**
 * Sends a string to a buffer through USART.
 *
 * @param buffer  The destination buffer
 * @return
 */
int DEBUG_usart_print(char* buffer) {
  return 0;

  if (HAL_UART_Transmit(&huart1, (uint8_t *) buffer, strlen(buffer), 1000) != HAL_OK) {
    return -1;
  }

  return 0;
}

/**
 * Receives a single character through USART.
 *
 * @return
 */
char DEBUG_usart_getchar(void) {
  char rxChar;

  if (HAL_UART_Receive(&huart1, (uint8_t *) &rxChar, 1, 100) != HAL_OK) {
    return '\0';
  }

  return rxChar;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* User can add his own implementation to report the HAL error return state */
}
