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

/* ERIKA Enterprise. */
#include "ee.h"

/* HAL */
#include "hal.h"

#include "main.h"
#include "tim.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

OsEE_bool volatile stk_wrong = OSEE_FALSE;
OsEE_addr volatile old_sp;
uint32_t volatile idle_cnt;
uint16_t volatile TaskControl_count;

/* VARIABLES FOR THE ROBOT */

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

UART_HandleTypeDef * huartOP = &huart1;
UART_HandleTypeDef * huartUSER = &huart2;

uint8_t temp_usartUSER;
uint8_t message_usartUSER[512];
uint8_t message_received_usartUSER = 0;
uint8_t message_length_usartUSER = 0;

uint8_t temp_usartOP;
uint8_t message_usartOP[512];
uint8_t message_received_usartOP = 0;
uint8_t message_length_usartOP = 0;

/////////////////////////////////////////////////////

float omegaDes = 0.3;
float velDes = 0.05;

float velDes_M1 = 0.0;
float velDes_M2 = 0.0;

float tickToTheta = 360.0 / (26);
float Rr = 210;
float borderVel = 0.01;
float targetVel = 0.04;
float wheelRadius = 0.023 / 2.0;
float rearTrack = 0.144;
float theta_des = 0.0;
float vehicleLength = 0.11;
float vehicleWidth = 0.15;

float fromDegSToRPM = 60.0 / 360.0;
float fromRPMtoRads = 2.0 * M_PI / 60.0;

/* -------------------------- */

void PWM_Set(uint32_t value, uint32_t Channel);
void resetPWM();

DeclareTask(TaskControl);
extern void idle_hook(void);
extern void StartupHook(void);

#define	IDLE_CNT_MAX	1000000U
#define	IDLE_STR	(P2CONST(uint8_t, AUTOMATIC, OS_APPL_DATA))"Idle\r\n"
#define TASK_STR  (P2CONST(uint8_t, AUTOMATIC, OS_APPL_DATA))"TaskControl\r\n"
#define HAL_DELAY_MS  1000U

void StartupHook(void) {
  DemoHAL_SerialInit();
}

void serial_print(char const * msg) {
  SuspendAllInterrupts();
  DemoHAL_SerialWrite((uint8_t const *) msg, strlen(msg));
  ResumeAllInterrupts();
}

void print_sp(TaskType tid, OsEE_addr sp) {
  static char msg[] = "TASK n SP<0xXXXX>\r\n  ";
  SuspendAllInterrupts();
  sprintf(msg, "TASK %d SP<%p>\r\n", tid, sp);
  ResumeAllInterrupts();
  serial_print(msg);
}

#define OSEE_BREAK_POINT()  do {                                    \
    DemoHAL_LedOn(DEMO_HAL_LED_1);                                  \
    DisableAllInterrupts();                                         \
    serial_print("Test Failed!!!, line:" OSEE_S(__LINE__) " \r\n"); \
    while ( 1 ) {                                                   \
      ;                                                             \
    }                                                               \
  } while ( 0 )

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
    ActivateTask(TaskControl);
    DemoHAL_LedToggle(DEMO_HAL_LED_0);
    serial_print(IDLE_STR);
  }

  DemoHAL_MainFunction();
}

/**
 * Main function.
 *
 * @return
 */
int main(void) {
  // Initialize the peripherals.
  DemoHAL_Init();

  // Start the RTOS.
  StartOS(OSDEFAULTAPPMODE);

  return 0;
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
  //int initialTime = xTaskGetTickCount();

  int decimator = 0;

  float integral_M1 = 0.0;
  float derivative_M1 = 0.0;

  for (;;) {
    // Toggle the LED every 10 iterations.
    if (decimator++ % 10 == 0) {
      DemoHAL_LedToggle(DEMO_HAL_LED_2);
      //HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);
    }

    // Compute the tick difference from the previous iteration.
    counterDiff_M1 = counterRelativeEncoder_M1 - counterRelativeEncoder_M1Old;
    counterDiff_M2 = counterRelativeEncoder_M2 - counterRelativeEncoder_M2Old;
    counterRelativeEncoder_M1Old = counterRelativeEncoder_M1;
    counterRelativeEncoder_M2Old = counterRelativeEncoder_M2;

    // Estimate the speed in RPM of each motor.
    estimatedSpeed_M1 = ((float) counterDiff_M1 / dt) * tickToTheta * fromDegSToRPM;
    estimatedSpeed_M2 = -((float) counterDiff_M2 / dt) * tickToTheta * fromDegSToRPM;

    // Find the desired velocity for each wheel.
    float omegaR = 0.5 * (omegaDes * rearTrack + 2.0 * velDes) / wheelRadius;
    float omegaL = 0.5 * (-omegaDes * rearTrack + 2.0 * velDes) / wheelRadius;

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
    HAL_Delay(20);
    //osDelay(20);
  }

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
    //htim3.Instance->CCR1 = value;  // TODO: Find CCR
    break;
  }
  case TIM_CHANNEL_2: {
    //htim3.Instance->CCR2 = value;
    break;
  }
  case TIM_CHANNEL_3: {
    //htim3.Instance->CCR3 = value;
    break;
  }
  case TIM_CHANNEL_4: {
    //htim3.Instance->CCR4 = value;
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
    HAL_UART_Receive_IT(huartUSER, &temp_usartUSER, 1);
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
