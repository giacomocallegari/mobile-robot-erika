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
 *  \author	Giuseppe Serano
 *  \date	2018
 */

/* ERIKA Enterprise. */
#include "ee.h"

/* HAL */
#include "hal.h"

#include <stdio.h>
#include <string.h>

OsEE_bool volatile stk_wrong = OSEE_FALSE;
OsEE_addr volatile old_sp;
uint32_t volatile idle_cnt;

DeclareTask(Task1);
extern void idle_hook(void);
extern void StartupHook(void);

#define	IDLE_CNT_MAX	1000000U

#define	IDLE_STR	(P2CONST(uint8_t, AUTOMATIC, OS_APPL_DATA))"Idle\r\n"

void StartupHook(void)
{
	DemoHAL_SerialInit();
}

void serial_print(char const * msg)
{
  SuspendAllInterrupts();
  DemoHAL_SerialWrite((uint8_t const *)msg, strlen(msg));
  ResumeAllInterrupts();
}

void print_sp(TaskType tid, OsEE_addr sp)
{
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

void idle_hook(void)
{
	if ( !stk_wrong ) {
		if (!old_sp) {
			old_sp = osEE_get_SP();
		}
		else if (old_sp != osEE_get_SP()) {
			stk_wrong = OSEE_TRUE;
			OSEE_BREAK_POINT();
		}
	}

	++idle_cnt;
	if (idle_cnt >= IDLE_CNT_MAX) {
		idle_cnt = 0;
		ActivateTask(Task1);
		DemoHAL_LedToggle(DEMO_HAL_LED_0);
		serial_print(IDLE_STR);
	}

	DemoHAL_MainFunction();
}

int main(void)
{
	DemoHAL_Init();

	DemoHAL_LedInit();

	StartOS(OSDEFAULTAPPMODE);

	return 0;
}
