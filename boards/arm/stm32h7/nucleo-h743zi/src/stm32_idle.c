/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_idle.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * WHY A CUSTOM IDLE (WFI)
 *
 * The common arm_idle.c leaves up_idle() empty (its WFI is #if 0'd), so the
 * idle task returns immediately and the scheduler busy-spins through the idle
 * loop. With ETM instruction trace enabled that busy-spin is traced FOREVER --
 * it dominated the stream (~16.8M nx_start instruction ranges in one 3 s
 * capture) and pinned the 4-bit parallel port at its 50 MB/s ceiling, stalling
 * the CPU and eventually overflowing the ETF.
 *
 * Executing WFI in idle puts the core to sleep until the next interrupt, so
 * the ETM emits essentially nothing while idle. The trace then contains only
 * the real thread activity we care about (nsh, the churn workers, switches),
 * which is exactly what the FPGA/cortrace thread-switch demo needs and keeps
 * the byte-rate well under the port ceiling.
 *
 * Enabled via CONFIG_ARCH_IDLE_CUSTOM=y (which drops the common up_idle).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  Sleep in WFI until an interrupt occurs so the idle
 *   loop is not traced by the ETM.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If interrupts are suppressed, process "fake" timer interrupts so the
   * system can still make progress. */

  nxsched_process_timer();
#else
  /* Sleep until the next interrupt. WFI holds the core clock off until an
   * interrupt is pending; nothing executes, so the ETM emits nothing while
   * idle. */

  asm volatile ("wfi");
#endif
}
