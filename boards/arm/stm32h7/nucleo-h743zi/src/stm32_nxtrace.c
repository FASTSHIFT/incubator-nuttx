/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_nxtrace.c
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
 * ZERO-INTRUSION PARALLEL ETM + DWT THREAD-SWITCH TRACE BRING-UP
 *
 * This programs the STM32H743 CoreSight trace path for capture by an external
 * Artix-7 FPGA over the 4-bit parallel trace port (TRACECK + TRACED0..3 on
 * PE2..PE6). It is a NuttX port of the bare-metal bring-up in the sibling
 * `stm32h743-etm-trace-firmware` (etm_selftrace.c / etm_regs.h), adapted to
 * the NuttX clock tree (SYSCLK 400 MHz, PLL1R -> TRACECLKIN 100 MHz) and the
 * getreg32/putreg32 accessors.
 *
 * Two trace streams share the one parallel TPIU, merged by the CoreSight
 * funnel and tagged by ATB ID so the host (cortrace) can demux them:
 *   - ETM  (ATB ID 2): the instruction stream. Programmed with BB=0 to keep
 *                      the byte-rate well under the port ceiling at 400 MHz.
 *   - DWT  (ATB ID 1): data-value packets emitted on every WRITE to
 *                      g_running_tasks. The payload IS the new TCB pointer, so
 *                      the host sees each thread switch WITHOUT reading target
 *                      memory (no UAF, short-lived threads never missed).
 *
 * The whole point is ZERO INTRUSION: this touches only debug/trace registers
 * and the trace GPIOs. It never modifies the scheduler, the TCB, or any
 * kernel data. See docs/01-nuttx-thread-profiling.md in cortrace-fpga.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/sched.h>           /* g_running_tasks[] */
#include <nuttx/kthread.h>         /* kthread_create */

#include "arm_internal.h"          /* getreg32 / putreg32 */

#ifdef CONFIG_NUCLEO_H743ZI_NXTRACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CoreSight software-lock magic. */

#define CS_LAR_UNLOCK        0xc5acce55u
#define CS_LAR_OFFSET        0xfb0u

/* DEMCR (core PPB): global trace enable. */

#define NXT_DEMCR            0xe000edfcu
#define NXT_DEMCR_TRCENA     (1u << 24)

/* DBGMCU_CR (H7 D-domain APB-D alias): gate the trace clocks. */

#define NXT_DBGMCU_CR        0x5c001004u
#define NXT_DBGMCU_CR_TRACECLKEN (1u << 20)
#define NXT_DBGMCU_CR_D1DBGCKEN  (1u << 21)
#define NXT_DBGMCU_CR_D3DBGCKEN  (1u << 22)
#define NXT_DBGMCU_CR_TRACE_ALL  (NXT_DBGMCU_CR_TRACECLKEN | \
                                  NXT_DBGMCU_CR_D1DBGCKEN | \
                                  NXT_DBGMCU_CR_D3DBGCKEN)

/* GPIOE PE2..PE6 = TRACECK / TRACED0..3 (AF0), very-high speed. */

#define NXT_RCC_AHB4ENR      0x580244e0u
#define NXT_RCC_AHB4ENR_GPIOEEN (1u << 4)
#define NXT_RCC_APB4ENR      0x580244f4u
#define NXT_RCC_APB4ENR_SYSCFGEN (1u << 1)

#define NXT_SYSCFG_CCCSR     0x58000420u
#define NXT_SYSCFG_CCCSR_EN    (1u << 0)
#define NXT_SYSCFG_CCCSR_READY (1u << 8)

#define NXT_GPIOE_MODER      0x58021000u
#define NXT_GPIOE_OTYPER     0x58021004u
#define NXT_GPIOE_OSPEEDR    0x58021008u
#define NXT_GPIOE_AFRL       0x58021020u

/* SoC-level TPIU (APB-D alias). */

#define NXT_TPIU_BASE        0x5c015000u
#define NXT_TPIU_CURPSIZE    (NXT_TPIU_BASE + 0x004u)
#define NXT_TPIU_SPPR        (NXT_TPIU_BASE + 0x0f0u)
#define NXT_TPIU_FFCR        (NXT_TPIU_BASE + 0x304u)
#define NXT_TPIU_ITCTRL      (NXT_TPIU_BASE + 0x204u)
#define NXT_TPIU_LAR         (NXT_TPIU_BASE + CS_LAR_OFFSET)
#define NXT_TPIU_CURPSIZE_4BIT 0x00000008u
#define NXT_TPIU_SPPR_PARALLEL 0x00000000u
#define NXT_TPIU_FFCR_CONT     0x00000102u  /* EnFCont | EnFTC */

/* CoreSight trace funnel (APB-D alias). S0 = ETM; S1/S2 for the ITM/DWT ATB
 * (which slave port it lands on is undocumented -- enable both, harmless). */

#define NXT_CSTF_BASE        0x5c013000u
#define NXT_CSTF_CTRL        (NXT_CSTF_BASE + 0x000u)
#define NXT_CSTF_LAR         (NXT_CSTF_BASE + CS_LAR_OFFSET)
#define NXT_CSTF_CTRL_ENS0     (1u << 0)
#define NXT_CSTF_CTRL_ENS1     (1u << 1)
#define NXT_CSTF_CTRL_ENS2     (1u << 2)

/* Embedded Trace FIFO (TMC, APB-D alias): ETM -> CSTF -> ETF -> TPIU. */

#define NXT_ETF_BASE         0x5c014000u
#define NXT_ETF_CTL          (NXT_ETF_BASE + 0x020u)
#define NXT_ETF_MODE         (NXT_ETF_BASE + 0x028u)
#define NXT_ETF_FFCR         (NXT_ETF_BASE + 0x304u)
#define NXT_ETF_LAR          (NXT_ETF_BASE + CS_LAR_OFFSET)
#define NXT_ETF_CTL_TRACECAPTEN 0x00000001u
#define NXT_ETF_MODE_HW_FIFO    0x00000002u
#define NXT_ETF_FFCR_ENFT       0x00000001u

/* ETM-M7 (ETMv4, core-local). */

#define NXT_ETM_BASE         0xe0041000u
#define NXT_ETM_TRCPRGCTLR   (NXT_ETM_BASE + 0x004u)
#define NXT_ETM_TRCSTATR     (NXT_ETM_BASE + 0x00cu)
#define NXT_ETM_TRCCONFIGR   (NXT_ETM_BASE + 0x010u)
#define NXT_ETM_TRCSTALLCTLR (NXT_ETM_BASE + 0x02cu)
#define NXT_ETM_TRCSYNCPR    (NXT_ETM_BASE + 0x034u)
#define NXT_ETM_TRCTSCTLR    (NXT_ETM_BASE + 0x030u)
#define NXT_ETM_TRCCCCTLR    (NXT_ETM_BASE + 0x038u)
#define NXT_ETM_TRCTRACEIDR  (NXT_ETM_BASE + 0x040u)
#define NXT_ETM_TRCVICTLR    (NXT_ETM_BASE + 0x080u)
#define NXT_ETM_TRCPDCR      (NXT_ETM_BASE + 0x310u)
#define NXT_ETM_LAR          (NXT_ETM_BASE + CS_LAR_OFFSET)

#define NXT_ETM_TRCPRGCTLR_EN   (1u << 0)
#define NXT_ETM_TRCSTATR_IDLE   (1u << 0)
#define NXT_ETM_TRCCONFIGR_BB   (1u << 3)
#define NXT_ETM_TRCCONFIGR_CCI  (1u << 4)
#define NXT_ETM_TRCCONFIGR_TS   (1u << 11)
#define NXT_ETM_TRCSTALLCTLR_ISTALL (1u << 8)
#define NXT_ETM_TRCVICTLR_TRACE_ALL 0x00000201u
#define NXT_ETM_TRCTRACEIDR_ID2 0x00000002u
#define NXT_ETM_TRCSYNCPR_4K    0x0000000cu
#define NXT_ETM_TRCCCCTLR_THRESHOLD 0x00000010u
#define NXT_ETM_TRCPDCR_PU      (1u << 3)

/* DWT + ITM (core PPB). */

#define NXT_DWT_BASE         0xe0001000u
#define NXT_DWT_COMP0        (NXT_DWT_BASE + 0x020u)
#define NXT_DWT_MASK0        (NXT_DWT_BASE + 0x024u)
#define NXT_DWT_FUNCTION0    (NXT_DWT_BASE + 0x028u)
#define NXT_DWT_FUNCTION_DATAVWRITE 0x0000000du /* data-value packet on write */

#define NXT_ITM_BASE         0xe0000000u
#define NXT_ITM_TER          (NXT_ITM_BASE + 0xe00u)
#define NXT_ITM_TCR          (NXT_ITM_BASE + 0xe80u)
#define NXT_ITM_LAR          (NXT_ITM_BASE + CS_LAR_OFFSET)
#define NXT_ITM_TCR_ITMENA     (1u << 0)
#define NXT_ITM_TCR_TSENA      (1u << 1)
#define NXT_ITM_TCR_SYNCENA    (1u << 2)
#define NXT_ITM_TCR_TXENA      (1u << 3)
#define NXT_ITM_TCR_TRACEBUSID_1 (1u << 16)
#define NXT_ITM_TCR_DWT_ATB    (NXT_ITM_TCR_ITMENA | NXT_ITM_TCR_TSENA | \
                                NXT_ITM_TCR_SYNCENA | NXT_ITM_TCR_TXENA | \
                                NXT_ITM_TCR_TRACEBUSID_1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NUCLEO_H743ZI_NXTRACE_HEARTBEAT
void stm32_nxtrace_heartbeat_start(void);
#endif

/****************************************************************************
 * Public Functions
 *
 * DWT watches the address of g_running_tasks[0] (from <nuttx/sched.h>). We
 * only ever take its address -- never dereference it -- so the trace path
 * stays fully decoupled from scheduler internals (zero intrusion).
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_nxtrace_setup
 *
 * Description:
 *   Program GPIO + DBGMCU + TPIU + CSTF + ETF + ETM + DWT/ITM for parallel
 *   ETM instruction trace plus DWT thread-switch data trace. Runs once at
 *   board bring-up. Purely a debug-register operation; zero scheduler impact.
 *
 ****************************************************************************/

void stm32_nxtrace_setup(void)
{
  volatile int i;

  /* 0a) SYSCFG I/O compensation cell -- VERY_HIGH-speed pads only reach their
   * rated slew with it enabled (needed for clean parallel trace edges).
   */

  modifyreg32(NXT_RCC_APB4ENR, 0, NXT_RCC_APB4ENR_SYSCFGEN);
  (void)getreg32(NXT_RCC_APB4ENR);
  modifyreg32(NXT_SYSCFG_CCCSR, 0, NXT_SYSCFG_CCCSR_EN);
  for (i = 0; i < 10000; i++)
    {
      if (getreg32(NXT_SYSCFG_CCCSR) & NXT_SYSCFG_CCCSR_READY)
        {
          break;
        }
    }

  /* 0) GPIOE PE2..PE6 -> AF0, very-high speed, push-pull. */

  modifyreg32(NXT_RCC_AHB4ENR, 0, NXT_RCC_AHB4ENR_GPIOEEN);
  modifyreg32(NXT_GPIOE_MODER, (0x3fffu << 4),
              (0x2u << 4) | (0x2u << 6) | (0x2u << 8) |
              (0x2u << 10) | (0x2u << 12));
  modifyreg32(NXT_GPIOE_OSPEEDR, 0,
              (0x3u << 4) | (0x3u << 6) | (0x3u << 8) |
              (0x3u << 10) | (0x3u << 12));
  modifyreg32(NXT_GPIOE_OTYPER, 0x7cu, 0);
  modifyreg32(NXT_GPIOE_AFRL, (0xfffffu << 8), 0);

  /* 1) core trace power + 2) DBGMCU trace clocks. */

  putreg32(NXT_DEMCR_TRCENA, NXT_DEMCR);
  modifyreg32(NXT_DBGMCU_CR, 0, NXT_DBGMCU_CR_TRACE_ALL);

  /* 3) TPIU: 4-bit parallel + continuous formatter, no test pattern. */

  putreg32(CS_LAR_UNLOCK, NXT_TPIU_LAR);
  putreg32(NXT_TPIU_CURPSIZE_4BIT, NXT_TPIU_CURPSIZE);
  putreg32(NXT_TPIU_SPPR_PARALLEL, NXT_TPIU_SPPR);
  putreg32(NXT_TPIU_FFCR_CONT, NXT_TPIU_FFCR);
  putreg32(0, NXT_TPIU_ITCTRL);

  /* 3b) CSTF funnel: S0 = ETM, S1/S2 opened for the ITM/DWT ATB. */

  putreg32(CS_LAR_UNLOCK, NXT_CSTF_LAR);
  modifyreg32(NXT_CSTF_CTRL, 0,
              NXT_CSTF_CTRL_ENS0 | NXT_CSTF_CTRL_ENS1 | NXT_CSTF_CTRL_ENS2);

  /* 3c) ETF: hardware-FIFO mode -> TPIU. */

  putreg32(0, NXT_ETF_CTL);
  putreg32(CS_LAR_UNLOCK, NXT_ETF_LAR);
  putreg32(NXT_ETF_MODE_HW_FIFO, NXT_ETF_MODE);
  putreg32(NXT_ETF_FFCR_ENFT, NXT_ETF_FFCR);
  putreg32(NXT_ETF_CTL_TRACECAPTEN, NXT_ETF_CTL);

  /* 4) ETM: BB=0 (low byte-rate, safe at 400 MHz), global timestamp + cycle
   * counting on so the host has an execution-time base. Trace ALL.
   */

  putreg32(CS_LAR_UNLOCK, NXT_ETM_LAR);
  putreg32(NXT_ETM_TRCPDCR_PU, NXT_ETM_TRCPDCR);
  putreg32(0, NXT_ETM_TRCPRGCTLR);
  for (i = 0; i < 1000; i++)
    {
      if (getreg32(NXT_ETM_TRCSTATR) & NXT_ETM_TRCSTATR_IDLE)
        {
          break;
        }
    }

  putreg32(NXT_ETM_TRCCONFIGR_TS | NXT_ETM_TRCCONFIGR_CCI, NXT_ETM_TRCCONFIGR);
  putreg32(NXT_ETM_TRCTRACEIDR_ID2, NXT_ETM_TRCTRACEIDR);
  putreg32(NXT_ETM_TRCCCCTLR_THRESHOLD, NXT_ETM_TRCCCCTLR);
  /* Keep ISTALL as a lossless backstop but LEVEL=0 so timestamps aren't
   * starved (DDI0494D 3.4.7). */

  putreg32(NXT_ETM_TRCSTALLCTLR_ISTALL, NXT_ETM_TRCSTALLCTLR);
  putreg32(NXT_ETM_TRCVICTLR_TRACE_ALL, NXT_ETM_TRCVICTLR);
  putreg32(NXT_ETM_TRCSYNCPR_4K, NXT_ETM_TRCSYNCPR);
  putreg32(0, NXT_ETM_TRCTSCTLR);
  putreg32(NXT_ETM_TRCPRGCTLR_EN, NXT_ETM_TRCPRGCTLR);

  /* 5) DWT data-value trace: emit a data-value packet (payload = written TCB
   * pointer) on every WRITE to g_running_tasks. ITM forwards DWT packets to
   * the ATB with a non-zero TraceBusID (stream 1). FUNCTION 0x0D, not 0x07
   * (0x07 is a watchpoint that halts the CPU).
   */

  putreg32(CS_LAR_UNLOCK, NXT_ITM_LAR);
  putreg32(NXT_ITM_TCR_DWT_ATB, NXT_ITM_TCR);

  putreg32(0, NXT_DWT_FUNCTION0);
  putreg32((uint32_t)(uintptr_t)&g_running_tasks[0], NXT_DWT_COMP0);
  putreg32(0, NXT_DWT_MASK0);   /* exact-address match */
  putreg32(NXT_DWT_FUNCTION_DATAVWRITE, NXT_DWT_FUNCTION0);

  syslog(LOG_INFO,
         "nxtrace: parallel ETM(id2)+DWT(id1) up; watch g_running_tasks@%p\n",
         (void *)&g_running_tasks[0]);

#ifdef CONFIG_NUCLEO_H743ZI_NXTRACE_HEARTBEAT
  stm32_nxtrace_heartbeat_start();
#endif
}

#ifdef CONFIG_NUCLEO_H743ZI_NXTRACE_HEARTBEAT

/****************************************************************************
 * Name: nxtrace_heartbeat
 *
 * Description:
 *   A tiny always-on kernel thread that just sleeps in a loop. Its only job
 *   is to guarantee a steady stream of context switches (idle <-> heartbeat)
 *   WITHOUT any debugger attached, so the DWT g_running_tasks watchpoint has
 *   writes to capture even when the system is otherwise quiescent (nsh idle).
 *   This lets the FPGA capture the thread-switch demo standalone -- important
 *   because attaching a debugger for RTT would clear the DWT comparator.
 *
 *   The demo's real workload (hellotrace churn workers) rides on top of this.
 ****************************************************************************/

static int nxtrace_heartbeat(int argc, char *argv[])
{
  for (; ; )
    {
      /* Sleep a short period; each wake/sleep is a context switch that writes
       * g_running_tasks, firing the DWT data-trace packet. 20 ms = 50 Hz, low
       * overhead, dense enough to prove the path. */

      usleep(20 * 1000);
    }

  return 0;
}

/****************************************************************************
 * Name: stm32_nxtrace_heartbeat_start
 ****************************************************************************/

void stm32_nxtrace_heartbeat_start(void)
{
  int pid = kthread_create("nxtrace_hb", SCHED_PRIORITY_DEFAULT,
                           1024, nxtrace_heartbeat, NULL);
  if (pid < 0)
    {
      syslog(LOG_ERR, "nxtrace: heartbeat thread create failed: %d\n", pid);
    }
}

#endif /* CONFIG_NUCLEO_H743ZI_NXTRACE_HEARTBEAT */

#endif /* CONFIG_NUCLEO_H743ZI_NXTRACE */
