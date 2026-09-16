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
 * ZERO-INTRUSION THREAD-SWITCH TRACE OVER THE PARALLEL TRACE PORT
 *
 * This brings up the STM32H743 CoreSight parallel-trace path (ETM instruction
 * stream + DWT data-value packets) so an external FPGA (A7-Lite) can capture
 * both on the 4-bit TRACED[3:0] port and cortrace can demux them:
 *
 *   ETM  --ATB--> CSTF S0 --\
 *                            +--> ETF --> TPIU --> TRACECK/TRACED[3:0]
 *   DWT/ITM --ATB-> CSTF S1 -/
 *
 * A DWT comparator watches WRITES to g_running_tasks; the hardware emits the
 * *written value* (the new TCB pointer) as a data-value packet at the instant
 * of the context switch -- so no scheduler code is touched and short-lived
 * threads are never missed (doc 01 §3/§4/§11.8).
 *
 * IMPORTANT: this is pure board-level bring-up -- GPIO mux + CoreSight trace
 * register programming. It does NOT modify the scheduler, the TCB layout, or
 * any RTOS data structure. The DWT comparator is a passive watchpoint in
 * data-value mode (FUNCTION=0x0D), NOT a debug watchpoint, so it never halts
 * the CPU. Zero RTOS impact.
 *
 * Register map / bit values are the on-board-validated sequence from the
 * bare-metal firmware (stm32h743-etm-trace-firmware, etm_selftrace.c) and
 * RM0433 §60 / DDI0494D. See cortrace-fpga/docs/01-nuttx-thread-profiling.md.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <syslog.h>
#include <unistd.h>
#include <sched.h>

#include <nuttx/kthread.h>

#include "arm_internal.h"          /* getreg32 / putreg32 / modifyreg32 */
#include "stm32_gpio.h"            /* stm32_configgpio + GPIO_* macros */

#ifdef CONFIG_NUCLEO_H743ZI_NXTRACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The kernel symbol whose writes mark a context switch is g_running_tasks[]
 * (declared in <nuttx/sched.h>, included via arm_internal.h -> nuttx/arch.h):
 * g_running_tasks[cpu] is written with the new TCB pointer on every switch, so
 * watching its address yields the new TCB pointer in the DWT stream. */

#include <nuttx/sched.h>

/* CoreSight software-lock magic (write to a component's LAR at +0xFB0). */

#define CS_LAR_UNLOCK 0xc5acce55u

/* --- GPIOE PE2..PE6 = TRACECK / TRACED0..3, AF0, very-high speed --------- */

#define GPIO_TRACECK  (GPIO_ALT | GPIO_AF0 | GPIO_PUSHPULL | \
                       GPIO_SPEED_100MHz | GPIO_PORTE | GPIO_PIN2)
#define GPIO_TRACED0  (GPIO_ALT | GPIO_AF0 | GPIO_PUSHPULL | \
                       GPIO_SPEED_100MHz | GPIO_PORTE | GPIO_PIN3)
#define GPIO_TRACED1  (GPIO_ALT | GPIO_AF0 | GPIO_PUSHPULL | \
                       GPIO_SPEED_100MHz | GPIO_PORTE | GPIO_PIN4)
#define GPIO_TRACED2  (GPIO_ALT | GPIO_AF0 | GPIO_PUSHPULL | \
                       GPIO_SPEED_100MHz | GPIO_PORTE | GPIO_PIN5)
#define GPIO_TRACED3  (GPIO_ALT | GPIO_AF0 | GPIO_PUSHPULL | \
                       GPIO_SPEED_100MHz | GPIO_PORTE | GPIO_PIN6)

/* --- DBGMCU trace clock gate (H7 D-domain APB-D alias) ------------------- */

#define DBGMCU_CR        0x5c001004u
#define DBGMCU_CR_TRACE_ALL ((1u << 20) | (1u << 21) | (1u << 22))

/* --- core PPB: DEMCR global trace enable -------------------------------- */

#define DEMCR            0xe000edfcu
#define DEMCR_TRCENA       (1u << 24)

/* --- SoC TPIU (APB-D alias 0x5C015000) ---------------------------------- */

#define TPIU_BASE        0x5c015000u
#define TPIU_CURPSIZE    (TPIU_BASE + 0x004u)
#define TPIU_SPPR        (TPIU_BASE + 0x0f0u)
#define TPIU_FFCR        (TPIU_BASE + 0x304u)
#define TPIU_LAR         (TPIU_BASE + 0xfb0u)
#define TPIU_CURPSIZE_4BIT 0x00000008u
#define TPIU_SPPR_PARALLEL 0x00000000u
#define TPIU_FFCR_CONT     0x00000102u

/* --- CoreSight trace funnel (CSTF, 0x5C013000): S0=ETM, S1=ITM/DWT ------- */

#define CSTF_BASE        0x5c013000u
#define CSTF_CTRL        (CSTF_BASE + 0x000u)
#define CSTF_PRIORITY    (CSTF_BASE + 0x004u)
#define CSTF_LAR         (CSTF_BASE + 0xfb0u)
#define CSTF_CTRL_ENS0     (1u << 0)
#define CSTF_CTRL_ENS1     (1u << 1)
#define CSTF_PRIPORT0_MASK 0x00000007u
#define CSTF_PRIPORT1_MASK 0x00000038u

/* --- Embedded trace FIFO (ETF/TMC, 0x5C014000) -------------------------- */

#define ETF_BASE         0x5c014000u
#define ETF_CTL          (ETF_BASE + 0x020u)
#define ETF_MODE         (ETF_BASE + 0x028u)
#define ETF_FFCR         (ETF_BASE + 0x304u)
#define ETF_LAR          (ETF_BASE + 0xfb0u)
#define ETF_CTL_TRACECAPTEN 0x00000001u
#define ETF_MODE_HW_FIFO    0x00000002u
#define ETF_FFCR_ENFT       0x00000001u

/* --- ETM-M7 (ETMv4, core-local 0xE0041000) ------------------------------ */

#define ETM_BASE         0xe0041000u
#define ETM_TRCPRGCTLR   (ETM_BASE + 0x004u)
#define ETM_TRCSTATR     (ETM_BASE + 0x00cu)
#define ETM_TRCCONFIGR   (ETM_BASE + 0x010u)
#define ETM_TRCSTALLCTLR (ETM_BASE + 0x02cu)
#define ETM_TRCTSCTLR    (ETM_BASE + 0x030u)
#define ETM_TRCSYNCPR    (ETM_BASE + 0x034u)
#define ETM_TRCCCCTLR    (ETM_BASE + 0x038u)
#define ETM_TRCTRACEIDR  (ETM_BASE + 0x040u)
#define ETM_TRCVICTLR    (ETM_BASE + 0x080u)
#define ETM_TRCPDCR      (ETM_BASE + 0x310u)
#define ETM_LAR          (ETM_BASE + 0xfb0u)
#define ETM_TRCPRGCTLR_EN   (1u << 0)
#define ETM_TRCSTATR_IDLE   (1u << 0)
#define ETM_TRCCONFIGR_BB   (1u << 3)
#define ETM_TRCCONFIGR_CCI  (1u << 4)   /* cycle counting instruction */
#define ETM_TRCCONFIGR_TS   (1u << 11)  /* global timestamping */
#define ETM_TRCCCCTLR_THRESHOLD 0x00000010u   /* 16 cycles (>= TRCIDR3.CCITMIN) */
#define ETM_TRCSTALLCTLR_ISTALL (1u << 8)
#define ETM_TRCVICTLR_TRACE_ALL 0x00000201u
#define ETM_TRCTRACEIDR_ID2     0x00000002u
#define ETM_TRCSYNCPR_4K        0x0000000cu
#define ETM_TRCPDCR_PU          (1u << 3)

/* --- CoreSight timestamp generator (TSGEN, APB-D alias 0x5C005000) -------- */
/* The SoC-wide counter the ETM samples into TIMESTAMP packets. Resets DISABLED
 * (CNTCR=0), so every ETM timestamp reads 0 until CNTCR.EN is set. */

#define TSGEN_CNTCR      0x5c005000u
#define TSGEN_CNTCR_EN     (1u << 0)

/* --- DWT + ITM (core PPB) ------------------------------------------------ */

#define DWT_BASE         0xe0001000u
#define DWT_CTRL_REG     (DWT_BASE + 0x000u)
#define DWT_COMP0        (DWT_BASE + 0x020u)
#define DWT_MASK0        (DWT_BASE + 0x024u)
#define DWT_FUNCTION0    (DWT_BASE + 0x028u)
#define DWT_CYCCNT_REG   (DWT_BASE + 0x004u)
#define DWT_CTRL_CYCCNTENA 0x40000001u
#define DWT_FUNCTION_DATAVWRITE 0x0000000du   /* data-value packet on write */

#define ITM_BASE         0xe0000000u
#define ITM_TER          (ITM_BASE + 0xe00u)
#define ITM_TPR          (ITM_BASE + 0xe40u)
#define ITM_TCR          (ITM_BASE + 0xe80u)
#define ITM_LAR          (ITM_BASE + 0xfb0u)
/* ITMENA|TSENA|SYNCENA|TXENA, TraceBusID = 1 (host demux: ETM=2, DWT/ITM=1). */
#define ITM_TCR_DWT_ATB    0x0001000fu

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void nxtrace_gpio(void)
{
  stm32_configgpio(GPIO_TRACECK);
  stm32_configgpio(GPIO_TRACED0);
  stm32_configgpio(GPIO_TRACED1);
  stm32_configgpio(GPIO_TRACED2);
  stm32_configgpio(GPIO_TRACED3);
}

/* Two ping-pong churn threads so the scheduler switches frequently from boot,
 * giving the DWT a steady stream of g_running_tasks writes to capture WITHOUT
 * any manual trigger. Each does a little work then sleeps a short interval, so
 * idle, churn_a and churn_b rotate continuously. Pure demo workload; remove or
 * gate off for production. */

static volatile uint32_t g_churn_sink;

static int nxtrace_churn(int argc, char *argv[])
{
  /* Tight ping-pong: a little work then sched_yield() to the equal-priority
   * peer, so the two churn threads (+ occasionally idle) rotate CONTINUOUSLY
   * and g_running_tasks is written on every switch -- a steady DWT stream, not
   * bursts separated by long idle. No usleep: sleeping drops the CPU to idle
   * for milliseconds and starves the DWT stream. */

  for (; ; )
    {
      /* Re-arm DWT comparator 0 every loop: if anything (debug monitor init,
       * a debugger connect, etc.) cleared it after boot, this restores it
       * within microseconds so the switch stream never dries up. Cheap: three
       * register writes. Harmless if already armed. */

      if (getreg32(DWT_FUNCTION0) != DWT_FUNCTION_DATAVWRITE ||
          getreg32(DWT_COMP0) != (uint32_t)(uintptr_t)&g_running_tasks[0])
        {
          putreg32(0, DWT_FUNCTION0);
          putreg32((uint32_t)(uintptr_t)&g_running_tasks[0], DWT_COMP0);
          putreg32(0, DWT_MASK0);
          putreg32(DWT_FUNCTION_DATAVWRITE, DWT_FUNCTION0);
        }

      for (volatile int i = 0; i < 200; i++)
        {
          g_churn_sink += i;
        }

      sched_yield();   /* round-robin to the equal-priority peer -> switch */
    }

  return 0;
}

static void nxtrace_spawn_churn(void)
{
  /* Equal priority so SCHED_RR / sched_yield round-robins between them. */

  kthread_create("churn_a", 100, 1024, nxtrace_churn, NULL);
  kthread_create("churn_b", 100, 1024, nxtrace_churn, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_nxtrace_setup
 *
 * Description:
 *   Bring up the parallel trace path (ETM + DWT/ITM -> CSTF -> ETF -> TPIU ->
 *   TRACED[3:0]) and arm a DWT comparator on g_running_tasks so context
 *   switches emit the new TCB pointer as a data-value packet. Pure board-level
 *   CoreSight programming; the scheduler is untouched.
 *
 ****************************************************************************/

void stm32_nxtrace_setup(void)
{
  volatile int i;

  /* 1) trace pins + clock domain + global trace enable. */

  nxtrace_gpio();
  modifyreg32(DBGMCU_CR, 0, DBGMCU_CR_TRACE_ALL);
  putreg32(DEMCR_TRCENA, DEMCR);

  /* Enable the SoC timestamp generator so ETM TIMESTAMP packets carry a real
   * count (it resets disabled -> every timestamp would otherwise read 0). */

  modifyreg32(TSGEN_CNTCR, 0, TSGEN_CNTCR_EN);

  /* 2) TPIU: 4-bit parallel, continuous formatter. */

  putreg32(CS_LAR_UNLOCK, TPIU_LAR);
  putreg32(TPIU_CURPSIZE_4BIT, TPIU_CURPSIZE);
  putreg32(TPIU_SPPR_PARALLEL, TPIU_SPPR);
  putreg32(TPIU_FFCR_CONT, TPIU_FFCR);

  /* 3) CSTF: S0=ETM + S1=ITM/DWT, ITM priority above ETM (RMW, keep reserved).
   */

  putreg32(CS_LAR_UNLOCK, CSTF_LAR);
  modifyreg32(CSTF_CTRL, 0, CSTF_CTRL_ENS0 | CSTF_CTRL_ENS1);
  modifyreg32(CSTF_PRIORITY, CSTF_PRIPORT0_MASK | CSTF_PRIPORT1_MASK,
              (1u << 0) /* PRIPORT0 ETM = 1 */ | (0u << 3) /* PRIPORT1 ITM = 0 */);

  /* 4) ETF: hardware-FIFO mode -> TPIU. */

  putreg32(0, ETF_CTL);
  putreg32(CS_LAR_UNLOCK, ETF_LAR);
  putreg32(ETF_MODE_HW_FIFO, ETF_MODE);
  putreg32(ETF_FFCR_ENFT, ETF_FFCR);
  putreg32(ETF_CTL_TRACECAPTEN, ETF_CTL);

  /* 5) ETM: BB + STALL(ISTALL) + trace-all, ATB ID 2, 4K sync. */

  putreg32(CS_LAR_UNLOCK, ETM_LAR);
  putreg32(ETM_TRCPDCR_PU, ETM_TRCPDCR);
  putreg32(0, ETM_TRCPRGCTLR);
  for (i = 0; i < 1000; i++)
    {
      if (getreg32(ETM_TRCSTATR) & ETM_TRCSTATR_IDLE)
        {
          break;
        }
    }

  /* Global timestamping (TS) + cycle counting (CCI), NO branch broadcast (BB).
   * BB emits an address on every taken branch -> high byte-rate that overruns
   * the parallel port when combined with TS/CC; without BB only indirect
   * branches emit addresses and the decoder walks the ELF for direct branches,
   * so the stream fits. TS gives sparse execution-time anchors from the TSGEN;
   * CCI emits exact CPU-cycle counts so the host interpolates between anchors at
   * cycle resolution -- the real execution-time base (NOT the FPGA ETF-egress
   * time, which lags through the FIFO). Keep stall as ISTALL only (LEVEL=0): a
   * nonzero stall LEVEL suppresses in-stream timestamps (DDI0494D §3.4.7). */

  putreg32(ETM_TRCCONFIGR_TS | ETM_TRCCONFIGR_CCI, ETM_TRCCONFIGR);
  putreg32(ETM_TRCTRACEIDR_ID2, ETM_TRCTRACEIDR);
  putreg32(ETM_TRCCCCTLR_THRESHOLD, ETM_TRCCCCTLR);
  putreg32(ETM_TRCSTALLCTLR_ISTALL, ETM_TRCSTALLCTLR);
  putreg32(ETM_TRCVICTLR_TRACE_ALL, ETM_TRCVICTLR);
  putreg32(ETM_TRCSYNCPR_4K, ETM_TRCSYNCPR);
  putreg32(0, ETM_TRCTSCTLR);
  putreg32(ETM_TRCPRGCTLR_EN, ETM_TRCPRGCTLR);

  /* 6) DWT data-value trace: watch WRITES to g_running_tasks; ITM forwards the
   * data-value packets onto ATB ID 1. Passive watchpoint (FUNCTION=0x0D), never
   * halts the CPU.
   */

  putreg32(CS_LAR_UNLOCK, ITM_LAR);
  putreg32(0, ITM_TPR);
  putreg32(0xffffffffu, ITM_TER);
  putreg32(ITM_TCR_DWT_ATB, ITM_TCR);

  putreg32(0, DWT_CYCCNT_REG);
  putreg32(DWT_CTRL_CYCCNTENA, DWT_CTRL_REG);
  putreg32(0, DWT_FUNCTION0);                     /* disable while programming */
  putreg32((uint32_t)(uintptr_t)&g_running_tasks[0], DWT_COMP0);
  putreg32(0, DWT_MASK0);                          /* exact address match */
  putreg32(DWT_FUNCTION_DATAVWRITE, DWT_FUNCTION0);

  syslog(LOG_INFO,
         "nxtrace: parallel ETM+DWT up; DWT watch g_running_tasks@%p "
         "(FUNC0=0x%08lx)\n",
         (void *)&g_running_tasks[0],
         (unsigned long)getreg32(DWT_FUNCTION0));

  /* Kick off the churn workload so switches happen continuously from boot. */

  nxtrace_spawn_churn();
}

#endif /* CONFIG_NUCLEO_H743ZI_NXTRACE */
