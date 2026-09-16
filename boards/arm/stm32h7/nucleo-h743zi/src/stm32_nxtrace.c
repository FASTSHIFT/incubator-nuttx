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
 * ZERO-INTRUSION THREAD-SWITCH TRACE -- NuttX side is now JUST A DEMO WORKLOAD
 *
 * The whole CoreSight parallel-trace path (GPIOE TRACED pins, TPIU, CSTF, ETF,
 * ETM, TSGEN) AND the DWT comparator that watches g_running_tasks are now
 * configured entirely by the debugger (DAP) from the host -- see
 * cortrace-fpga/host/scripts/nxtrace_dap.cfg. That works because NuttX's
 * arm_enable_dbgmonitor() skips its DWT-clearing init while a debugger is
 * attached (DHCSR.C_DEBUGEN=1), so as long as the DAP stays connected the DWT
 * config survives. This is the TRACE32/J-Trace "probe always attached" model.
 *
 * => NuttX needs NO trace/register bring-up code. This file only spawns two
 * demo worker threads with distinct call graphs so there is something to see
 * (thread switches + per-thread call stacks) in the captured trace. For a real
 * application these are unnecessary; the trace is fully non-intrusive.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <sched.h>
#include <stdint.h>
#include <unistd.h>

#include <nuttx/kthread.h>

#ifdef CONFIG_NUCLEO_H743ZI_NXTRACE

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile uint32_t g_compute_sink;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* compute worker: a small arithmetic call tree (compute_step -> compute_mix). */

static uint32_t compute_mix(uint32_t x)
{
  return (x * 2654435761u) ^ (x >> 15);
}

static uint32_t compute_step(uint32_t acc, uint32_t i)
{
  acc += compute_mix(i);
  acc = compute_mix(acc + i * 3u);
  return acc;
}

static int worker_compute(int argc, char *argv[])
{
  uint32_t acc = 0x1234u;

  for (; ; )
    {
      for (uint32_t i = 0; i < 400; i++)
        {
          acc = compute_step(acc, i);
        }

      g_compute_sink = acc;
      sched_yield();
    }

  return 0;
}

/* fileio worker: write then read back a tmpfs file (pulls in the VFS path). */

static int worker_fileio(int argc, char *argv[])
{
  static char buf[128];
  int n;

  for (; ; )
    {
      int fd = open("/tmp/nxio.dat", O_WRONLY | O_CREAT | O_TRUNC, 0666);
      if (fd >= 0)
        {
          for (n = 0; n < (int)sizeof(buf); n++)
            {
              buf[n] = (char)(n + g_compute_sink);
            }

          write(fd, buf, sizeof(buf));
          close(fd);
        }

      fd = open("/tmp/nxio.dat", O_RDONLY);
      if (fd >= 0)
        {
          read(fd, buf, sizeof(buf));
          close(fd);
        }

      sched_yield();
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_nxtrace_setup
 *
 * Description:
 *   No-op: the trace path + DWT are configured by the debugger (see
 *   nxtrace_dap.cfg). Kept as a stable board hook so stm32_bringup.c does not
 *   need conditional edits.
 *
 ****************************************************************************/

void stm32_nxtrace_setup(void)
{
}

/****************************************************************************
 * Name: stm32_nxtrace_workers_start
 *
 * Description:
 *   Spawn the two demo worker threads (compute + tmpfs file I/O) so the trace
 *   has thread switches + distinct per-thread call stacks to show. Call AFTER
 *   the tmpfs mount (the fileio worker needs /tmp). Equal priority so
 *   sched_yield round-robins them.
 *
 ****************************************************************************/

void stm32_nxtrace_workers_start(void)
{
  kthread_create("compute", 100, 1536, worker_compute, NULL);
  kthread_create("fileio", 100, 2048, worker_fileio, NULL);
}

#endif /* CONFIG_NUCLEO_H743ZI_NXTRACE */
