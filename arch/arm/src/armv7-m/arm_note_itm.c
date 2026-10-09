/****************************************************************************
 * arch/arm/src/armv7-m/arm_note_itm.c
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

/* Note driver that emits sched notes as ITM instrumentation packets.
 *
 * Each note is written to one ITM stimulus port as 32-bit words (then a
 * 16-bit and an 8-bit tail), so the ITM packet payloads of that port,
 * concatenated in order, are exactly the note byte stream that the host
 * note parser (nxtrace) expects -- no extra framing.
 *
 * The ITM itself is configured by the debugger (ITM_TCR.ITMENA, ITM_TER bit
 * for the port).  With no debugger / trace sink configured the driver writes
 * nothing, so it costs only two register reads per note.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/note_itm.h>

#include "arm_internal.h"
#include "itm.h"

#ifdef CONFIG_ARMV7M_NOTE_ITM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NOTEITM_PORT     CONFIG_ARMV7M_NOTE_ITM_PORT
#define NOTEITM_REG      ITM_PORT(NOTEITM_PORT)
#define NOTEITM_TIMEOUT  CONFIG_ARMV7M_NOTE_ITM_TIMEOUT

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct noteitm_s
{
  struct note_driver_s driver;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void noteitm_add(FAR struct note_driver_s *drv,
                        FAR const void *note, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_noteitm_ops =
{
  noteitm_add,
};

static struct noteitm_s g_noteitm =
{
  {
#ifdef CONFIG_SCHED_INSTRUMENTATION_FILTER
    "itm",
    {
      {
        CONFIG_SCHED_INSTRUMENTATION_FILTER_DEFAULT_MODE,
#  ifdef CONFIG_SMP
        CONFIG_SCHED_INSTRUMENTATION_CPUSET
#  endif
      },
    },
#endif
    &g_noteitm_ops
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noteitm_ready
 *
 * Description:
 *   Wait (bounded) for room in the ITM stimulus FIFO.  Reading a stimulus
 *   port returns bit 0 = FIFOREADY.
 *
 * Returned Value:
 *   true if the port can take a write, false if the wait timed out (the
 *   trace sink is backed up; the rest of this note is dropped).
 *
 ****************************************************************************/

static bool noteitm_ready(void)
{
  int n;

  for (n = NOTEITM_TIMEOUT; n > 0; n--)
    {
      if (getreg32(NOTEITM_REG) & 1)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: noteitm_add
 ****************************************************************************/

static void noteitm_add(FAR struct note_driver_s *drv,
                        FAR const void *buf, size_t len)
{
  FAR const uint8_t *p = (FAR const uint8_t *)buf;
  irqstate_t flags;

  /* Nothing to do unless the debugger enabled the ITM and this port. */

  if ((getreg32(ITM_TCR) & ITM_TCR_ITMENA_MASK) == 0 ||
      (getreg32(ITM_TER) & (1ul << NOTEITM_PORT)) == 0)
    {
      return;
    }

  /* Keep one note contiguous: an interrupt that emitted its own note in the
   * middle would interleave bytes on the same port.
   */

  flags = up_irq_save();

  while (len >= 4)
    {
      if (!noteitm_ready())
        {
          goto out;
        }

      putreg32((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24),
               NOTEITM_REG);
      p   += 4;
      len -= 4;
    }

  if (len >= 2)
    {
      if (!noteitm_ready())
        {
          goto out;
        }

      putreg16((uint16_t)p[0] | ((uint16_t)p[1] << 8), NOTEITM_REG);
      p   += 2;
      len -= 2;
    }

  if (len > 0 && noteitm_ready())
    {
      putreg8(p[0], NOTEITM_REG);
    }

out:
  up_irq_restore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: noteitm_register
 ****************************************************************************/

int noteitm_register(void)
{
  return note_driver_register(&g_noteitm.driver);
}

#endif /* CONFIG_ARMV7M_NOTE_ITM */
